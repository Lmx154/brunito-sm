/**
 * FC.cpp - Flight Controller for Brunito Project
 * 
 * This file contains the Flight Controller (FC) module implementation.
 * Phase 4: Implements binary packet reception from NAVC and telemetry handling.
 */

#include <Arduino.h>
#include "../include/utils/Heartbeat.h"
#include "../include/utils/FrameCodec.h"
#include "../include/utils/LoraManager.h"
#include "../include/config/lora.h"
#include "../include/fc/State.h"
#include "../include/fc/CmdParser.h"
#include "../include/fc/UartManager.h"
#include "../include/navc/Sensors.h" // For SensorPacket structure

// Already defined in Heartbeat.h, no need to redefine here
// Using PC13 directly as it's defined by the STM32 framework

// Global objects
HeartbeatManager heartbeat(PC13); // Using PC13 directly (defined by STM32 framework)
StateManager stateManager;
CmdParser cmdParser(stateManager);
LoraManager loraManager;
UartManager uartManager; // For binary packet reception from NAVC

// Task scheduling variables
unsigned long lastStatusUpdate = 0;
static const uint32_t STATUS_PERIOD_MS = 10'000;   // 10 s everywhere
unsigned long lastHeartbeatUpdate = 0;
unsigned long lastLoraCheck = 0;
unsigned long lastCommandTime = 0; // Used for optimizing command ACK processing
unsigned long lastUartCheck = 0;   // For UART packet processing
unsigned long lastNavcStatsReport = 0; // For reporting NAVC packet statistics

void cmdTask() {
  // Process any incoming serial data (from USB-CDC) 
  static String cdcCmdBuffer;
  static bool cdcCmdInProgress = false;
  
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Begin collecting a command when '<' is received
    if (c == '<') {
      cdcCmdBuffer = c;
      cdcCmdInProgress = true;
      
    // Complete command processing when '>' is received
    } else if (c == '>' && cdcCmdInProgress) {
      cdcCmdBuffer += c;
      cdcCmdInProgress = false;
      
      // Check for LoRa stats reset command (special case)
      if (cdcCmdBuffer.equals("<CMD:LORA_RESET_STATS>")) {
        if (loraManager.isInitialized()) {
          loraManager.resetStats();
          char buffer[64];
          FrameCodec::formatDebug(buffer, sizeof(buffer), "LORA_STATS_RESET");
          Serial.println(buffer);
          
          // Also send to GS
          if (loraManager.isInitialized()) {
            loraManager.sendPacket(LORA_TYPE_CMD, "<CMD_ACK:OK:LORA_STATS_RESET>", 30);
          }
        }
      }
      // Check for LoRa stats command
      else if (cdcCmdBuffer.equals("<CMD:LORA_STATS>")) {
        if (loraManager.isInitialized()) {
          char buffer[128];
          snprintf(buffer, sizeof(buffer), 
                  "<CMD_ACK:OK:LORA_STATS,RSSI:%d,SNR:%.1f,TX:%u,RX:%u,LOST:%u,LOSS:%.1f%%>",
                  loraManager.getLastRssi(),
                  loraManager.getLastSnr(),
                  loraManager.getPacketsSent(),
                  loraManager.getPacketsReceived(),
                  loraManager.getPacketsLost(),
                  loraManager.getPacketLossRate() * 100.0f);
          Serial.println(buffer);
          
          // Also send to GS
          if (loraManager.isInitialized()) {
            loraManager.sendPacket(LORA_TYPE_CMD, buffer, strlen(buffer));
          }
        }
      }
      // Check if it's a command
      else if (cdcCmdBuffer.startsWith("<CMD:")) {
        // Process locally via command parser
        for (size_t i = 0; i < cdcCmdBuffer.length(); i++) {
          cmdParser.processChar(cdcCmdBuffer[i]);
        }
        
        // Also forward to GS via LoRa if available
        if (loraManager.isInitialized()) {
          loraManager.sendPacket(LORA_TYPE_CMD, cdcCmdBuffer.c_str(), cdcCmdBuffer.length());
        }
      } else {
        // Other message types are processed locally only
        for (size_t i = 0; i < cdcCmdBuffer.length(); i++) {
          cmdParser.processChar(cdcCmdBuffer[i]);
        }
      }
      
    // Continue collecting command characters
    } else if (cdcCmdInProgress) {
      cdcCmdBuffer += c;
      
      // Prevent buffer overflow
      if (cdcCmdBuffer.length() > LORA_MAX_PACKET_SIZE - 1) {
        cdcCmdInProgress = false;
        cdcCmdBuffer = "";
      }
      
    // Process character directly if not collecting a command
    } else {
      cmdParser.processChar(c);
    }
  }
}

void statusTask() {
  // Update state manager (check for auto transitions)
  stateManager.updateState();
  
  // Periodic status updates at fixed rate
  unsigned long now = millis();
  
  // Send status update if interval has passed
  if (now - lastStatusUpdate >= STATUS_PERIOD_MS) {    lastStatusUpdate = now;
    
    // Send status message using FrameCodec
    char statusBuffer[64];
    FrameCodec::formatStatus(statusBuffer, sizeof(statusBuffer), 
                            stateManager.getStateString(), millis());
    Serial.println(statusBuffer);
    
    // Fire-and-forget over LoRa (no ACK, 0 retries)
    if (loraManager.isInitialized()) {
      loraManager.sendPacket(LORA_TYPE_STATUS, statusBuffer, strlen(statusBuffer));
    }
  }
}

void updateHeartbeatPattern() {
  // Set LED pattern based on current state
  switch (stateManager.getCurrentState()) {
    case STATE_IDLE:
      heartbeat.setPattern(HB_IDLE);
      break;
    case STATE_TEST:
      heartbeat.setPattern(HB_TEST);
      break;
    case STATE_ARMED:
      heartbeat.setPattern(HB_ARMED);
      break;
    case STATE_RECOVERY:
      heartbeat.setPattern(HB_RECOVERY);
      break;
    default:
      heartbeat.setPattern(HB_ERROR);
  }
}

// Function to handle received LoRa packets
void handleLoraPacket(LoraPacket* packet) {
  // Process packet
  if (packet->type == LORA_TYPE_CMD) {
    // Update the last command time
    lastCommandTime = millis();
    
    // Process command received from GS
    char cmdBuffer[LORA_MAX_PACKET_SIZE];
    memcpy(cmdBuffer, packet->data, packet->len);
    cmdBuffer[packet->len] = '\0'; // Ensure null termination
    
    // Debug info
    char buffer[64];
    FrameCodec::formatDebug(buffer, sizeof(buffer), "LORA_CMD");
    Serial.println(buffer);
    
    // Process command through the parser
    for (size_t i = 0; i < packet->len; i++) {
      cmdParser.processChar(cmdBuffer[i]);
    }
    // Complete the command with a '>' if it doesn't have one
    if (packet->len == 0 || cmdBuffer[packet->len-1] != '>') {
      cmdParser.processChar('>');
    }
    
    // Force an extra queue check to immediately process any resulting ACKs
    loraManager.checkQueue();
  }
}

// Function to process binary packets from NAVC
void uartTask() {
  // Check for UART data from NAVC at high frequency
  UartPacketStatus status = uartManager.processUartData();
  
  // If a complete packet is ready, process it
  if (status == PACKET_COMPLETE && uartManager.isPacketReady()) {
    const SensorPacket& packet = uartManager.getLatestPacket();
    
    // Process sensor data based on current state
    switch (stateManager.getCurrentState()) {
      case STATE_ARMED:
        // In ARMED state, format full telemetry and send over LoRa
        if (loraManager.isInitialized()) {
          char telemBuffer[128];
          FrameCodec::formatArmedTelemetry(
            telemBuffer, sizeof(telemBuffer),
            packet.packetId, packet.timestamp,
            packet.altitude, packet.accelX, packet.accelY, packet.accelZ,
            packet.gyroX, packet.gyroY, packet.gyroZ,
            packet.magX, packet.magY, packet.magZ,
            packet.latitude, packet.longitude
          );
          
          // Send telemetry to debug port and LoRa (LoRa at reduced rate if needed)
          Serial.println(telemBuffer);
          loraManager.sendPacket(LORA_TYPE_TELEM, telemBuffer, strlen(telemBuffer));
          
          // Report motion to state manager for auto-recovery detection
          float accelMag = sqrt(
            pow((float)packet.accelX / 1000.0f, 2) +
            pow((float)packet.accelY / 1000.0f, 2) +
            pow((float)packet.accelZ / 1000.0f, 2)
          );
          stateManager.reportMotion(accelMag);
        }
        break;
        
      case STATE_RECOVERY:
        // In RECOVERY state, format recovery telemetry (GPS only) and send over LoRa
        if (loraManager.isInitialized()) {
          char telemBuffer[64];
          FrameCodec::formatRecoveryTelemetry(
            telemBuffer, sizeof(telemBuffer),
            packet.timestamp, packet.latitude, packet.longitude, packet.altitude
          );
          
          // Send recovery telemetry to debug port and LoRa
          Serial.println(telemBuffer);
          loraManager.sendPacket(LORA_TYPE_TELEM, telemBuffer, strlen(telemBuffer));
        }
        break;
        
      case STATE_TEST:
        // In TEST state, report sensor values for testing
        char testBuffer[128];
        snprintf(testBuffer, sizeof(testBuffer), 
                "<TEST:PKT_ID:%u,ALT:%.2fm,ACCEL:%.2f,%.2f,%.2f>",
                packet.packetId,
                (float)packet.altitude / 100.0f,
                (float)packet.accelX / 1000.0f,
                (float)packet.accelY / 1000.0f,
                (float)packet.accelZ / 1000.0f);
        Serial.println(testBuffer);
        break;
        
      default:
        // In other states, just monitor packets without sending telemetry
        break;
    }
    
    // Mark packet as processed
    uartManager.markPacketProcessed();
  } else if (status == PACKET_ERROR) {
    // Log packet error
    char errorBuffer[64];
    FrameCodec::formatDebug(errorBuffer, sizeof(errorBuffer), "PACKET_ERROR");
    Serial.println(errorBuffer);
  }
  
  // Periodically report UART packet statistics
  unsigned long now = millis();
  if (now - lastNavcStatsReport >= 5000) { // Every 5 seconds
    lastNavcStatsReport = now;
    
    // Format and send statistics
    char statsBuffer[128];
    snprintf(statsBuffer, sizeof(statsBuffer),
             "<DEBUG:NAVC_STATS,RECV:%lu,DROPS:%lu,ERRS:%lu,RATE:%.2f,LAST:%lu>",
             uartManager.getPacketsReceived(),
             uartManager.getPacketsDropped(),
             uartManager.getCrcErrors(),
             uartManager.getPacketRate(),
             uartManager.getTimeSinceLastPacket());
    Serial.println(statsBuffer);
  }
}

void setup() {
  // Initialize serial communication for debug
  Serial.begin(921600); // FC uses a higher baud rate for USB-CDC
  while (!Serial && millis() < 3000); // Wait for Serial, but timeout after 3 seconds
    // Initialize UART communication with NAVC
  Serial2.begin(115200);
  Serial2.setRx(PA2);
  Serial2.setTx(PA3);
  
  // Send initialization messages using FrameCodec
  char buffer[64];
  
  FrameCodec::formatDebug(buffer, sizeof(buffer), "FC_INIT: Brunito Flight Controller v0.4");
  Serial.println(buffer);
  FrameCodec::formatDebug(buffer, sizeof(buffer), "COMMANDS: <CMD:COMMAND:params:checksum>");
  Serial.println(buffer);
  
  // Initialize LoRa communication
  FrameCodec::formatDebug(buffer, sizeof(buffer), "INITIALIZING_LORA");
  Serial.println(buffer);
  
  if (loraManager.begin(LORA_FC_ADDR, LORA_GS_ADDR)) {
    FrameCodec::formatDebug(buffer, sizeof(buffer), "LORA_INIT_SUCCESS");
    Serial.println(buffer);
    
    // Set the packet handler
    loraManager.onPacketReceived = handleLoraPacket;
  } else {
    FrameCodec::formatDebug(buffer, sizeof(buffer), "LORA_INIT_FAILED");
    Serial.println(buffer);
  }
  
  FrameCodec::formatDebug(buffer, sizeof(buffer), "NAVC_INTERFACE_READY");
  Serial.println(buffer);
  
  FrameCodec::formatStatus(buffer, sizeof(buffer), "IDLE", millis());
  Serial.println(buffer);
  
  // Initialize heartbeat with IDLE pattern
  updateHeartbeatPattern();
}

void loop() {
  // Run tasks
  cmdTask();
  statusTask();
  
  // Process UART communication with NAVC (high priority, check every loop iteration)
  uartTask();
  
  // Process LoRa communication
  unsigned long now = millis();
  if (now - lastLoraCheck >= 10) { // Check LoRa every 10ms
    lastLoraCheck = now;
    
    // Check for received packets
    loraManager.checkReceived();
    
    // Process queue (send pending packets, retry failed ones)
    loraManager.checkQueue();
    
    // For more reliability, process the queue more frequently for ACKs when commands are being processed
    static unsigned long lastCommandTime = 0;
    const unsigned long commandProcessingWindow = 1000; // 1 second window after receiving a command
    
    if (now - lastCommandTime < commandProcessingWindow) {
      // We are in a post-command processing window, check queue more frequently
      loraManager.checkQueue();
    }
  }
  
  // Update heartbeat pattern if state changed
  updateHeartbeatPattern();
  
  // Update heartbeat LED
  heartbeat.update();
}
