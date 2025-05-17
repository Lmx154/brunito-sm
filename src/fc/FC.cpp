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
#include "../include/fc/TelemManager.h" // Added TelemManager
#include "../include/navc/Sensors.h" // For SensorPacket structure

// Already defined in Heartbeat.h, no need to redefine here
// Using PC13 directly as it's defined by the STM32 framework

// Global objects
HeartbeatManager heartbeat(PC13); // Using PC13 directly (defined by STM32 framework)
StateManager stateManager;
CmdParser cmdParser(stateManager);
LoraManager loraManager;
UartManager uartManager; // For binary packet reception from NAVC
TelemManager telemManager; // For telemetry formatting and scheduling

// Task scheduling variables
unsigned long lastStatusUpdate = 0;
unsigned long lastHeartbeatUpdate = 0;
unsigned long lastLoraCheck = 0;
unsigned long lastCommandTime = 0; // Used for optimizing command ACK processing
unsigned long lastUartCheck = 0;   // For UART packet processing
unsigned long lastNavcStatsReport = 0; // For reporting NAVC packet statistics
unsigned long lastTelemRateAdjust = 0; // For adjusting telemetry rate based on RSSI

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
      }      // Only keep telem_status command for diagnostics
      else if (cdcCmdBuffer.equals("<CMD:TELEM_STATUS>")) {
        char buffer[128];
        TelemState state = telemManager.getTelemState();
        const char* stateStr = "";
        SystemState sysState = stateManager.getCurrentState();
        const char* sysStateStr = stateManager.getStateString();
        
        if (state == TELEM_OFF) stateStr = "OFF";
        else if (state == TELEM_STANDARD) stateStr = "STANDARD";
        else if (state == TELEM_LOW_BW) stateStr = "LOW_BW";
        
        snprintf(buffer, sizeof(buffer), 
                "<CMD_ACK:OK:TELEM_STATUS,SYS_STATE:%s,TELEM_STATE:%s,RATE:%u,RSSI:%d>",
                sysStateStr,
                stateStr,
                telemManager.getTelemetryRate(),
                loraManager.getLastRssi());
        Serial.println(buffer);
        
        // Send response to GS via LoRa
        if (loraManager.isInitialized()) {
          loraManager.sendPacket(LORA_TYPE_CMD, buffer, strlen(buffer));
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
  
  // Periodic status updates at different rates based on state
  unsigned long now = millis();
  unsigned long interval;
  
  // Set status update interval based on state
  switch (stateManager.getCurrentState()) {
    case STATE_IDLE:
      interval = 1000; // 1Hz in IDLE
      break;
    case STATE_TEST:
      interval = 200;  // 5Hz in TEST
      break;
    case STATE_ARMED:
      interval = 50;   // 20Hz in ARMED
      break;
    case STATE_RECOVERY:
      interval = 1000; // 1Hz in RECOVERY
      break;
    default:
      interval = 1000;
  }
    // Send status message if interval has passed
  if (now - lastStatusUpdate >= interval) {
    lastStatusUpdate = now;
    
    // Send status message using FrameCodec
    char statusBuffer[64];
    FrameCodec::formatStatus(statusBuffer, sizeof(statusBuffer), 
                            stateManager.getStateString(), millis());
    Serial.println(statusBuffer);
    
    // Also send status update over LoRa (at a lower rate to avoid congestion)
    static uint8_t loraCounter = 0;
    if (loraManager.isInitialized() && 
        (loraCounter++ % 5 == 0 || // Every 5th update for IDLE/RECOVERY 
         stateManager.getCurrentState() == STATE_ARMED)) { // Every update for ARMED
      
      loraManager.sendPacket(LORA_TYPE_TELEM, statusBuffer, strlen(statusBuffer));
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

void reportNavcStats() {
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

// New function for telemetry task - handles telemetry scheduling and transmission
void telemTask() {
  // Only process telemetry when in ARMED state, as per architecture spec
  SystemState currentState = stateManager.getCurrentState();
  
  // Check if we should process telemetry based on system state
  if (currentState == STATE_ARMED) {
    // If we just entered ARMED state, start telemetry
    static bool telemetryStarted = false;
    if (!telemetryStarted) {
      telemManager.startTelemetry();
      telemetryStarted = true;
      
      char buffer[64];
      FrameCodec::formatDebug(buffer, sizeof(buffer), "TELEMETRY_STARTED_IN_ARMED");
      Serial.println(buffer);
    }
    
    // Adjust telemetry rate based on LoRa RSSI periodically
    unsigned long now = millis();
    if (now - lastTelemRateAdjust >= 2000) { // Check every 2 seconds
      lastTelemRateAdjust = now;
      
      if (loraManager.isInitialized()) {
        telemManager.adjustTelemRate(loraManager.getLastRssi());
        
        // For debugging, report rate changes
        static TelemState lastReportedState = TELEM_OFF;
        TelemState currentState = telemManager.getTelemState();
        
        if (currentState != lastReportedState) {
          char buffer[128];
          if (currentState == TELEM_STANDARD) {
            snprintf(buffer, sizeof(buffer), "<DEBUG:TELEM_RATE:STANDARD,%dHz,RSSI:%d>", 
                    telemManager.getTelemetryRate(), loraManager.getLastRssi());
          } else if (currentState == TELEM_LOW_BW) {
            snprintf(buffer, sizeof(buffer), "<DEBUG:TELEM_RATE:LOW_BW,%dHz,RSSI:%d>", 
                    telemManager.getTelemetryRate(), loraManager.getLastRssi());
          } else {
            snprintf(buffer, sizeof(buffer), "<DEBUG:TELEM_STOPPED>");
          }
          Serial.println(buffer);
          lastReportedState = currentState;
        }
      }
    }
    
    // Check if it's time to send a telemetry packet
    if (telemManager.update() && uartManager.isPacketReady()) {
      // Get the latest sensor packet
      const SensorPacket& packet = uartManager.getLatestPacket();
      
      // Format telemetry as ASCII frame
      char telemBuffer[LORA_MAX_PACKET_SIZE];
      telemManager.formatTelem(telemBuffer, sizeof(telemBuffer), packet);
      
      // Send over LoRa
      if (loraManager.isInitialized()) {
        loraManager.sendPacket(LORA_TYPE_TELEM, telemBuffer, strlen(telemBuffer));
      }
      
      // Mark packet as processed (it will be overwritten by the next one)
      uartManager.markPacketProcessed();
    }
  }   // If we're in RECOVERY state, send simplified recovery telemetry
  else if (currentState == STATE_RECOVERY) {
    // RECOVERY telemetry should have GPS data only at 1Hz
    static bool recoveryTelemetryStarted = false;
    if (!recoveryTelemetryStarted) {
      telemManager.startTelemetry(); // This will initialize the telemetry system
      recoveryTelemetryStarted = true;
      
      char buffer[64];
      FrameCodec::formatDebug(buffer, sizeof(buffer), "RECOVERY_TELEMETRY_STARTED");
      Serial.println(buffer);
    }
    
    // Override telemetry rate to 1Hz for recovery
    static unsigned long lastRecoveryTelem = 0;
    unsigned long now = millis();
    
    if (now - lastRecoveryTelem >= 1000) { // 1Hz rate for recovery
      lastRecoveryTelem = now;
      
      if (uartManager.isPacketReady()) {
        // Get the latest sensor packet
        const SensorPacket& packet = uartManager.getLatestPacket();
        
        // Format recovery telemetry (GPS only)
        char telemBuffer[LORA_MAX_PACKET_SIZE];
        FrameCodec::formatRecoveryTelemetry(
          telemBuffer, sizeof(telemBuffer),
          packet.timestamp, packet.latitude, packet.longitude, packet.altitude
        );
        
        // Send over LoRa
        if (loraManager.isInitialized()) {
          loraManager.sendPacket(LORA_TYPE_TELEM, telemBuffer, strlen(telemBuffer));
        }
        
        // Mark packet as processed (it will be overwritten by the next one)
        uartManager.markPacketProcessed();
      }
    }
  }
  // If state changed to something other than ARMED, stop telemetry
  else {
    static bool telemetryStopped = false;
    if (!telemetryStopped && telemManager.isTelemetryActive()) {
      telemManager.stopTelemetry();
      telemetryStopped = true;
      
      char buffer[64];
      FrameCodec::formatDebug(buffer, sizeof(buffer), "TELEMETRY_STOPPED_NOT_ARMED");
      Serial.println(buffer);
    }
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
  
  // Process telemetry only when in appropriate states (ARMED/RECOVERY)
  // The telemTask function internally checks the state
  telemTask();
  
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
