/**
 * GS.cpp - Ground Station for Brunito Project
 * 
 * This file contains the Ground Station (GS) module implementation.
 * Phase 3: Implements LoRa communication with the Flight Controller.
 */

#include <Arduino.h>
#include "../include/utils/Heartbeat.h"
#include "../include/utils/FrameCodec.h"
#include "../include/utils/LoraManager.h"
#include "../include/config/lora.h"

// Global objects
HeartbeatManager heartbeat(PC13);
LoraManager loraManager;

// Variables for command handling
char cmdBuffer[LORA_MAX_PACKET_SIZE];
int cmdIndex = 0;
bool cmdReady = false;

// Variables for task scheduling
unsigned long lastHeartbeatUpdate = 0;
unsigned long lastLoraCheck = 0;

// Function to handle received LoRa packets
void handleLoraPacket(LoraPacket* packet) {
  // Process packet
  if (packet->type == LORA_TYPE_CMD) {
    // Command or ACK from FC - forward to Serial
    char msgBuffer[LORA_MAX_PACKET_SIZE + 1]; // +1 for null terminator
    memcpy(msgBuffer, packet->data, packet->len);
    msgBuffer[packet->len] = '\0'; // Ensure null termination
    
    Serial.println(msgBuffer);
    
    // Show RSSI info
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "<RSSI:%d,SNR:%.2f>", 
             loraManager.getLastRssi(), loraManager.getLastSnr());
    Serial.println(buffer);
  }
  else if (packet->type == LORA_TYPE_TELEM) {
    // Telemetry from FC - forward to Serial
    char msgBuffer[LORA_MAX_PACKET_SIZE + 1]; // +1 for null terminator
    memcpy(msgBuffer, packet->data, packet->len);
    msgBuffer[packet->len] = '\0'; // Ensure null termination
    
    Serial.println(msgBuffer);
  }
  else if (packet->type == LORA_TYPE_STATUS) {
    // Status messages from FC - forward to Serial
    char msgBuffer[LORA_MAX_PACKET_SIZE + 1]; // +1 for null terminator
    memcpy(msgBuffer, packet->data, packet->len);
    msgBuffer[packet->len] = '\0'; // Ensure null termination
    
    Serial.println(msgBuffer);
  }
}

// Process serial input
void processSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Check for command termination (Enter key)
    if (c == '\r' || c == '\n') {
      if (cmdIndex > 0) {
        // Null-terminate the command string
        cmdBuffer[cmdIndex] = '\0';
        cmdReady = true;
        break;
      }
    }
    // Backspace handling
    else if (c == 8 || c == 127) {
      if (cmdIndex > 0) {
        cmdIndex--;
        Serial.print("\b \b"); // Erase character on console
      }
    }
    // Add character to buffer if not full
    else if (cmdIndex < LORA_MAX_PACKET_SIZE - 1) {
      cmdBuffer[cmdIndex++] = c;
      Serial.print(c); // Echo character
    }
  }
  
  // Process command if ready
  if (cmdReady) {
    Serial.println(); // New line after command
    
    // Look for command framing - if not properly framed, add it
    if (strncmp(cmdBuffer, "<CMD:", 5) != 0 || cmdBuffer[cmdIndex - 1] != '>') {
      // Not a properly framed command, frame it
      char framedCmd[LORA_MAX_PACKET_SIZE];
      snprintf(framedCmd, sizeof(framedCmd), "<CMD:%s>", cmdBuffer);
      
      // Send over LoRa
      if (loraManager.sendCommand(framedCmd)) {
        // Echo properly framed command
        Serial.print("Sent via LoRa: ");
        Serial.println(framedCmd);
        Serial.println("Waiting for acknowledgment...");
      } else {
        Serial.println("FAILED to queue command - queue full or not initialized");
      }
    } else {
      // Already properly framed, send as is
      if (loraManager.sendCommand(cmdBuffer)) {
        // Echo command
        Serial.print("Sent via LoRa: ");
        Serial.println(cmdBuffer);
        Serial.println("Waiting for acknowledgment...");
      } else {
        Serial.println("FAILED to queue command - queue full or not initialized");
      }
    }
    
    // Reset command buffer
    cmdIndex = 0;
    cmdReady = false;
  }
}

// Print help information
void printHelp() {
  Serial.println("\n--- Brunito Ground Station v0.4 ---");
  Serial.println("Available commands:");
  Serial.println("  <CMD:ARM>                - Arm the flight controller");
  Serial.println("  <CMD:DISARM>             - Disarm the flight controller");
  Serial.println("  <CMD:ENTER_TEST>         - Enter test mode");
  Serial.println("  <CMD:ENTER_RECOVERY>     - Enter recovery mode");
  Serial.println("  <CMD:QUERY>              - Query current state");
  Serial.println("  <CMD:FIND_ME>            - Activate buzzer for locating");
  Serial.println("  <CMD:CONTROL:param=val>  - Control actuators (e.g., servo=90)");
  Serial.println("  <CMD:NAVC_RESET_STATS>   - Reset NAVC packet statistics");
  Serial.println("  <CMD:LORA_RESET_STATS>   - Reset LoRa statistics counters");
  Serial.println("  <CMD:LORA_STATS>         - Show detailed LoRa statistics");
  Serial.println("Type command and press Enter to send");
  Serial.println("Commands are automatically framed if needed\n");
}

void setup() {
  // Initialize serial communication for console
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial, but timeout after 3 seconds
  
  // Initialize LED
  heartbeat.setPattern(HB_IDLE);
  
  Serial.println("\n<DEBUG:GS_INIT>");
  printHelp();
  
  // Initialize LoRa communication  Serial.println("<DEBUG:INITIALIZING_LORA>");
  if (loraManager.begin(LORA_GS_ADDR, LORA_FC_ADDR)) {
    Serial.println("<DEBUG:LORA_INIT_SUCCESS>");
    Serial.println("<GS_LINK:READY>");
    
    // Set the packet handler
    loraManager.onPacketReceived = handleLoraPacket;
  } else {
    Serial.println("<DEBUG:LORA_INIT_FAILED>");
    Serial.println("<GS_LINK:ERROR>");
    
    // Set error pattern on LED
    heartbeat.setPattern(HB_ERROR);
  }
}

void loop() {
  // Process any incoming serial data
  processSerialInput();
  
  // Process LoRa communication
  unsigned long now = millis();
  if (now - lastLoraCheck >= 10) { // Check LoRa every 10ms
    lastLoraCheck = now;
    
    // Check for received packets first - this gives us a chance to process ACKs
    loraManager.checkReceived();
    
    // Give a small delay to allow for ACK processing
    delay(5);
    
    // Process queue (send pending packets, retry failed ones)
    loraManager.checkQueue();
    
    // Periodic status updates less frequently to avoid duplicates (every 30 seconds)
    static unsigned long lastStatusUpdate = 0;
    if (now - lastStatusUpdate >= 30000) {
      lastStatusUpdate = now;
      
      // Show link status
      if (loraManager.isInitialized()) {
        // First check if we're connected to the FC
        if (!loraManager.isConnected()) {
          // Haven't received anything from FC within timeout period
          Serial.println("<GS_LINK:DOWN,NO_CONNECTION>");
          heartbeat.setPattern(HB_ERROR); // Error pattern for no connection
        } else {
          // We have a connection, evaluate its quality
          int16_t rssi = loraManager.getLastRssi();
          float snr = loraManager.getLastSnr();
          
          // Get packet statistics
          uint16_t sent = loraManager.getPacketsSent();
          uint16_t received = loraManager.getPacketsReceived();
          uint16_t lost = loraManager.getPacketsLost();
          float lossRate = loraManager.getPacketLossRate() * 100.0f; // Convert to percentage
          
          // Determine link quality
          String linkQuality;
          if (rssi > -90) {
            linkQuality = "EXCELLENT";
            heartbeat.setPattern(HB_IDLE); // Slow blink for good connection
          } else if (rssi > -100) {
            linkQuality = "GOOD";
            heartbeat.setPattern(HB_TEST); // Medium blink for OK connection
          } else if (rssi > -110) {
            linkQuality = "FAIR";
            heartbeat.setPattern(HB_ARMED); // Fast blink for poor connection
          } else {
            linkQuality = "POOR";
            heartbeat.setPattern(HB_RECOVERY); // Very fast blink for bad connection
          }
          
          char buffer[128];
          snprintf(buffer, sizeof(buffer), "<GS_LINK:%s,RSSI:%d,SNR:%.1f,PKT_SENT:%u,PKT_RECV:%u,LOSS:%.1f%%>", 
                  linkQuality.c_str(), rssi, snr, sent, received, lossRate);
          Serial.println(buffer);
          
          // Reset statistics every hour to avoid overflow
          if (loraManager.getStatsDuration() > 3600000) {
            loraManager.resetStats();
          }
        }
      } else {
        Serial.println("<GS_LINK:DOWN>");
        heartbeat.setPattern(HB_ERROR); // Error pattern for no connection
      }
    }
  }
  
  // Update heartbeat LED
  heartbeat.update();
}
