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
#include "../include/gs/TelemParser.h" // Added TelemParser

// Global objects
HeartbeatManager heartbeat(PC13);
LoraManager loraManager;
TelemParser telemParser;

// Variables for command handling
char cmdBuffer[LORA_MAX_PACKET_SIZE];
int cmdIndex = 0;
bool cmdReady = false;

// Variables for task scheduling
unsigned long lastHeartbeatUpdate = 0;
unsigned long lastLoraCheck = 0;
unsigned long lastLinkStatusUpdate = 0;
bool linkStatusChanged = false;
String currentLinkStatus = "UNKNOWN";

// Function to handle received LoRa packets
void handleLoraPacket(LoraPacket* packet) {
  // Update link status whenever we receive any packet
  linkStatusChanged = true;
  
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
  }  else if (packet->type == LORA_TYPE_TELEM) {
    // Telemetry from FC - process with TelemParser
    char msgBuffer[LORA_MAX_PACKET_SIZE + 1]; // +1 for null terminator
    memcpy(msgBuffer, packet->data, packet->len);
    msgBuffer[packet->len] = '\0'; // Ensure null termination
    
    // Track the last packet ID we received to avoid processing duplicates
    static uint16_t lastStatusPacketId = 0;
    
    // Check if this is a status update packet (now used for heartbeat)
    if (strncmp(msgBuffer, "<STATUS:", 8) == 0) {
      // This is a status message that also serves as heartbeat
      // The linkStatusChanged flag has already been set to update connection status
      
      // Only process each status packet once by tracking the packet ID
      if (packet->id != lastStatusPacketId) {
        lastStatusPacketId = packet->id;
        
        // Print status messages but limit to once every 10 seconds to avoid log spam
        static unsigned long lastStatusPrinted = 0;
        unsigned long now = millis();
        
        if (now - lastStatusPrinted >= 10000) {
          lastStatusPrinted = now;
          
          // Only occasionally show the status message
          Serial.print("# STATUS: ");
          Serial.println(msgBuffer);
        }
      }
    } else if (strncmp(msgBuffer, "<HEARTBEAT:", 11) == 0) {
      // Legacy heartbeat packet format (if any), don't print these
      // The link status is already updated via linkStatusChanged flag
    } else {
      // Regular telemetry packet, process normally
      // Log raw telemetry frame (prefixed to distinguish from CSV output)
      Serial.print("# RAW: ");
      Serial.println(msgBuffer);
      
      // Process and output as CSV
      telemParser.processTelemetryFrame(msgBuffer);
        
      // Add RSSI information as comment for plotting tools
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "# RSSI:%d,SNR:%.2f",
               loraManager.getLastRssi(), loraManager.getLastSnr());
      Serial.println(buffer);
    }
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
  
  // Initialize LoRa communication
  Serial.println("<DEBUG:INITIALIZING_LORA>");
  
  if (loraManager.begin(LORA_GS_ADDR, LORA_FC_ADDR)) {
    Serial.println("<DEBUG:LORA_INIT_SUCCESS>");
    Serial.println("<STATUS:READY>");
    
    // Set the packet handler
    loraManager.onPacketReceived = handleLoraPacket;
    
    // Increase connection timeout to 30 seconds to prevent false disconnection reports
    loraManager.setConnectionTimeout(30000);
  } else {
    Serial.println("<DEBUG:LORA_INIT_FAILED>");
    Serial.println("<STATUS:ERROR>");
    
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
    
    // Check for received packets
    loraManager.checkReceived();
    
    // Process queue (send pending packets, retry failed ones)
    loraManager.checkQueue();
      // ===== LINK STATUS MANAGEMENT - SEPARATE FROM FC STATE STATUS =====
    // Update link status in two cases:
    // 1. When a packet is received (linkStatusChanged flag)
    // 2. Periodically (every 10 seconds) to detect disconnections
    
    // Only attempt to evaluate link status if LoRa is initialized
    if (loraManager.isInitialized()) {
      // Update link status when:
      // - We just received a packet (flagged by handleLoraPacket)
      // - OR every 30 seconds as a periodic check (matching the FC timeout setting)
      if (linkStatusChanged || (now - lastLinkStatusUpdate >= 30000)) {
        lastLinkStatusUpdate = now;
        linkStatusChanged = false;
        
        // Determine the current link status
        String newLinkStatus;
        bool isConnected = loraManager.isConnected();
        
        if (!isConnected) {
          // No packets received within timeout period
          newLinkStatus = "LINK_DOWN";
          heartbeat.setPattern(HB_ERROR); // Error pattern for no connection
        } else {
          // Evaluate link quality based on RSSI
          int16_t rssi = loraManager.getLastRssi();
          
          if (rssi > -90) {
            newLinkStatus = "LINK_EXCELLENT";
            heartbeat.setPattern(HB_IDLE); // Slow blink for good connection
          } else if (rssi > -100) {
            newLinkStatus = "LINK_GOOD"; 
            heartbeat.setPattern(HB_TEST); // Medium blink for OK connection
          } else if (rssi > -110) {
            newLinkStatus = "LINK_FAIR";
            heartbeat.setPattern(HB_ARMED); // Fast blink for poor connection
          } else {
            newLinkStatus = "LINK_POOR";
            heartbeat.setPattern(HB_RECOVERY); // Very fast blink for bad connection
          }
        }
        
        // Only print link status if it has changed
        // (we no longer periodically print status - this is handled by the FC's status heartbeats)
        if (newLinkStatus != currentLinkStatus) {
          currentLinkStatus = newLinkStatus;
          
          // Format and print appropriate status message - prefixed with # for consistency
          if (currentLinkStatus == "LINK_DOWN") {
            Serial.println("# LINK STATUS: LINK_DOWN,NO_CONNECTION");
          } else {
            // We have a connection, get statistics
            float snr = loraManager.getLastSnr();
            uint16_t sent = loraManager.getPacketsSent();
            uint16_t received = loraManager.getPacketsReceived();
            uint16_t lost = loraManager.getPacketsLost();
            float lossRate = loraManager.getPacketLossRate() * 100.0f; // Convert to percentage
            
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "# LINK STATUS: %s,RSSI:%d,SNR:%.1f,PKT_SENT:%u,PKT_RECV:%u,LOSS:%.1f%%", 
                    currentLinkStatus.c_str(), loraManager.getLastRssi(), snr, sent, received, lossRate);
            Serial.println(buffer);
          }
        }
      }
      
      // Reset statistics every hour to avoid overflow
      if (loraManager.getStatsDuration() > 3600000) {
        loraManager.resetStats();
      }
    } else {
      // Only report LoRa not initialized status when the status changes
      if (currentLinkStatus != "LINK_NOT_INITIALIZED") {
        currentLinkStatus = "LINK_NOT_INITIALIZED";
        Serial.println("<STATUS:LINK_DOWN,NOT_INITIALIZED>");
        heartbeat.setPattern(HB_ERROR); // Error pattern for no connection
      }
    }
  }
  
  // Update heartbeat LED
  heartbeat.update();
}
