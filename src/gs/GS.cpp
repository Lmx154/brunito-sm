/**
 * GS.cpp - Ground Station for Brunito Project
 * 
 * This file contains the Ground Station (GS) module implementation.
 * Phase 4: Implements LoRa communication with FC and telemetry parsing/CSV output.
 */

#include <Arduino.h>
#include "../include/utils/Heartbeat.h"
#include "../include/utils/FrameCodec.h"
#include "../include/utils/LoraManager.h"
#include "../include/config/lora.h"

// Forward declarations
void printHelp();

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
unsigned long lastBandwidthCheck = 0;
const unsigned long BANDWIDTH_CHECK_INTERVAL = 60000; // Check bandwidth every 60 seconds

// CSV output control
bool outputCsvFormat = true;
bool csvHeaderPrinted = false;

/**
 * Parse telemetry frame and output as CSV
 * 
 * @param frame The telemetry frame string
 */
void parseTelemToCsv(const char* frame) {
  // Verify it's a TELEM frame
  if (strncmp(frame, "<TELEM:", 7) != 0) {
    Serial.println("<DEBUG:NOT_TELEM_FRAME>");
    return;
  }
  
  // Extract payload between : and >
  char payload[LORA_MAX_PACKET_SIZE];
  size_t i = 7;  // Skip "<TELEM:"
  size_t p = 0;
  
  while (frame[i] != '>' && frame[i] != '\0' && p < sizeof(payload) - 1) {
    payload[p++] = frame[i++];
  }
  payload[p] = '\0';  // Null terminate
  // Count commas to determine format (ARMED or RECOVERY)
  int commaCount = 0;
  for (size_t j = 0; j < p; j++) {
    if (payload[j] == ',') commaCount++;
  }
  // Print CSV header if needed
  if (!csvHeaderPrinted) {
    if (commaCount == 14) {  // ARMED format (15 values - now includes sats)
      Serial.println("pkID,timestamp_ms,alt_m,accel_x_g,accel_y_g,accel_z_g,"
                    "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
                    "mag_x_uT,mag_y_uT,mag_z_uT,lat_deg,lon_deg,sats");
    } else if (commaCount == 4) {  // RECOVERY format (5 values - now includes sats)
      Serial.println("timestamp_ms,lat_deg,lon_deg,alt_m,sats");
    }
    csvHeaderPrinted = true;
  }
  
  // Parse values and apply scaling based on format
  if (commaCount == 14) {  // ARMED format
    // Parse all 15 values
    long values[15];
    int valueIndex = 0;
    char* token = strtok(payload, ",");
    
    while (token != NULL && valueIndex < 15) {
      values[valueIndex++] = atol(token);
      token = strtok(NULL, ",");
    }
    
    // Apply scaling and print CSV line
    if (valueIndex == 15) {
      // Values: pkID, timestamp, alt, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, lat, lon, sats
      Serial.print(values[0]); Serial.print(",");                   // pkID (raw)
      Serial.print(values[1]); Serial.print(",");                   // timestamp_ms (raw)
      Serial.print(values[2] / 100.0f, 2); Serial.print(",");       // alt_m (cm → m)
      Serial.print(values[3] / 1000.0f, 3); Serial.print(",");      // accel_x_g (mg → g)
      Serial.print(values[4] / 1000.0f, 3); Serial.print(",");      // accel_y_g (mg → g)
      Serial.print(values[5] / 1000.0f, 3); Serial.print(",");      // accel_z_g (mg → g)
      Serial.print(values[6] / 100.0f, 2); Serial.print(",");       // gyro_x_dps (0.01 dps → dps)
      Serial.print(values[7] / 100.0f, 2); Serial.print(",");       // gyro_y_dps (0.01 dps → dps)
      Serial.print(values[8] / 100.0f, 2); Serial.print(",");       // gyro_z_dps (0.01 dps → dps)
      Serial.print(values[9] / 10.0f, 1); Serial.print(",");        // mag_x_uT (0.1 μT → μT)
      Serial.print(values[10] / 10.0f, 1); Serial.print(",");       // mag_y_uT (0.1 μT → μT)
      Serial.print(values[11] / 10.0f, 1); Serial.print(",");       // mag_z_uT (0.1 μT → μT)
      Serial.print(values[12] / 10000000.0f, 7); Serial.print(","); // lat_deg (1e7 → degrees)
      Serial.print(values[13] / 10000000.0f, 7); Serial.print(","); // lon_deg (1e7 → degrees)
      Serial.print(values[14]);                                     // sats (raw)
      Serial.println();
    }
  } 
  else if (commaCount == 4) {  // RECOVERY format
    // Parse all 5 values
    long values[5];
    int valueIndex = 0;
    char* token = strtok(payload, ",");
    
    while (token != NULL && valueIndex < 5) {
      values[valueIndex++] = atol(token);
      token = strtok(NULL, ",");
    }
    
    // Apply scaling and print CSV line
    if (valueIndex == 5) {
      // Values: timestamp, lat, lon, alt, sats
      Serial.print(values[0]); Serial.print(",");                  // timestamp_ms (raw)
      Serial.print(values[1] / 10000000.0f, 7); Serial.print(","); // lat_deg (1e7 → degrees)
      Serial.print(values[2] / 10000000.0f, 7); Serial.print(","); // lon_deg (1e7 → degrees)
      Serial.print(values[3] / 100.0f, 2); Serial.print(",");      // alt_m (cm → m)
      Serial.print(values[4]);                                     // sats (raw)
      Serial.println();
    }
  }
}

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
    float snr = loraManager.getLastSnr();
    int rssi = loraManager.getLastRssi();
    
    // Check if SNR is valid
    if (snr != 0) {
      snprintf(buffer, sizeof(buffer), "<RSSI:%d,SNR:%.2f>", rssi, snr);
    } else {
      snprintf(buffer, sizeof(buffer), "<RSSI:%d,SNR:>", rssi);
    }
    Serial.println(buffer);  }  else if (packet->type == LORA_TYPE_TELEM) {
    // Telemetry from FC - process and output as CSV
    char msgBuffer[LORA_MAX_PACKET_SIZE + 1]; // +1 for null terminator
    memcpy(msgBuffer, packet->data, packet->len);
    msgBuffer[packet->len] = '\0'; // Ensure null termination
      // Debug telemetry reception with RSSI information
    char debugBuffer[96];
    int16_t rssi = loraManager.getLastRssi();
    float snr = loraManager.getLastSnr();
    
    // Add rate throttling indicator to help user understand if packets are being throttled
    const char* rateIndicator = "NORMAL";
    if (rssi < -110) {
        rateIndicator = "THROTTLED_4HZ";
    } else if (rssi < -100) {
        rateIndicator = "THROTTLED_10HZ";
    }
    
    snprintf(debugBuffer, sizeof(debugBuffer), 
             "<DEBUG:TELEM_RECEIVED:RSSI=%d,SNR=%.1f,LEN=%u,RATE=%s>", 
             rssi, snr, packet->len, rateIndicator);
    Serial.println(debugBuffer);
    
    // Always show the raw frame for debug purposes
    Serial.print("<RAW_TELEM:");
    Serial.print(msgBuffer);
    Serial.println(">");
    
    // Parse telemetry data and output as CSV if enabled
    if (outputCsvFormat) {
      parseTelemToCsv(msgBuffer);
    }
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
    
    // Check for local commands that don't get sent to FC
    if (strcmp(cmdBuffer, "CSV_ON") == 0) {
      outputCsvFormat = true;
      csvHeaderPrinted = false; // Reset to print header on next telemetry
      Serial.println("<GS:CSV_FORMAT_ENABLED>");
    }
    else if (strcmp(cmdBuffer, "CSV_OFF") == 0) {
      outputCsvFormat = false;
      Serial.println("<GS:CSV_FORMAT_DISABLED>");
    }
    else if (strcmp(cmdBuffer, "HELP") == 0) {
      printHelp();
    }
    // Look for command framing - if not properly framed, add it
    else if (strncmp(cmdBuffer, "<CMD:", 5) != 0 || cmdBuffer[cmdIndex - 1] != '>') {
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
  Serial.println("Local commands:");
  Serial.println("  CSV_ON                   - Enable CSV output format for telemetry");
  Serial.println("  CSV_OFF                  - Display raw telemetry frames");
  Serial.println("  HELP                     - Show this help message");
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
    
    // Enable ping mechanism for better link quality assessment
    loraManager.enablePing(true);
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
    
    // Send ping to check link quality
    loraManager.sendPing();
    
    // Periodically monitor bandwidth usage
    if (now - lastBandwidthCheck >= BANDWIDTH_CHECK_INTERVAL) {
      lastBandwidthCheck = now;
      loraManager.reportBandwidthUsage();
      
      // Calculate approximate telemetry rate based on packet count
      static unsigned long lastPacketCount = 0;
      static unsigned long lastRateCheckTime = 0;
      
      uint16_t received = loraManager.getPacketsReceived();
      unsigned long packetsSinceLastCheck = 0;
      if (received > lastPacketCount) {
        packetsSinceLastCheck = received - lastPacketCount;
      }
      
      float interval = (now - lastRateCheckTime) / 1000.0f;
      if (interval > 0.1f) { // Avoid division by near-zero
        float packetsPerSecond = packetsSinceLastCheck / interval;
        
        // Only report if we got some packets
        if (packetsSinceLastCheck > 0) {
          char rateBuffer[80];
          snprintf(rateBuffer, sizeof(rateBuffer), 
                  "<DEBUG:TELEM_RATE:%.1f_Hz,RSSI=%d,THROTTLING=%s>", 
                  packetsPerSecond, 
                  loraManager.getLastRssi(),
                  (loraManager.getLastRssi() < -110 ? "YES" : 
                   (loraManager.getLastRssi() < -100 ? "PARTIAL" : "NO")));
          Serial.println(rateBuffer);
        }
      }
      
      // Update counters for next time
      lastPacketCount = received;
      lastRateCheckTime = now;
    }
    
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
            // Format the loss rate string properly or use "0.0" to avoid a bare %
          char lossRateStr[16];
          if (sent > 0) {
            snprintf(lossRateStr, sizeof(lossRateStr), "%.1f", lossRate);
          } else {
            strcpy(lossRateStr, "0.0");
          }
          
          // Include last received time in HH:MM:SS format
          char timeStr[16];
          uint32_t timeSinceLastRx = loraManager.getTimeSinceLastRx();
          uint32_t seconds = timeSinceLastRx / 1000;
          uint8_t hours = seconds / 3600;
          seconds %= 3600;
          uint8_t minutes = seconds / 60;
          seconds %= 60;
          snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hours, minutes, (uint8_t)seconds);
            char buffer[128];
          // Check if SNR value is valid (RFM95 returns 0 SNR when no packets received)
          if (received > 0 && snr != 0) {
            snprintf(buffer, sizeof(buffer), "<GS_LINK:%s,RSSI:%d,SNR:%.1f,PKT_SENT:%u,PKT_RECV:%u,LOSS:%s%%,LAST_RX:%s>", 
                    linkQuality.c_str(), rssi, snr, sent, received, lossRateStr, timeStr);
          } else {
            // No valid SNR, omit it from output
            snprintf(buffer, sizeof(buffer), "<GS_LINK:%s,RSSI:%d,SNR:,PKT_SENT:%u,PKT_RECV:%u,LOSS:%s%%,LAST_RX:%s>", 
                    linkQuality.c_str(), rssi, sent, received, lossRateStr, timeStr);
          }
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
