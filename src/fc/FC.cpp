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
unsigned long lastLoraLinkReport = 0; // For reporting LoRa link quality
// Removed lastUartLossReport variable as packet loss reporting has been removed
static const uint32_t LORA_LINK_REPORT_PERIOD_MS = 30'000;   // 30 seconds, same as GS
static const uint32_t UART_LOSS_REPORT_PERIOD_MS = 10'000;   // 10 seconds

// Telemetry control variables
uint8_t telemRate = 10; // Default fixed to 10 Hz for optimal command reception
unsigned long lastTelemTime = 0;
unsigned long telemPeriodMs = 100; // 100ms = 10Hz

// Velocity packet timing variables (10Hz when velocity test is active)
unsigned long lastVelocityPacketTime = 0;
static const uint32_t VELOCITY_PACKET_PERIOD_MS = 100; // 100ms = 10Hz

// Forward declarations
void reportLoraLinkStatus();
String formatTelem(const SensorPacket& packet, bool gpsOnly);
String formatVelocityPacket(const SensorPacket& packet, int32_t velocity, int32_t threshold);
void startTelemetry(uint8_t hz = 10);
void adjustTelemRate();
void reinitializeUart();

// Global variables in FC.cpp at the top with other globals
static const uint32_t TELEM_RATE_CHECK_INTERVAL = 5000; // Check rate every 5 seconds
unsigned long lastRateAdjustTime = 0;  // Last time we adjusted the telemetry rate

// Add to the global variables section at the top of the file
static const uint32_t BANDWIDTH_REPORT_INTERVAL_MS = 60'000; // Report bandwidth usage every 60 seconds
unsigned long lastBandwidthReport = 0; // For bandwidth usage reporting

/**
 * Formats telemetry data from binary packets to ASCII format
 * 
 * @param packet The binary sensor packet
 * @param gpsOnly Whether to only include GPS data (for RECOVERY state)
 * @return Formatted ASCII telemetry string
 */
String formatTelem(const SensorPacket& packet, bool gpsOnly) {
  char buffer[250];
    // Format altitude as meters with 2 decimal places using integer formatting
  float altitudeMeters = packet.altitude / 100.0f;
  char altStr[10]; // Buffer for formatted altitude
  
  // Use integer-based formatting to handle negative values correctly
  int altWhole = (int)altitudeMeters;
  int altFrac = (int)(fabs(altitudeMeters - altWhole) * 100);
  snprintf(altStr, sizeof(altStr), "%s%d.%02d", 
           (altitudeMeters < 0) ? "-" : "", abs(altWhole), altFrac);
  
  // Debug output to see what altitude values we're getting
  static unsigned long lastAltitudeDebug = 0;
  if (millis() - lastAltitudeDebug > 5000) { // Every 5 seconds
    char debugBuf[64];
    snprintf(debugBuf, sizeof(debugBuf), "<DEBUG:RAW_ALTITUDE=%ld,CONVERTED=%s>", 
             packet.altitude, altStr);
    Serial.println(debugBuf);
    lastAltitudeDebug = millis();
  }
  
  // Format date and time as MM/DD/YYYY,HH:MM:SS
  char datetime[22]; // Enough space for MM/DD/YYYY,HH:MM:SS
  snprintf(datetime, sizeof(datetime), "%02u/%02u/20%02u,%02u:%02u:%02u", 
           packet.month, packet.day, packet.year, 
           packet.hour, packet.minute, packet.second);
           
  if (gpsOnly) {
    // RECOVERY mode - GPS only telemetry with formatted date/time, satellite count, and temperature
    snprintf(buffer, sizeof(buffer), "<%s,%ld,%ld,%s,%u,%d>", // No T: prefix as requested by user
             datetime,
             packet.latitude, 
             packet.longitude, 
             altStr, // Using formatted altitude with decimal places
             packet.satellites,
             packet.temperature);
  } else {
    // ARMED mode - More compact format with no tag prefix as requested by user
    // Removed packetId and T: prefix as requested by user 
    snprintf(buffer, sizeof(buffer), 
             "<%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%u,%d>",
             datetime,
             altStr, // Using formatted altitude with decimal places
             packet.accelX, packet.accelY, packet.accelZ,
             packet.gyroX, packet.gyroY, packet.gyroZ,
             packet.magX, packet.magY, packet.magZ,
             packet.latitude, packet.longitude,
             packet.satellites,
             packet.temperature);
  }
  
  return String(buffer);
}

/**
 * Starts telemetry streaming at specified rate,
 * adjusting based on bandwidth limitations
 * 
 * @param hz Desired telemetry rate in Hz
 */
void startTelemetry(uint8_t hz) {
  // Set the maximum desired rate
  telemRate = hz;
    // Cap rate at 10 Hz maximum to ensure reliable command handling
  if (telemRate > 10) {
    telemRate = 10; // Maximum 10 Hz for optimal command reception
  }
  
  // Force immediate rate adjustment check
  lastRateAdjustTime = 0;
  
  // Adjust telemRate based on link quality
  adjustTelemRate();
  
  // Calculate period in milliseconds
  telemPeriodMs = 1000 / telemRate;
  
  // Reset last telemetry time to send first packet immediately
  lastTelemTime = 0;
  
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "<DEBUG:TELEM_RATE_SET:%u>", telemRate);
  Serial.println(buffer);
}

/**
 * Adjusts telemetry rate based on LoRa link quality
 * Throttles from 20 → 1 Hz depending on link conditions
 */
void adjustTelemRate() {
  if (!loraManager.isInitialized()) {
    return;
  }
  
  unsigned long now = millis();
  
  // Check more frequently (every 2 seconds) to respond faster to changing conditions
  static const uint32_t QUICK_RATE_CHECK_INTERVAL = 2000;
  if (now - lastRateAdjustTime < QUICK_RATE_CHECK_INTERVAL) {
    return;
  }
  
  lastRateAdjustTime = now;
  
  int16_t rssi = loraManager.getLastRssi();
  uint8_t newRate = telemRate;
  
  // Check for failed transmissions to adapt rate
  float lossRate = loraManager.getPacketLossRate();
  uint8_t pendingCount = loraManager.getPendingPacketCount();
    // Very aggressive rate adjustment based on multiple factors
  if (rssi < -110 || lossRate > 0.3 || pendingCount > 3) {
    // Very poor link quality - reduce to 1 Hz
    newRate = 1;
  } else if (rssi < -100 || lossRate > 0.2 || pendingCount > 2) {
    // Poor link quality - reduce to 2 Hz
    newRate = 2;
  } else if (rssi < -90 || lossRate > 0.1 || pendingCount > 1) {
    // Fair link quality - reduce to 5 Hz
    newRate = 5;  } else if (rssi < -80) {
    // Good link quality - use 8 Hz
    newRate = 8;
  } else {
    // Excellent link quality - use max 10 Hz to ensure command reception
    newRate = 10;
  }
  
  // Only update if rate changed
  if (newRate != telemRate) {
    telemRate = newRate;
    telemPeriodMs = 1000 / telemRate;
    
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "<DEBUG:TELEM_RATE_ADJUSTED:%u,RSSI:%d>", telemRate, rssi);
    Serial.println(buffer);
    
    // Also report bandwidth usage when rate changes
    loraManager.reportBandwidthUsage();
  }
}

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
      }      // Check for LoRa stats command
      else if (cdcCmdBuffer.equals("<CMD:LORA_STATS>")) {
        if (loraManager.isInitialized()) {
          // Simply call our reportLoraLinkStatus function for consistent formatting
          reportLoraLinkStatus();
          
          // Also create a separate message just for the GS
          char buffer[128];
          float snr = loraManager.getLastSnr();
          int rssi = loraManager.getLastRssi();
          
          if (snr != 0) {
            snprintf(buffer, sizeof(buffer), 
                    "<CMD_ACK:OK:LORA_STATS,RSSI:%d,SNR:%.1f,TX:%u,RX:%u,LOST:%u,LOSS:%.1f%%%%>",
                    rssi, snr,
                    loraManager.getPacketsSent(),
                    loraManager.getPacketsReceived(),
                    loraManager.getPacketsLost(),
                    loraManager.getPacketLossRate() * 100.0f);
          } else {
            snprintf(buffer, sizeof(buffer), 
                    "<CMD_ACK:OK:LORA_STATS,RSSI:%d,SNR:,TX:%u,RX:%u,LOST:%u,LOSS:%.1f%%%%>",
                    rssi,
                    loraManager.getPacketsSent(),
                    loraManager.getPacketsReceived(),
                    loraManager.getPacketsLost(),
                    loraManager.getPacketLossRate() * 100.0f);
          }
          
          // Send to GS
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

// Report LoRa link status similar to GS format
void reportLoraLinkStatus() {
  if (!loraManager.isInitialized()) {
    Serial.println("<FC_LINK:DOWN,NOT_INITIALIZED>");
    return;
  }
  
  // Get packet statistics
  uint16_t sent = loraManager.getPacketsSent();
  uint16_t received = loraManager.getPacketsReceived();
  uint16_t lost = loraManager.getPacketsLost();
  float lossRate = (received > 0) ? loraManager.getPacketLossRate() * 100.0f : 0.0f; // Convert to percentage
  
  // Get RF quality metrics
  int16_t rssi = loraManager.getLastRssi();
  float snr = loraManager.getLastSnr();
  
  // Get timing data
  uint32_t timeSinceLastRx = loraManager.getTimeSinceLastRx();
  uint32_t statsDuration = loraManager.getStatsDuration();
    // Determine link quality based on RSSI and connection status
  String linkQuality;
  bool isConnected = loraManager.isConnected();
  
  if (!isConnected) {
    linkQuality = "DOWN";
  } else if (rssi > -90) {
    linkQuality = "EXCELLENT";
  } else if (rssi > -100) {
    linkQuality = "GOOD";
  } else if (rssi > -110) {
    linkQuality = "FAIR";
  } else {
    linkQuality = "POOR";
  }
    // Format and send link status
  char buffer[128];
  // Format the last RX time in HH:MM:SS format
  char timeStr[16];
  uint32_t seconds = timeSinceLastRx / 1000;
  uint8_t hours = seconds / 3600;
  seconds %= 3600;
  uint8_t minutes = seconds / 60;
  seconds %= 60;
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hours, minutes, (uint8_t)seconds);
  
  // Format the loss rate string properly or use "0.0" to avoid a bare %
  char lossRateStr[16];
  if (sent > 0) {
    snprintf(lossRateStr, sizeof(lossRateStr), "%.1f", lossRate);
  } else {
    strcpy(lossRateStr, "0.0");
  }
  // Check if SNR value is valid (RFM95 returns 0 SNR when no packets received)
  if (received > 0 && snr != 0) {
    snprintf(buffer, sizeof(buffer), 
            "<FC_LINK:%s,RSSI:%d,SNR:%.1f,PKT_SENT:%u,PKT_RECV:%u,LOSS:%s%%%%,LAST_RX:%s>", 
            linkQuality.c_str(), rssi, snr, sent, received, lossRateStr, timeStr);
  } else {
    // No valid SNR, omit it from output
    snprintf(buffer, sizeof(buffer), 
            "<FC_LINK:%s,RSSI:%d,PKT_SENT:%u,PKT_RECV:%u,LOSS:%s%%%%,LAST_RX:%s>", 
            linkQuality.c_str(), rssi, sent, received, lossRateStr, timeStr);
  }
  Serial.println(buffer);
  
  // Reset statistics every hour to avoid overflow
  if (statsDuration > 3600000) {
    loraManager.resetStats();
    Serial.println("<DEBUG:LORA_STATS_AUTO_RESET>");
  }
}

// Removed UART packet loss reporting function as it's causing more problems than it solves

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
    
  // Check for critical commands like DISARM and ENTER_RECOVERY that need priority handling
    bool isEmergencyCommand = false;
    if (strstr(cmdBuffer, "DISARM") || strstr(cmdBuffer, "ENTER_RECOVERY")) {
      isEmergencyCommand = true;
      Serial.println("<DEBUG:PRIORITY_COMMAND_RECEIVED>");
    }
    
    // Process command through the parser
    for (size_t i = 0; i < packet->len; i++) {
      cmdParser.processChar(cmdBuffer[i]);
    }
    // Complete the command with a '>' if it doesn't have one
    if (packet->len == 0 || cmdBuffer[packet->len-1] != '>') {
      cmdParser.processChar('>');
    }
    
    // For critical commands, process the queue multiple times to ensure ACKs are sent quickly
    if (isEmergencyCommand) {
      // Force immediate queue processing multiple times
      loraManager.checkQueue();
      loraManager.checkQueue();
      loraManager.checkQueue();
    } else {
      // Force a single queue check for regular commands
      loraManager.checkQueue();
    }
  }
}

// Function to process binary packets from NAVC
void uartTask() {
  // Check if we're in ARMED, RECOVERY, or TEST state - process data if in these states
  uint8_t currentState = stateManager.getCurrentState();
  bool isArmed = (currentState == STATE_ARMED);
  bool isRecovery = (currentState == STATE_RECOVERY);
  bool isTest = (currentState == STATE_TEST);
  bool shouldProcessTelemetry = (isArmed || isRecovery || isTest);
  
  // If we're in ARMED, RECOVERY, or TEST state, tell NAVC to start streaming telemetry
  // Otherwise, tell it to stop
  static bool lastTelemetryState = false;
  if (shouldProcessTelemetry != lastTelemetryState) {
    if (shouldProcessTelemetry) {
      // Small delay to ensure NAVC is ready to receive command
      delay(5);
      uartManager.sendCommand("<START_TELEMETRY>");
      Serial.println("<DEBUG:REQUESTING_TELEMETRY_START>");
    } else {
      // Small delay to ensure NAVC is ready to receive command
      delay(5);
      uartManager.sendCommand("<STOP_TELEMETRY>");
      Serial.println("<DEBUG:REQUESTING_TELEMETRY_STOP>");
    }
    lastTelemetryState = shouldProcessTelemetry;
  }
    // Check for UART data from NAVC at high frequency
  // Only process telemetry data when in ARMED or RECOVERY states
  // Command responses will still be processed regardless of state (handled internally in uartManager)
  UartPacketStatus status = shouldProcessTelemetry ? uartManager.processUartData() : PACKET_NONE;
  // Report UART statistics only occasionally for bandwidth monitoring
  static unsigned long lastUartDebugTime = 0;
  static int uartFailCount = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastUartDebugTime >= 10000) { // Reduced from 2s to 10s
    lastUartDebugTime = currentTime;
    
    // Check if there's any UART data at all
    int bytesAvailable = Serial2.available();
    unsigned long timeSinceLastPacket = uartManager.getTimeSinceLastPacket();
      // Only report issues or very infrequent periodic stats
    if ((uartManager.getCrcErrors() > 0 && currentTime % 30000 < 1000) || // Report errors every 30s max
        (bytesAvailable == 0 && timeSinceLastPacket > 5000) ||           // Report when no data and last packet was long ago
        (currentTime % 300000 < 1000)) {                                // Periodic report every 5 minutes
      char buffer[128];
      snprintf(buffer, sizeof(buffer), 
              "<UART_MONITOR:RX=%lu,DROP=%lu,ERR=%lu>",
              uartManager.getPacketsReceived(),
              uartManager.getPacketsDropped(),
              uartManager.getCrcErrors());
      Serial.println(buffer);
    }
      // If no data is coming in, try sending a ping to NAVC
    if (bytesAvailable == 0) {      if (uartManager.getPacketsReceived() == 0 || timeSinceLastPacket > 10000) {
        // Only log ping attempts every 5s to reduce verbosity
        static unsigned long lastPingLogTime = 0;
        if (currentTime - lastPingLogTime > 5000) {
          // Check RX pin state for hardware troubleshooting
          int rxPinState = digitalRead(PA2);
          char pingBuf[40];
          snprintf(pingBuf, sizeof(pingBuf), "<NAVC_PING:RX_PIN=%d>", rxPinState);
          Serial.println(pingBuf);
          lastPingLogTime = currentTime;
        }
        
        // Safely send a ping without blocking
        uartManager.sendCommand("<PING>");
        uartFailCount++;
        
        // After 3 consecutive failures (6 seconds total), try reinitializing UART
        // This threshold is reduced from 5 to detect problems earlier
        if (uartFailCount >= 3) {
          // Call our non-blocking reinitialization function
          // It will manage its own state across multiple calls
          reinitializeUart();
          
          // Don't reset uartFailCount here, we'll reset it after we start receiving packets again
          // This prevents repeated reinitialization attempts that might make things worse
          if (uartFailCount >= 10) {
            // If we've tried many times, reset the counter to prevent integer overflow
            // but keep it high enough to continue periodic reinit attempts
            uartFailCount = 5; 
          }
        }
      } else {
        uartFailCount = 0; // Reset counter if we've received packets but just not at this moment
      }} else {      // Data is available, but don't log raw data unless we're in an error state
      // Read bytes into temporary buffer, then add them back
      uint8_t tempBuffer[16]; // Make it bigger just in case
      int bytesToExamine = min(bytesAvailable, 8);
      
      // Read bytes into temporary buffer silently
      for (int i = 0; i < bytesToExamine; i++) {
        tempBuffer[i] = Serial2.read();
      }
      
      // Only print raw data if we're experiencing persistent errors
      static unsigned long lastRawDataTime = 0;
      if (uartFailCount > 2 && currentTime - lastRawDataTime > 15000) {
        char hexBuf[50] = "<RAW_UART:";
        int hexPos = 10; // Start after "<RAW_UART:"
        
        for (int i = 0; i < bytesToExamine && hexPos < 40; i++) {
          hexPos += snprintf(hexBuf + hexPos, sizeof(hexBuf) - hexPos, "%02X ", tempBuffer[i]);
        }
        
        hexBuf[hexPos] = '>';
        hexBuf[hexPos+1] = '\0';
        Serial.println(hexBuf);
        lastRawDataTime = currentTime;
      }
      
      // Put the data back by writing it to a different port and copying back
      for (int i = bytesToExamine - 1; i >= 0; i--) {
        // Since there's no way to push bytes back into the rx buffer,
        // we need to store them and process them separately
        uartManager.storeExaminedByte(tempBuffer[i]);
      }
      
      uartFailCount = 0; // Reset counter when data is available
    }
  }
  
  // If a complete packet is ready, process it
  if (status == PACKET_COMPLETE && uartManager.isPacketReady()) {
    const SensorPacket& packet = uartManager.getLatestPacket();
    
    // Process sensor data based on current state
    switch (stateManager.getCurrentState()) {
      case STATE_ARMED:        // In ARMED state, format full telemetry and send over LoRa
        if (loraManager.isInitialized()) {
          // Check if it's time to send telemetry based on the current rate
          unsigned long now = millis();
          if (now - lastTelemTime >= telemPeriodMs) {
            lastTelemTime = now;
            
            // Before sending new telemetry, implement improved flow control
            // Only queue up to a small number of packets to avoid flooding
            // More aggressively drop packets when queue is getting full
            uint8_t pendingCount = loraManager.getPendingPacketCount();
            
            // Implement a smoother adaptive strategy based on pending packets
            static uint8_t skipCounter = 0;
            bool shouldSkip = false;
            
            if (pendingCount == 0) {
              // No pending packets, always send
              skipCounter = 0;
              shouldSkip = false;
            } else if (pendingCount == 1) {
              // Skip every other packet
              shouldSkip = (++skipCounter % 2 == 0);
            } else if (pendingCount == 2) {
              // Skip 2 out of 3 packets
              shouldSkip = (++skipCounter % 3 != 0);
            } else {
              // Skip this one to avoid overloading
              shouldSkip = true;
              static unsigned long lastSkipReportTime = 0;
              if (now - lastSkipReportTime > 5000) { // Report skips at most every 5 seconds
                Serial.println("<DEBUG:TELEM_SKIPPED:LORA_OVERLOADED>");
                lastSkipReportTime = now;
              }
            }
            
            // Only send if we shouldn't skip
            if (!shouldSkip) {
              // Format telemetry using our new function
              String telemStr = formatTelem(packet, false);
              
              // Send telemetry to debug port and LoRa - don't always echo to debug port
              static uint8_t debugEchoCounter = 0;
              if (++debugEchoCounter >= 5) { // Only echo every 5th packet to reduce debug output
                Serial.println(telemStr);
                debugEchoCounter = 0;
              }
                
              // Add minimal telemetry info at a much lower frequency to reduce debug output
              static uint8_t lastReportedRate = 0;
              static unsigned long lastTelemStatusReport = 0;
              if ((telemRate != lastReportedRate && now - lastTelemStatusReport > 5000) || (now - lastTelemStatusReport > 60000)) { 
                  // Report only when rate changes (and at least 5s since last report) or every minute
                  char rateBuffer[64];
                  snprintf(rateBuffer, sizeof(rateBuffer), 
                           "<TELEM_INFO:RATE=%u_Hz,RSSI=%d>", 
                           telemRate, loraManager.getLastRssi());
                  Serial.println(rateBuffer);
                  lastReportedRate = telemRate;
                  lastTelemStatusReport = now;
              }
              
              // Send the packet over LoRa
              loraManager.sendPacket(LORA_TYPE_TELEM, telemStr.c_str(), telemStr.length());
            }
            
            // Check if we need to adjust telemetry rate based on link quality
            adjustTelemRate();
          }
          
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
          // Recovery mode uses a fixed 1Hz rate to conserve bandwidth
          unsigned long now = millis();
          if (now - lastTelemTime >= 1000) { // 1 Hz fixed rate
            lastTelemTime = now;
            
            // Format telemetry using our new function (gpsOnly=true)
            String telemStr = formatTelem(packet, true);
            
            // Send recovery telemetry to debug port and LoRa
            Serial.println(telemStr);
            loraManager.sendPacket(LORA_TYPE_TELEM, telemStr.c_str(), telemStr.length());
          }
        }        break;      case STATE_TEST:
        // In TEST state, send telemetry at 0.5 Hz (1 packet every 2 seconds) for debugging
        if (loraManager.isInitialized()) {
          unsigned long now = millis();
          if (now - lastTelemTime >= 2000) { // 0.5 Hz (every 2 seconds)
            lastTelemTime = now;
            
            // Format full telemetry like ARMED state but at slower rate
            String telemStr = formatTelem(packet, false);
            
            // Send telemetry to debug port and LoRa for testing
            Serial.println(telemStr);
            loraManager.sendPacket(LORA_TYPE_TELEM, telemStr.c_str(), telemStr.length());
          }
            // Send velocity packets at 10Hz when velocity test is active
          if (stateManager.isVelocityTestEnabled() && stateManager.hasVelocityData()) {
            if (now - lastVelocityPacketTime >= VELOCITY_PACKET_PERIOD_MS) {
              lastVelocityPacketTime = now;
              
              // Debug: Show current velocity test state
              char debugBuffer[128];
              snprintf(debugBuffer, sizeof(debugBuffer), 
                       "<DEBUG:VEL_PACKET:enabled=%d,hasData=%d,vel=%.2f,thresh=%.1f>",
                       stateManager.isVelocityTestEnabled(), stateManager.hasVelocityData(),
                       stateManager.getCurrentVelocity(), stateManager.getVelocityTestThreshold());
              Serial.println(debugBuffer);
              
              // Format velocity packet
              String velocityStr = formatVelocityPacket(packet, 
                                                       stateManager.getCurrentVelocity(),
                                                       stateManager.getVelocityTestThreshold());
              
              // Send velocity packet to debug port and LoRa
              Serial.println(velocityStr);
              loraManager.sendPacket(LORA_TYPE_TELEM, velocityStr.c_str(), velocityStr.length());
            }
          }
        }
        
        // Also report sensor values for testing (every packet)
        char testBuffer[128];
        snprintf(testBuffer, sizeof(testBuffer), 
                "<TEST:ALT:%.2fm,ACCEL:%.2f,%.2f,%.2f>",
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
    uartManager.markPacketProcessed();  } else if (status == PACKET_ERROR) {
    // Log packet errors but batch them to avoid flooding
    static unsigned long lastCrcErrorTime = 0;
    static uint16_t batchErrorCount = 0;
    
    batchErrorCount++;    // Removed CRC error reporting as it's causing more problems than it solves
    // Resetting error count without logging
    if (currentTime - lastCrcErrorTime > 10000 || batchErrorCount >= 100) {
      lastCrcErrorTime = currentTime;
      batchErrorCount = 0;
    }
  }
    // Periodically report UART packet statistics (at a reduced frequency)
  if (currentTime - lastNavcStatsReport >= 10000) { // Every 10 seconds (reduced from 5s)
    lastNavcStatsReport = currentTime;
    
    // Format and send statistics
    char statsBuffer[128];
    
    // Determine NAVC link status based on time since last packet
    String linkStatus = "DOWN";
    unsigned long timeSinceLastPacket = uartManager.getTimeSinceLastPacket();
    if (timeSinceLastPacket < 1000) {
      linkStatus = "EXCELLENT";
    } else if (timeSinceLastPacket < 3000) {
      linkStatus = "GOOD";
    } else if (timeSinceLastPacket < 5000) {
      linkStatus = "FAIR";
    } else if (uartManager.getPacketsReceived() > 0) {
      linkStatus = "POOR";
    }
    
    // Use the packet rate to determine health percentage
    float packetRateHz = uartManager.getPacketRate();
    float healthPct = 100.0f;
    
    if (packetRateHz > 0) {
      // Expected rate is typically ~10Hz for NAVC
      healthPct = min(100.0f, (packetRateHz / 10.0f) * 100.0f);
    } else if (timeSinceLastPacket > 5000) {
      healthPct = 0.0f;
    }
    
    snprintf(statsBuffer, sizeof(statsBuffer),
             "<NAVC_LINK:%s,HEALTH:%.1f%%%%,RECV:%lu,DROPS:%lu,ERRS:%lu,RATE:%.2f,LAST:%lums>",
             linkStatus.c_str(),
             healthPct,
             uartManager.getPacketsReceived(),
             uartManager.getPacketsDropped(),
             uartManager.getCrcErrors(),
             packetRateHz,
             timeSinceLastPacket);
    Serial.println(statsBuffer);
  }
}

/**
 * Try to send LoRa settings if GS is connected.
 * Called periodically to ensure settings are sent
 * but with protective measures to avoid infinite loops.
 */
void trySendLoraSettings() {
  static bool settingsAttempted = false;
  static uint32_t lastAttempt = 0;
  static uint8_t attemptCount = 0;
  const uint32_t RETRY_INTERVAL = 60000; // 1 minute between attempts
  const uint8_t MAX_ATTEMPTS = 3;        // Maximum 3 attempts ever
  
  uint32_t now = millis();
  
  // Check if we've already successfully sent settings
  if (settingsAttempted) {
    return;
  }
  
  // Check if we've reached max attempts
  if (attemptCount >= MAX_ATTEMPTS) {
    return;
  }
  
  // Check if it's time for another attempt
  if (lastAttempt > 0 && (now - lastAttempt < RETRY_INTERVAL)) {
    return;
  }
  
  // Only send settings if we believe GS is connected
  if (loraManager.isConnected()) {
    if (loraManager.sendSettings()) {
      Serial.println("<DEBUG:LORA_SETTINGS_SENT>");
      settingsAttempted = true;
    } else {
      Serial.println("<DEBUG:LORA_SETTINGS_SEND_FAILED>");
    }
    
    lastAttempt = now;
    attemptCount++;
  }
}

void setup() {
  // Initialize serial communication for debug
  Serial.begin(921600); // FC uses a higher baud rate for USB-CDC
  while (!Serial && millis() < 3000); // Wait for Serial, but timeout after 3 seconds
    // Configure buzzer pin
  pinMode(PA0, OUTPUT);
  digitalWrite(PA0, LOW);
  
  // Configure UART pins explicitly
  pinMode(PA2, INPUT_PULLUP); // RX pin as input with pullup
  pinMode(PA3, OUTPUT);      // TX pin as output
  
  // Check NAVC UART settings in startup
  Serial.println("<DEBUG:UART_SETUP:STARTING>");
  
  // Initialize UART communication with NAVC
  Serial2.begin(115200);
  Serial2.setRx(PA2);
  Serial2.setTx(PA3);
  
  // Small delay for UART to fully initialize
  delay(100);
  
  // Try clearing any pending data
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.println("<DEBUG:UART_FLUSH_CHAR:" + String(c) + ">");
  }
  
  // Send a test command to NAVC to check if it's alive with multiple attempts
  Serial.println("<DEBUG:SENDING_NAVC_PING>");
  Serial2.println("<PING>");
  delay(10);
  Serial2.println("<ECHO:UART_TEST>");
  Serial2.flush();
  
  // Initialize report timers
  lastLoraLinkReport = millis(); // Initialize link report timer
  
  // Set a longer connection timeout (30 seconds, matching report interval)
  loraManager.setConnectionTimeout(30000);
  
  // Enable ping mechanism for better link quality assessment
  loraManager.enablePing(true);
  
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
    
    // Wait for a moment to ensure the GS is ready
    delay(1000);
    
    // Process queue but DO NOT send settings yet
    loraManager.checkQueue();
    
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
  if (now - lastLoraCheck >= 5) { // Check LoRa every 5ms (reduced from 10ms) for faster telemetry
    lastLoraCheck = now;
    
    // Check for received packets
    loraManager.checkReceived();
    
    // Process queue (send pending packets, retry failed ones)
    loraManager.checkQueue();
    
    // Process queue again to handle more telemetry packets per loop iteration
    // This second call will prioritize sending telemetry packets
    loraManager.checkQueue();
    
    // Periodically report bandwidth usage to monitor telemetry efficiency
    if (now - lastBandwidthReport >= BANDWIDTH_REPORT_INTERVAL_MS) {
      lastBandwidthReport = now;
      loraManager.reportBandwidthUsage();
    }
    // For more reliability, process the queue more frequently for ACKs when commands are being processed
    static unsigned long lastCommandTime = 0;
    const unsigned long commandProcessingWindow = 3000; // 3 second window after receiving a command (increased from 2s)
    
    if (now - lastCommandTime < commandProcessingWindow) {
      // We are in a post-command processing window, check queue more frequently
      // Process queue up to 3 times for faster command processing
      loraManager.checkQueue();
      loraManager.checkQueue(); 
      loraManager.checkQueue(); // Add another check for critical commands
    }
      // Send ping to check link quality
    loraManager.sendPing();
    
    // Periodically report LoRa link status (every 30 seconds)
    if (now - lastLoraLinkReport >= LORA_LINK_REPORT_PERIOD_MS) {
      lastLoraLinkReport = now;
      reportLoraLinkStatus();
    }    // Removed packet loss reporting as it's causing more problems than it solves
    // No need to track and report UART packet loss statistics
    
    // Periodically check if we should send settings, but only if connected
    static unsigned long lastSettingsCheck = 0;
    if (now - lastSettingsCheck >= 30000) { // Check every 30 seconds
      lastSettingsCheck = now;
      trySendLoraSettings(); // This has safety mechanisms built in
    }
  }
    // Update heartbeat pattern if state changed
  updateHeartbeatPattern();
  
  // Update heartbeat LED and buzzer sounds
  heartbeat.update();
  stateManager.updateState(); // This calls updateBuzzerSound() internally
}

// Reinitialize UART after multiple failures
void reinitializeUart() {
  // State machine for non-blocking UART reinitialization
  enum ReinitState {
    UART_REINIT_START,
    UART_REINIT_END_PORT,
    UART_REINIT_BEGIN_PORT,
    UART_REINIT_SEND_PING,
    UART_REINIT_SEND_ECHO,
    UART_REINIT_COMPLETE
  };
  
  static ReinitState reinitState = UART_REINIT_START;
  static unsigned long reinitStateTime = 0;
  static bool reinitializationInProgress = false;
  
  unsigned long currentTime = millis();
  
  // If we're not already in a reinitialization process, start one
  if (!reinitializationInProgress) {
    reinitState = UART_REINIT_START;
    reinitializationInProgress = true;
    reinitStateTime = currentTime;
    Serial.println("<DEBUG:REINITIALIZING_UART:INITIATED>");
  }
  
  // State machine for reinitialization
  switch (reinitState) {
    case UART_REINIT_START:
      // Configure pins
      pinMode(PA2, INPUT_PULLUP); // RX pin as input with pullup
      pinMode(PA3, OUTPUT);       // TX pin as output
      
      Serial.println("<DEBUG:REINITIALIZING_UART:START>");
      reinitState = UART_REINIT_END_PORT;
      reinitStateTime = currentTime;
      break;
      
    case UART_REINIT_END_PORT:
      if (currentTime - reinitStateTime >= 10) {
        // After small delay, close the port safely
        Serial.println("<DEBUG:REINITIALIZING_UART:CLOSING_PORT>");
        Serial2.end();
        
        reinitState = UART_REINIT_BEGIN_PORT;
        reinitStateTime = currentTime;
      }
      break;
      
    case UART_REINIT_BEGIN_PORT:
      if (currentTime - reinitStateTime >= 50) {
        // After longer delay, reopen the port
        Serial.println("<DEBUG:UART_CONFIG:BAUD=115200,RX=PA2,TX=PA3>");
        Serial2.begin(115200);
        Serial2.setRx(PA2);
        Serial2.setTx(PA3);
        
        reinitState = UART_REINIT_SEND_PING;
        reinitStateTime = currentTime;
      }
      break;
      
    case UART_REINIT_SEND_PING:
      if (currentTime - reinitStateTime >= 20) {
        // Send first test command
        Serial.println("<DEBUG:SENDING_UART_PING>");
        Serial2.println("<PING>");
        
        reinitState = UART_REINIT_SEND_ECHO;
        reinitStateTime = currentTime;
      }
      break;
      
    case UART_REINIT_SEND_ECHO:
      if (currentTime - reinitStateTime >= 20) {
        // Send second test command
        Serial.println("<DEBUG:SENDING_UART_ECHO>");
        Serial2.println("<ECHO:UART_TEST>");
        
        reinitState = UART_REINIT_COMPLETE;
        reinitStateTime = currentTime;
      }
      break;
      
    case UART_REINIT_COMPLETE:
      if (currentTime - reinitStateTime >= 20) {
        // Reset statistics and finish the reinitialization
        Serial2.flush(); // Non-blocking flush
        uartManager.resetStats();
        
        Serial.println("<DEBUG:REINITIALIZING_UART:COMPLETE>");
        
        // Mark reinitialization as complete
        reinitializationInProgress = false;
        reinitState = UART_REINIT_START; // Reset for next time
      }
      break;
  }
}

/**
 * Formats velocity data packet for velocity test monitoring
 * 
 * @param packet The binary sensor packet
 * @param velocity The calculated velocity in cm/s
 * @param threshold The velocity threshold in cm/s
 * @return Formatted velocity packet string
 */
String formatVelocityPacket(const SensorPacket& packet, int32_t velocity, int32_t threshold) {
  char buffer[128];
  
  // Format as: <VEL:altitude_cm,velocity_cm_s,threshold_cm_s,timestamp>
  // Using integer values to avoid float precision issues
  snprintf(buffer, sizeof(buffer), "<VEL:%ld,%ld,%ld,%lu>",
           packet.altitude, velocity, threshold, packet.timestamp);
  
  return String(buffer);
}
