/**
 * NAVC.cpp - Navigation Controller for Brunito Project
 * 
 * This file contains the Navigation Controller (NAVC) module implementation.
 * Implements sensor fusion and binary packet streaming over UART.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include "../include/navc/Sensors.h"
#include "../include/navc/Packet.h"
#include "../include/utils/FrameCodec.h"

// Global objects
SensorManager sensorManager;
PacketManager packetManager;

// Status variables
unsigned long lastStatusReportTime = 0;
unsigned long startTime = 0;
bool sensorsInitialized = false;
uint32_t sampleCount = 0;
uint32_t loopCounter = 0;
float actualSampleRate = 0.0f;

// Command buffer
static char cmdBuffer[64];
static uint8_t cmdIndex = 0;
static bool cmdInProgress = false;

// Telemetry control
bool telemetryEnabled = false;      // Start with telemetry disabled
bool usbDebugEnabled = true;        // Enable USB debugging by default

// Function declarations
void reportStatus();
void processFcCommands();
void processCommand();
void sendCommandResponse(const char* response);
uint16_t calculateCrc16(const uint8_t* data, size_t length);

void setup() {
  // Initialize USB CDC serial communication for debugging (high baud rate)
  Serial.begin(921600);
  // Wait for up to 3 seconds for Serial connection to be established
  unsigned long startWaitTime = millis();
  while (!Serial && (millis() - startWaitTime < 3000));
  
  // Initialize I2C at 100kHz for better stability with the magnetometer
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(100000); // 100kHz clock rate for reliable I2C communication
  
  // Initialize pins for UART communication with FC
  pinMode(PA3, OUTPUT); // TX pin (NAVC to FC)
  pinMode(PA2, INPUT_PULLUP); // RX pin (FC to NAVC)
  
  // Initialize Serial2 for UART communication with FC
  // IMPORTANT: According to HWSETUP.md, for NAVC:
  // TX: PA3, RX: PA2
  Serial2.begin(115200);
  Serial2.setTx(PA3); // Corrected pin (PA3 for TX)
  Serial2.setRx(PA2); // Corrected pin (PA2 for RX)
  
  // Clear any pending data to start with clean buffers
  while (Serial2.available()) {
    Serial2.read();
  }
  
  // Status LED is initialized by SensorManager
  Serial.println("<DEBUG:NAVC_INIT>");
  Serial.println("<DEBUG:USB_DEBUG:ENABLED>");
  Serial.println("<DEBUG:BAUD_RATE:921600>");
  Serial.println("<DEBUG:I2C_CLOCK:100kHz>"); // Log the I2C clock speed
  
  // Initialize sensors with more detailed error reporting
  Serial.println("<DEBUG:ATTEMPTING_SENSOR_INIT>");
  
  // Try initialization up to 3 times in case of transient I2C issues
  int initResult = -1;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("<DEBUG:INIT_ATTEMPT:");
    Serial.print(attempt);
    Serial.println(">");
    
    initResult = sensorManager.beginWithDiagnostics();
    if (initResult == 0) {
      sensorsInitialized = true;
      Serial.println("<DEBUG:SENSORS_INITIALIZED>");
      // Set the status LED to green for successful initialization
      sensorManager.setStatusLED(0, 255, 0);
      break;
    } else {
      // Report which specific sensor failed
      char errorBuffer[64];
      snprintf(errorBuffer, sizeof(errorBuffer), "<DEBUG:SENSOR_INIT_FAILED:CODE=%d>", initResult);
      Serial.println(errorBuffer);
      
      // Brief pause before retry
      delay(500);
    }
  }
  
  // If all attempts failed, set red LED
  if (initResult != 0) {
    Serial.println("<DEBUG:ALL_INIT_ATTEMPTS_FAILED>");
    // Set the status LED to red for failed initialization
    sensorManager.setStatusLED(255, 0, 0);
  }
  
  // DEBUG: Send a test message to verify UART is working
  Serial.println("<DEBUG:TESTING_UART_CONNECTION>");
  Serial2.println("<NAVC:READY>");
  Serial2.flush(); // Ensure data is sent
  
  startTime = millis();
}

void loop() {
  // Main loop runs as fast as possible
  loopCounter++;
    if (sensorsInitialized) {
    // Try to update sensors and verify they're still responding
    bool sensorUpdateSuccessful = sensorManager.updateWithDiagnostics();
    
    // If sensor update failed, try to reinitialize
    if (!sensorUpdateSuccessful) {
      Serial.println("<DEBUG:SENSOR_UPDATE_FAILED>");
      Serial.println("<DEBUG:ATTEMPTING_SENSOR_REINIT>");
      
      int reinitResult = sensorManager.beginWithDiagnostics();
      if (reinitResult != 0) {
        char errorBuffer[64];
        snprintf(errorBuffer, sizeof(errorBuffer), "<DEBUG:SENSOR_REINIT_FAILED:CODE=%d>", reinitResult);
        Serial.println(errorBuffer);
        sensorManager.setStatusLED(255, 128, 0); // Orange indicates partial failure
      } else {        Serial.println("<DEBUG:SENSOR_REINIT_SUCCESS>");
        sensorManager.setStatusLED(0, 255, 0); // Green for successful reinitialization
      }
    }
    
    // GPS data is already processed inside updateWithDiagnostics()
    
    // Check if a new packet is ready
    if (sensorManager.isPacketReady()) {
      // Get the latest packet
      SensorPacket packet = sensorManager.getPacket();
      
      // Calculate and set CRC
      packet.crc16 = calculateCrc16(
          (const uint8_t*)&packet, 
          sizeof(SensorPacket) - sizeof(uint16_t)
      );
      
      // If USB debugging is enabled, send detailed telemetry to USB
      if (usbDebugEnabled) {
        // Format telemetry data for USB debugging with more details
        char debugBuffer[256]; // Increased buffer size for more data
          // Format basic telemetry with more comprehensive data
        snprintf(debugBuffer, sizeof(debugBuffer), 
                "<TELEM_DEBUG:ID=%u,ALT=%ld,ACC=%.2f,%.2f,%.2f,GYRO=%.2f,%.2f,%.2f>",
                packet.packetId,
                packet.altitude,
                (float)packet.accelX / 1000.0f,  // Convert from mg to g
                (float)packet.accelY / 1000.0f, 
                (float)packet.accelZ / 1000.0f,
                (float)packet.gyroX / 100.0f,    // Convert from 0.01dps to dps
                (float)packet.gyroY / 100.0f,
                (float)packet.gyroZ / 100.0f);
        Serial.println(debugBuffer);
        
        // Only include GPS and detailed stats in every 5th packet to avoid flooding USB
        if (packet.packetId % 5 == 0) {
          // Additional debugging for GPS
          snprintf(debugBuffer, sizeof(debugBuffer),
                  "<GPS_DEBUG:LAT=%ld,LON=%ld,SAT=%u>",
                  packet.latitude,
                  packet.longitude,
                  sensorManager.getGpsSatelliteCount());
          Serial.println(debugBuffer);
          
          // Additional debugging for packet transmission
          snprintf(debugBuffer, sizeof(debugBuffer),
                  "<UART_DEBUG:QUEUE=%u,SENT=%lu,DROPPED=%lu,LOSS=%.2f%%%%>",
                  packetManager.getQueueSize(),
                  packetManager.getPacketsSent(),
                  packetManager.getPacketsDropped(),
                  packetManager.getPacketLossRate());
          Serial.println(debugBuffer);
        }
      }
      
      // Only send telemetry over UART if explicitly enabled
      if (telemetryEnabled) {
        // Enqueue the packet for transmission
        if (packetManager.enqueuePacket(packet)) {
          sampleCount++;
        }
        
        // Send all queued packets
        packetManager.sendQueuedPackets();
      }
    }
    
    // Calculate actual sample rate and report status every second
    unsigned long currentTime = millis();
    if (currentTime - lastStatusReportTime >= 1000) {
      actualSampleRate = sampleCount / ((currentTime - startTime) / 1000.0f);
      reportStatus();
      lastStatusReportTime = currentTime;
    }
  } else {
    // If sensor initialization failed, blink LED rapidly
    digitalWrite(PC13, (millis() % 200) < 100);
      // Try to reinitialize sensors every 5 seconds
    unsigned long currentTime = millis();
    if (currentTime - lastStatusReportTime >= 5000) {
      Serial.println("<DEBUG:ATTEMPTING_SENSOR_REINIT>");
      int initResult = sensorManager.beginWithDiagnostics();
      if (initResult == 0) {
        sensorsInitialized = true;
        Serial.println("<DEBUG:SENSORS_INITIALIZED>");
        sensorManager.setStatusLED(0, 255, 0);
      } else {
        // Report which specific sensor failed
        char errorBuffer[64];
        snprintf(errorBuffer, sizeof(errorBuffer), "<DEBUG:SENSOR_REINIT_FAILED:CODE=%d>", initResult);
        Serial.println(errorBuffer);
      }
      lastStatusReportTime = currentTime;
    }
  }
  
  // Always process FC commands, regardless of sensor status
  processFcCommands();
}

void reportStatus() {
  char buffer[256]; // Increased buffer for more detailed reports
  SensorPacket packet = sensorManager.getPacket(); // Get the latest packet

  // Consolidated USB Debug Output
  if (usbDebugEnabled) {
    char debugOutput[200]; // Buffer for consolidated debug string
    snprintf(debugOutput, sizeof(debugOutput),
             "NAVC Data: TS=%lu, Lat=%ld, Lon=%ld, Alt=%ld, Sats=%u, AccX=%d, AccY=%d, AccZ=%d, GyroX=%d, GyroY=%d, GyroZ=%d, MagX=%d, MagY=%d, MagZ=%d",
             packet.timestamp,
             packet.latitude,
             packet.longitude,
             packet.altitude,
             packet.satellites,
             packet.accelX,
             packet.accelY,
             packet.accelZ,
             packet.gyroX,
             packet.gyroY,
             packet.gyroZ,
             packet.magX,
             packet.magY,
             packet.magZ);
    Serial.println(debugOutput);
  }
  // Consolidated system statistics - combine multiple metrics in one line
  snprintf(buffer, sizeof(buffer), "<NAVC_STATS:SAMPLE=%.2f,PACKETS=%lu,LOSS=%.2f%%,LOOP=%lu>", 
           actualSampleRate,
           packetManager.getPacketsSent(),
           packetManager.getPacketLossRate(),
           loopCounter);
  Serial.println(buffer);

  // Report sensor health with improved accuracy
  
  // Accelerometer - check for reasonable non-zero values
  bool accelOK = (packet.accelX != 0 || packet.accelY != 0 || packet.accelZ != 0);
  
  // Gyroscope - some drift is normal, so being exactly zero is unlikely
  bool gyroOK = (packet.gyroX != 0 || packet.gyroY != 0 || packet.gyroZ != 0);
  
  // Magnetometer - considered OK if NOT using the placeholder values we defined (10,5,50)
  bool magOK = !(packet.magX == 10 && packet.magY == 5 && packet.magZ == 50);
  char magBuf[30];
  snprintf(magBuf, sizeof(magBuf), "%s(%d,%d,%d)", 
           magOK ? "OK" : "ERR",
           packet.magX, packet.magY, packet.magZ);
  
  // GPS - considered OK only if we have at least 1 satellite
  bool gpsOK = (packet.satellites > 0);
  
  // GPS status includes the satellite count and whether we have a valid fix
  // A valid fix is indicated by latitude/longitude not being the placeholder values (1,1 or 10,10)
  bool hasValidFix = (packet.latitude != 1 && packet.longitude != 1 && 
                     packet.latitude != 10 && packet.longitude != 10);
                     
  char gpsBuf[50];
  snprintf(gpsBuf, sizeof(gpsBuf), "%s(%u,%s)", 
           gpsOK ? "OK" : "ERR",
           packet.satellites,
           hasValidFix ? "FIX" : "NO_FIX");

  // Barometer - check if producing non-zero altitude
  bool baroOK = (packet.altitude != 0);
  
  // RTC - check for reasonable date/time values
  bool rtcOK = (packet.year > 0 && packet.year <= 99 && 
               packet.month >= 1 && packet.month <= 12 && 
               packet.day >= 1 && packet.day <= 31);
               
  char rtcBuf[30];
  snprintf(rtcBuf, sizeof(rtcBuf), "%s(20%02d-%02d-%02d)",
           rtcOK ? "OK" : "ERR",
           packet.year, packet.month, packet.day);
  // Generate comprehensive sensor status report with less verbose format
  snprintf(buffer, sizeof(buffer),
          "<SENSOR_STATUS:AC=%s,GY=%s,MG=%s,GPS=%s,BA=%s,RTC=%s>",
          accelOK ? "OK" : "ERR",
          gyroOK ? "OK" : "ERR",
          magBuf,
          gpsBuf,
          baroOK ? "OK" : "ERR",
          rtcBuf);
  Serial.println(buffer);
    // Add calibration reminder if magnetometer or GPS show issues, but at lower frequency
  static unsigned long lastCalibrationReminder = 0;
  if ((!magOK || !gpsOK) && (millis() - lastCalibrationReminder > 60000)) { // Reduced to once per minute
    char alertBuf[128];
    if (!magOK && !gpsOK) {
      snprintf(alertBuf, sizeof(alertBuf), "<ALERT:CALIBRATION_NEEDED:MAG=FIGURE_8_MOTION,GPS=CLEAR_SKY_VIEW>");
    } else if (!magOK) {
      snprintf(alertBuf, sizeof(alertBuf), "<ALERT:MAG_CALIBRATION_NEEDED:MOVE_IN_FIGURE_8_PATTERN>");
    } else {
      snprintf(alertBuf, sizeof(alertBuf), "<ALERT:GPS_SIGNAL_WEAK:ENSURE_CLEAR_SKY_VIEW>");
    }
    Serial.println(alertBuf);
    lastCalibrationReminder = millis();
  }

  // Reset counter
  loopCounter = 0;
}

void processFcCommands() {
  // Read any commands from the Flight Controller over Serial2
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    
    // Echo the received character for debugging
    //Serial.print(c); // Uncomment for debugging
    
    // Start collecting a new command when '<' is received
    if (c == '<') {
      cmdIndex = 0;
      cmdBuffer[cmdIndex++] = c;
      cmdInProgress = true;
      continue;
    }
    
    // If we're not currently collecting a command, ignore the character
    if (!cmdInProgress) {
      continue;
    }
    
    // Complete command on '>'
    if (c == '>') {
      cmdBuffer[cmdIndex++] = c;
      cmdBuffer[cmdIndex] = '\0'; // Null terminate the command
      
      // Handle the complete command
      processCommand();
      
      // Reset for next command
      cmdInProgress = false;
      cmdIndex = 0;
      continue;
    }
    
    // Add character to buffer if there's space
    if (cmdIndex < sizeof(cmdBuffer) - 1) {
      cmdBuffer[cmdIndex++] = c;
    } else {
      // Buffer overflow, reset
      cmdInProgress = false;
      cmdIndex = 0;
    }
  }
}

void processCommand() {
  // Log the received command for debugging
  Serial.print("<DEBUG:COMMAND_RECEIVED:");
  Serial.print(cmdBuffer);
  Serial.println(">");
  
  // Process known commands
  if (strcmp(cmdBuffer, "<PING>") == 0) {
    sendCommandResponse("<PONG>");
  }
  else if (strcmp(cmdBuffer, "<RESET_STATS>") == 0) {
    packetManager.resetStats();
    sampleCount = 0;
    startTime = millis();
    sendCommandResponse("<ACK:STATS_RESET>");
    Serial.println("<DEBUG:STATS_RESET>");
  }
  else if (strcmp(cmdBuffer, "<START_TELEMETRY>") == 0) {
    // First respond to acknowledge the command immediately
    sendCommandResponse("<ACK:TELEMETRY_STARTED>");
    
    // Then enable telemetry streaming
    telemetryEnabled = true;
    Serial.println("<DEBUG:TELEMETRY_STARTED>");
  }
  else if (strcmp(cmdBuffer, "<STOP_TELEMETRY>") == 0) {
    // First respond to acknowledge the command immediately  
    sendCommandResponse("<ACK:TELEMETRY_STOPPED>");
    
    // Then disable telemetry streaming
    telemetryEnabled = false;
    Serial.println("<DEBUG:TELEMETRY_STOPPED>");
  }
  else if (strcmp(cmdBuffer, "<USB_DEBUG_ON>") == 0) {
    // Enable USB debugging
    usbDebugEnabled = true;
    Serial.println("<DEBUG:USB_DEBUG_ENABLED>");
    sendCommandResponse("<ACK:USB_DEBUG_ENABLED>");
  }
  else if (strcmp(cmdBuffer, "<USB_DEBUG_OFF>") == 0) {
    // Disable USB debugging
    Serial.println("<DEBUG:USB_DEBUG_DISABLED>");
    sendCommandResponse("<ACK:USB_DEBUG_DISABLED>");
    usbDebugEnabled = false;
  }
  else if (strncmp(cmdBuffer, "<ECHO:", 6) == 0) {
    // Echo the command back (remove trailing '>')
    cmdBuffer[cmdIndex-1] = '\0';
    char response[64];
    snprintf(response, sizeof(response), "<ECHO_RESPONSE:%s>", cmdBuffer+6);
    sendCommandResponse(response);
  }
  else {
    // Unknown command
    sendCommandResponse("<NAK:UNKNOWN_COMMAND>");
  }
}

void sendCommandResponse(const char* response) {
  // Send the response to FC via UART with timeout check
  unsigned long sendStartTime = millis();
  
  // Send response and ensure it's properly terminated
  Serial2.println(response);
  
  // Flush with timeout to prevent freezing
  unsigned long flushStartTime = millis();
  Serial2.flush(); // Ensure data is sent
  
  // Calculate how long the flush operation took for debugging
  unsigned long flushDuration = millis() - flushStartTime;
  
  // Log the response for debugging with time info
  if (usbDebugEnabled) {
    char debugBuffer[128];
    snprintf(debugBuffer, sizeof(debugBuffer), 
             "<DEBUG:RESPONSE_SENT:%s,FLUSH_TIME:%lu>", 
             response, flushDuration);
    Serial.println(debugBuffer);
  }
  
  // Add small delay to ensure FC has time to process
  if (flushDuration < 2) {
    delay(2 - flushDuration); // Small delay to prevent buffer issues
  }
}

uint16_t calculateCrc16(const uint8_t* data, size_t length) {
  // CRC16-CCITT (0xFFFF) implementation
  uint16_t crc = 0xFFFF;
  
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  
  return crc;
}
