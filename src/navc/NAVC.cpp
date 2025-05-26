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
#include "../include/navc/SDLogger.h"

// Global objects
SensorManager sensorManager;
PacketManager packetManager;

// Add global SDLogger instance after SensorManager
SDLogger* sdLogger = nullptr;  // Initialize as nullptr for safety

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
  
  // Initialize SD card logger
  sdLogger = new SDLogger(sensorManager.getRTC(), &sensorManager);
  if (sdLogger->begin()) {
    Serial.println("<DEBUG:SD_LOGGER_INITIALIZED>");
  } else {
    Serial.println("<DEBUG:SD_LOGGER_INIT_FAILED>");
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
      
      // Log packet to SD card if logging is active
      if (sdLogger && sdLogger->isLogging()) {
        sdLogger->addSensorPacket(packet);
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
      
      // Mark the packet as consumed to update timing for next packet
      sensorManager.markPacketConsumed();
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
    }  }
  
  // Update SD logger
  if (sdLogger) {
    sdLogger->update();
  }
  
  // Always process FC commands, regardless of sensor status
  processFcCommands();
}

void reportStatus() {
  // Status reporting function - maintains loop timing and statistics
  // Debug output is handled in the main loop, not here
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
  }  else if (strncmp(cmdBuffer, "<ECHO:", 6) == 0) {
    // Check if it's a test command for the buzzer
    if (strcmp(cmdBuffer, "<ECHO:BUZZER_TEST>") == 0) {
      // This is our test buzzer command
      Serial.println("<DEBUG:BUZZER_TEST_RECEIVED>");
      
      // Activate buzzer on pin A0 (using digitalWrite for simple on/off)
      pinMode(PA0, OUTPUT);
      digitalWrite(PA0, HIGH);  // Turn on buzzer
      delay(500);               // Beep for 500ms
      digitalWrite(PA0, LOW);   // Turn off buzzer
      
      sendCommandResponse("<ACK:BUZZER_TEST_COMPLETE>");
    } else {
      // Standard echo command
      // Echo the command back (remove trailing '>')
      cmdBuffer[cmdIndex-1] = '\0';
      char response[64];
      snprintf(response, sizeof(response), "<ECHO_RESPONSE:%s>", cmdBuffer+6);
      sendCommandResponse(response);
    }
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
