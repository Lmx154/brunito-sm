/**
 * NAVC.cpp - Navigation Controller for Brunito Project
 * 
 * This file contains the Navigation Controller (NAVC) module implementation.
 * Phase 4: Implements the sensor fusion and binary packet streaming over UART.
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

// Function declarations
void reportStatus();
void processFcCommands();

void setup() {
  // Initialize serial communication for debug
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial, but timeout after 3 seconds
  
  // Initialize Serial2 for UART communication with FC (PA2: TX, PA3: RX)
  Serial2.begin(115200);
  Serial2.setTx(PA2);
  Serial2.setRx(PA3);
  
  // Status LED is initialized by SensorManager
  Serial.println("<DEBUG:NAVC_INIT>");
  
  // Initialize sensors
  if (sensorManager.begin()) {
    sensorsInitialized = true;
    Serial.println("<DEBUG:SENSORS_INITIALIZED>");
  } else {
    Serial.println("<DEBUG:SENSOR_INIT_FAILED>");
  }
  
  startTime = millis();
}

void loop() {
  // Main loop runs as fast as possible
  loopCounter++;
  
  if (sensorsInitialized) {
    // Update sensors (samples at 100 Hz internally)
    sensorManager.update();
    
    // Check if a new packet is ready
    if (sensorManager.isPacketReady()) {
      // Enqueue the packet for transmission
      if (packetManager.enqueuePacket(sensorManager.getPacket())) {
        sampleCount++;
      }
      
      // Send all queued packets
      packetManager.sendQueuedPackets();
      
      // Calculate actual sample rate every second
      unsigned long currentTime = millis();
      if (currentTime - lastStatusReportTime >= 1000) {
        actualSampleRate = sampleCount / ((currentTime - startTime) / 1000.0f);
        reportStatus();
        lastStatusReportTime = currentTime;
      }
    }
  } else {
    // If sensor initialization failed, blink LED rapidly
    digitalWrite(PC13, (millis() % 200) < 100);
  }
  
  // Process any commands from FC (if implemented)
  processFcCommands();
}

void reportStatus() {
  char buffer[64];
  
  // Report sampling statistics
  snprintf(buffer, sizeof(buffer), "<DEBUG:SAMPLE_RATE:%.2f>", actualSampleRate);
  Serial.println(buffer);
  
  // Report packet statistics
  snprintf(buffer, sizeof(buffer), "<DEBUG:PACKETS_SENT:%lu>", packetManager.getPacketsSent());
  Serial.println(buffer);
  
  snprintf(buffer, sizeof(buffer), "<DEBUG:PACKET_LOSS_RATE:%.2f%%>", packetManager.getPacketLossRate());
  Serial.println(buffer);
  
  // Report loop performance
  snprintf(buffer, sizeof(buffer), "<DEBUG:LOOP_RATE:%lu>", loopCounter);
  Serial.println(buffer);
  
  loopCounter = 0;
}

void processFcCommands() {
  // Read any commands from the Flight Controller over Serial2
  static char cmdBuffer[64];
  static uint8_t cmdIndex = 0;
  static bool cmdComplete = false;
  
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    
    // Simple command parsing - in this phase only implementing minimal command support
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        cmdComplete = true;
      }
    } else if (cmdIndex < sizeof(cmdBuffer) - 1) {
      cmdBuffer[cmdIndex++] = c;
    }
  }
  
  if (cmdComplete) {
    // Process the command
    if (strcmp(cmdBuffer, "RESET_STATS") == 0) {
      packetManager.resetStats();
      sampleCount = 0;
      startTime = millis();
      Serial.println("<DEBUG:STATS_RESET>");
    }
    
    // Reset for next command
    cmdIndex = 0;
    cmdComplete = false;
  }
}
