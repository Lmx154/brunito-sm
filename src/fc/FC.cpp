/**
 * FC.cpp - Flight Controller for Brunito Project
 * 
 * This file contains the Flight Controller (FC) module implementation.
 * Phase 1: Implements the finite-state machine (FSM) and command handling.
 */

#include <Arduino.h>
#include "../include/utils/Heartbeat.h"
#include "../include/utils/FrameCodec.h"
#include "../include/fc/State.h"
#include "../include/fc/CmdParser.h"

// Already defined in Heartbeat.h, no need to redefine here
// Using PC13 directly as it's defined by the STM32 framework

// Global objects
HeartbeatManager heartbeat(PC13); // Using PC13 directly (defined by STM32 framework)
StateManager stateManager;
CmdParser cmdParser(stateManager);

// Task scheduling variables
unsigned long lastStatusUpdate = 0;
unsigned long lastHeartbeatUpdate = 0;

void cmdTask() {
  // Process any incoming serial data
  static bool debugPrinted = false;

  if (Serial.available() > 0 && !debugPrinted) {
    char buffer[64];
    FrameCodec::formatDebug(buffer, sizeof(buffer), "SERIAL_DATA_AVAILABLE");
    Serial.println(buffer);
    debugPrinted = true;
  }
  
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Print the character as hex to debug
    char debugChar[16];
    sprintf(debugChar, "0x%02X", c);
    char buffer[64];
    FrameCodec::formatDebug(buffer, sizeof(buffer), debugChar);
    Serial.println(buffer);
    
    cmdParser.processChar(c);
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
    // Send status update if interval has passed
  if (now - lastStatusUpdate >= interval) {
    lastStatusUpdate = now;
    
    // Send status message using FrameCodec
    char statusBuffer[64];
    FrameCodec::formatStatus(statusBuffer, sizeof(statusBuffer), 
                            stateManager.getStateString(), millis());
    Serial.println(statusBuffer);
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

void setup() {
  // Initialize serial communication for debug
  Serial.begin(921600); // FC uses a higher baud rate for USB-CDC
  while (!Serial && millis() < 3000); // Wait for Serial, but timeout after 3 seconds
  
  // Send initialization messages using FrameCodec
  char buffer[64];
  
  FrameCodec::formatDebug(buffer, sizeof(buffer), "FC_INIT");
  Serial.println(buffer);
    // Add explicit command instructions
  FrameCodec::formatDebug(buffer, sizeof(buffer), "WAITING_FOR_COMMANDS");
  Serial.println(buffer);
  FrameCodec::formatDebug(buffer, sizeof(buffer), "FORMAT: <CMD:COMMAND>");
  Serial.println(buffer);
  FrameCodec::formatDebug(buffer, sizeof(buffer), "FORMAT WITH PARAMS: <CMD:COMMAND:param1=val1,param2=val2>");
  Serial.println(buffer);
  FrameCodec::formatDebug(buffer, sizeof(buffer), "FORMAT WITH CHECKSUM: <CMD:COMMAND:param:XXXX>");
  Serial.println(buffer);
  FrameCodec::formatDebug(buffer, sizeof(buffer), "AVAILABLE: ARM, ENTER_TEST, DISARM, QUERY, CONTROL");
  Serial.println(buffer);
  FrameCodec::formatDebug(buffer, sizeof(buffer), "CONTROL PARAMS: servo=0-180,buzzer=0-1");
  Serial.println(buffer);
  
  FrameCodec::formatDebug(buffer, sizeof(buffer), "FSM_READY");
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
  
  // Update heartbeat pattern if state changed
  updateHeartbeatPattern();
  
  // Update heartbeat LED
  heartbeat.update();
}
