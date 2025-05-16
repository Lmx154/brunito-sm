/**
 * GS.cpp - Ground Station for Brunito Project
 * 
 * This file contains the Ground Station (GS) module implementation.
 * In this initial phase (Phase 0), it simply blinks the onboard LED.
 */

#include <Arduino.h>

// Pin definitions - PC13 is already defined by the STM32 framework as a pin number

void setup() {
  // Initialize serial communication for console
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial, but timeout after 3 seconds
    // Initialize LED pin
  pinMode(PC13, OUTPUT);
  
  Serial.println("<DEBUG:GS_INIT>");
}

void loop() {  // Blink LED at 0.5Hz (GS specific rate - slowest of all)
  digitalWrite(PC13, HIGH);
  delay(1000);
  digitalWrite(PC13, LOW);
  delay(1000);
}
