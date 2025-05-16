/**
 * NAVC.cpp - Navigation Controller for Brunito Project
 * 
 * This file contains the Navigation Controller (NAVC) module implementation.
 * In this initial phase (Phase 0), it simply blinks the onboard LED.
 */

#include <Arduino.h>

// Pin definitions - PC13 is already defined by the STM32 framework as a pin number

void setup() {
  // Initialize serial communication for debug
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial, but timeout after 3 seconds
    // Initialize LED pin
  pinMode(PC13, OUTPUT);
  
  Serial.println("<DEBUG:NAVC_INIT>");
}

void loop() {  // Blink LED at 2Hz (NAVC specific rate)
  digitalWrite(PC13, HIGH);
  delay(250);
  digitalWrite(PC13, LOW);
  delay(250);
}
