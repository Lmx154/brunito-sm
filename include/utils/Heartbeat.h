/**
 * Heartbeat.h - LED status indicator patterns
 * 
 * This utility flashes the onboard LED in different patterns
 * to indicate the current state of the system. 
 * Used across all modules (NAVC, FC, GS).
 */

#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <Arduino.h>

// Pin definitions - PC13 is defined by the STM32 framework

// Heartbeat patterns - each pattern is a set of on/off durations in ms
enum HeartbeatPattern {
    HB_OFF,      // LED constantly off
    HB_ON,       // LED constantly on  
    HB_IDLE,     // Slow single blink (1Hz) - default state
    HB_TEST,     // Fast double blink (2Hz) - test mode
    HB_ARMED,    // Fast triple blink (2Hz) - armed/active state
    HB_RECOVERY, // SOS pattern - recovery mode
    HB_ERROR     // Fast continuous blinking (4Hz) - error state
};

class HeartbeatManager {
private:
    int ledPin;
    HeartbeatPattern currentPattern;
    unsigned long lastUpdateTime;
    uint8_t patternStep;
    bool ledState;
    
    // Pattern definitions (on/off times in ms)
    const uint16_t patterns[7][10] = {
        {0, 1000, 0, 0, 0, 0, 0, 0, 0, 0},           // HB_OFF - constantly off
        {1000, 0, 0, 0, 0, 0, 0, 0, 0, 0},           // HB_ON - constantly on
        {500, 500, 0, 0, 0, 0, 0, 0, 0, 0},          // HB_IDLE - slow blink
        {200, 200, 200, 400, 0, 0, 0, 0, 0, 0},      // HB_TEST - double blink
        {200, 200, 200, 200, 200, 1000, 0, 0, 0, 0}, // HB_ARMED - triple blink
        {200, 200, 200, 200, 200, 200, 200, 200, 200, 800}, // HB_RECOVERY - SOS
        {125, 125, 0, 0, 0, 0, 0, 0, 0, 0}           // HB_ERROR - fast blink
    };
    
    // How many steps in each pattern before we repeat
    const uint8_t patternSteps[7] = {2, 2, 2, 4, 6, 10, 2};
    
public:    HeartbeatManager(int pin = PC13) : 
        ledPin(pin), 
        currentPattern(HB_IDLE), 
        lastUpdateTime(0),
        patternStep(0),
        ledState(false) {
        pinMode(ledPin, OUTPUT);
        digitalWrite(ledPin, HIGH); // LED is active LOW on Blackpill
    }
    
    void setPattern(HeartbeatPattern pattern) {
        if (pattern != currentPattern) {
            currentPattern = pattern;
            patternStep = 0;
            lastUpdateTime = millis();
            ledState = false;
            digitalWrite(ledPin, HIGH); // Start with LED off (HIGH on Blackpill)
        }
    }
    
    void update() {
        unsigned long currentTime = millis();
        
        if (currentTime - lastUpdateTime >= patterns[currentPattern][patternStep]) {
            lastUpdateTime = currentTime;
            
            // Move to next step in pattern
            patternStep = (patternStep + 1) % patternSteps[currentPattern];
            
            // Toggle LED state (note: LED is active LOW on Blackpill)
            ledState = !ledState;
            digitalWrite(ledPin, ledState ? LOW : HIGH);
        }
    }
};

#endif // HEARTBEAT_H
