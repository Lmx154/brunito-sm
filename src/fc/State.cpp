/**
 * State.cpp - Finite-State Machine implementation for Flight Controller
 * 
 * Implements the state machine logic, transition rules,
 * and command validation according to the current state.
 */

#include "../include/fc/State.h"
#include "../include/utils/FrameCodec.h"

// Forward declarations for telemetry functions declared in FC.cpp
extern void startTelemetry(uint8_t hz);
extern void adjustTelemRate();

// Pin definitions
const int BUZZER_PIN = PB13;

// Buzzer tone frequencies for state acknowledgments
const int TONE_IDLE = 440;  // A4
const int TONE_TEST = 523;  // C5
const int TONE_ARMED = 659; // E5
const int TONE_RECOVERY = 784; // G5
const int TONE_ERROR = 220; // A3

// Buzzer durations
const unsigned long SHORT_BEEP = 100;
const unsigned long LONG_BEEP = 250;

StateManager::StateManager() : 
    currentState(STATE_IDLE), 
    armedTimestamp(0), 
    lastMotionTimestamp(0),
    inMotion(false),
    buzzerStartTime(0),
    buzzerDuration(0),
    buzzerActive(false),
    currentBuzzerTone(0),
    pendingSoundState(STATE_IDLE),
    soundSequenceStep(0) {
    // Initialize buzzer pin
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
}

SystemState StateManager::getCurrentState() const {
    return currentState;
}

bool StateManager::changeState(SystemState newState) {
    // If we're already in the requested state, consider it a success
    if (currentState == newState) {
        return true;
    }
    
    // Validate transitions
    switch (currentState) {
        case STATE_IDLE:
            // From IDLE, can go to TEST or ARMED
            if (newState != STATE_TEST && newState != STATE_ARMED) {
                return false;
            }
            break;
            
        case STATE_TEST:
            // From TEST, can go to IDLE or ARMED
            if (newState != STATE_IDLE && newState != STATE_ARMED) {
                return false;
            }
            break;
            
        case STATE_ARMED:
            // From ARMED, can go to IDLE or RECOVERY
            if (newState != STATE_IDLE && newState != STATE_RECOVERY) {
                return false;
            }
            break;
            
        case STATE_RECOVERY:
            // From RECOVERY, can only go to IDLE
            if (newState != STATE_IDLE) {
                return false;
            }
            break;
    }    
    
    // If transition is allowed, update state
    // Execute state transitions
    switch (newState) {        case STATE_IDLE:
            // Stop any active systems
            stopBuzzer();
            // Play state change sound
            startBuzzerSound(STATE_IDLE);
            // Report state change
            Serial.println("<DEBUG:STATE_CHANGE:IDLE>");
            Serial.println("<DEBUG:TELEMETRY:STOPPED>");
            break;
              case STATE_TEST:
            // Initialize test mode
            startBuzzerSound(STATE_TEST);
            Serial.println("<DEBUG:STATE_CHANGE:TEST>");
            Serial.println("<DEBUG:TELEMETRY:TEST_MODE>");
            break;
              case STATE_ARMED:
            // Initialize armed mode
            armedTimestamp = millis();
            lastMotionTimestamp = millis();
            inMotion = false;
            startBuzzerSound(STATE_ARMED);
            Serial.println("<DEBUG:STATE_CHANGE:ARMED>");
            
            // Start telemetry at default rate (20Hz)
            Serial.println("<DEBUG:TELEMETRY:STARTING_20HZ>");
            startTelemetry(20);
            break;
              case STATE_RECOVERY:
            // Initialize recovery mode with buzzer
            Serial.println("<DEBUG:STATE_CHANGE:RECOVERY>");
            startBuzzerSound(STATE_RECOVERY);
            Serial.println("<DEBUG:TELEMETRY:RECOVERY_MODE_1HZ>");
            break;
    }
    
    // Update state
    currentState = newState;
    return true;
}

bool StateManager::processCommand(CommandType cmd) {
    // Process commands that might change state
    switch (cmd) {
        case CMD_DISARM:
            // DISARM is always allowed and takes us to IDLE
            if (currentState != STATE_IDLE) {
                changeState(STATE_IDLE);
            }
            // DISARM should always succeed even if already in IDLE state
            return true;
              case CMD_ARM:
            // ARM is allowed from IDLE or TEST, takes us to ARMED
            if (currentState == STATE_IDLE || currentState == STATE_TEST) {
                return changeState(STATE_ARMED);
            } else if (currentState == STATE_ARMED) {
                // Already in ARMED state, still return success
                return true;
            }
            break;
              case CMD_ENTER_TEST:
            // ENTER_TEST is only allowed from IDLE
            if (currentState == STATE_IDLE) {
                return changeState(STATE_TEST);
            } else if (currentState == STATE_TEST) {
                // Already in TEST state, still return success
                return true;
            }
            break;
              case CMD_ENTER_RECOVERY:
            // ENTER_RECOVERY is only allowed from ARMED
            if (currentState == STATE_ARMED) {
                return changeState(STATE_RECOVERY);
            } else if (currentState == STATE_RECOVERY) {
                // Already in RECOVERY state, still return success
                return true;
            }
            break;
            
        case CMD_NAVC_RESET_STATS:
            // This command is always allowed
            // Forward to NAVC via UART2
            Serial2.println("RESET_STATS");
            
            // Log the action
            char buffer[64];
            FrameCodec::formatDebug(buffer, sizeof(buffer), "NAVC_STATS_RESET_REQUESTED");
            Serial.println(buffer);
            return true;
            break;
            
        default:
            // Other commands don't change state, check if they're allowed
            return isCommandAllowed(cmd);
    }
    
    return false;
}

bool StateManager::isCommandAllowed(CommandType cmd) const {
    // Commands that are always allowed in any state
    if (cmd == CMD_DISARM || cmd == CMD_QUERY || cmd == CMD_NAVC_RESET_STATS) {
        return true;
    }
    
    // Check state-specific command permissions
    switch (currentState) {
        case STATE_IDLE:
            // In IDLE, all commands are allowed
            return true;
            
        case STATE_TEST:
            // In TEST, only TEST, QUERY, ARM are allowed
            return (cmd == CMD_TEST || cmd == CMD_ARM);
            
        case STATE_ARMED:
            // In ARMED, all commands except TEST are allowed
            return (cmd != CMD_TEST);
            
        case STATE_RECOVERY:
            // In RECOVERY, only FIND_ME is allowed (DISARM already handled)
            return (cmd == CMD_FIND_ME);
            
        default:
            return false;
    }
}

void StateManager::updateState() {
    // Check for auto-recovery condition when in ARMED state
    if (currentState == STATE_ARMED && shouldAutoRecovery()) {
        changeState(STATE_RECOVERY);
        
        char buffer[64];
        FrameCodec::formatDebug(buffer, sizeof(buffer), "AUTO_RECOVERY_TRIGGERED");
        Serial.println(buffer);
    }
    
    // Update buzzer state (non-blocking sound handling)
    updateBuzzerSound();
}

bool StateManager::shouldAutoRecovery() {
    // Check if we've been in ARMED state with no motion for AUTO_RECOVERY_TIMEOUT
    if (inMotion) {
        return false;
    }
    
    unsigned long now = millis();
    unsigned long armedDuration = now - armedTimestamp;
    unsigned long noMotionDuration = now - lastMotionTimestamp;
    
    // Must be armed for at least AUTO_RECOVERY_TIMEOUT and no motion for the same duration
    return (armedDuration > AUTO_RECOVERY_TIMEOUT && noMotionDuration > AUTO_RECOVERY_TIMEOUT);
}

void StateManager::reportMotion(float accelMagnitude) {
    // Update motion detection based on accelerometer magnitude
    bool motionDetected = abs(accelMagnitude - 1.0) > NO_MOTION_THRESHOLD;
    
    if (motionDetected) {
        lastMotionTimestamp = millis();
        inMotion = true;
    } else {
        // If no motion for a while, update the flag
        if (millis() - lastMotionTimestamp > 10000) { // 10 seconds of no motion
            inMotion = false;
        }
    }
}

// Get state string from enum value (helper for internal use)
const char* StateManager::getStateStringFromEnum(SystemState state) const {
    switch (state) {
        case STATE_IDLE:
            return "IDLE";
        case STATE_TEST:
            return "TEST";
        case STATE_ARMED:
            return "ARMED";
        case STATE_RECOVERY:
            return "RECOVERY";
        default:
            return "UNKNOWN";
    }
}

const char* StateManager::getStateString() const {
    return getStateStringFromEnum(currentState);
}

// Start a buzzer sound pattern for the specific state
void StateManager::startBuzzerSound(SystemState state) {
    // Stop any current sound
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Set up for the new sound sequence
    pendingSoundState = state;
    soundSequenceStep = 0;
    buzzerActive = false;
    buzzerStartTime = millis();
    buzzerDuration = 0;
    
    // Immediate first beep based on state
    switch (state) {
        case STATE_IDLE:
            // Simple low beep for IDLE
            tone(BUZZER_PIN, TONE_IDLE);
            buzzerActive = true;
            buzzerDuration = SHORT_BEEP;
            break;
            
        case STATE_TEST:
            // Higher beep for TEST
            tone(BUZZER_PIN, TONE_TEST);
            buzzerActive = true;
            buzzerDuration = SHORT_BEEP;
            break;
            
        case STATE_ARMED:
            // Ascending two tones for ARMED
            tone(BUZZER_PIN, TONE_ARMED);
            buzzerActive = true;
            buzzerDuration = LONG_BEEP;
            break;
            
        case STATE_RECOVERY:
            // SOS pattern for RECOVERY (starts with first tone)
            tone(BUZZER_PIN, TONE_RECOVERY);
            buzzerActive = true;
            buzzerDuration = SHORT_BEEP;
            break;
            
        default:
            // Error tone
            tone(BUZZER_PIN, TONE_ERROR);
            buzzerActive = true;
            buzzerDuration = LONG_BEEP;
            break;
    }
}

// Update buzzer sounds during each loop iteration
void StateManager::updateBuzzerSound() {
    unsigned long currentTime = millis();
    
    // If a beep is active and its duration has passed, stop it
    if (buzzerActive && (currentTime - buzzerStartTime >= buzzerDuration)) {
        noTone(BUZZER_PIN);
        digitalWrite(BUZZER_PIN, LOW);
        buzzerActive = false;
        
        // Move to next step in the sequence
        soundSequenceStep++;
        
        // A small delay between tones (non-blocking)
        buzzerStartTime = currentTime;
        buzzerDuration = 50; // 50ms gap between tones
        return;
    }
    
    // If we're in the pause between tones
    if (!buzzerActive && (currentTime - buzzerStartTime >= buzzerDuration)) {
        // Play the next tone based on current state and sequence step
        switch (pendingSoundState) {
            case STATE_IDLE:
                // IDLE: Single medium beep
                if (soundSequenceStep == 0) {
                    tone(BUZZER_PIN, 1000);
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = 300;
                    currentBuzzerTone = 1000;
                }
                break;
                
            case STATE_TEST:
                // TEST: Alternating high/low beeps (6 tones total)
                if (soundSequenceStep < 6) {
                    // Even steps are high tones, odd steps are low tones
                    int freq = (soundSequenceStep % 2 == 0) ? 1500 : 800;
                    tone(BUZZER_PIN, freq);
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = 150;
                    currentBuzzerTone = freq;
                }
                break;
                
            case STATE_ARMED:
                // ARMED: Four ascending beeps
                if (soundSequenceStep < 4) {
                    // Increasing frequencies
                    int frequencies[] = {1000, 1500, 2000, 2500};
                    int durations[] = {150, 150, 250, 350};
                    
                    tone(BUZZER_PIN, frequencies[soundSequenceStep]);
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = durations[soundSequenceStep];
                    currentBuzzerTone = frequencies[soundSequenceStep];
                }
                break;
                
            case STATE_RECOVERY:
                // RECOVERY: SOS pattern (simpler version for non-blocking)
                // 3 short, 3 long, 3 short (9 tones total)
                if (soundSequenceStep < 9) {
                    // Short beeps for steps 0-2 and 6-8, long beeps for 3-5
                    int duration = (soundSequenceStep >= 3 && soundSequenceStep <= 5) ? 500 : 200;
                    
                    tone(BUZZER_PIN, 2000);
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = duration;
                    currentBuzzerTone = 2000;
                }
                break;
                
            default:
                break;
        }
    }
}

// Stop the buzzer immediately
void StateManager::stopBuzzer() {
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
    buzzerDuration = 0;
}

