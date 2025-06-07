/**
 * State.cpp - Finite-State Machine implementation for Flight Controller
 * 
 * Implements the state machine logic, transition rules,
 * and command validation according to the current state.
 */

#include "../include/fc/State.h"
#include "../include/utils/FrameCodec.h"
#include "../include/navc/Sensors.h"  // For SensorPacket structure
#include "../include/fc/UartManager.h" // For UartManager
#include <HardwareTimer.h>
#include <Servo.h>

// Forward declarations for telemetry functions declared in FC.cpp
extern void startTelemetry(uint8_t hz);
extern void adjustTelemRate();
extern UartManager uartManager; // Forward declaration of UartManager instance from FC.cpp

// Pin definitions
const int BUZZER_PIN = PA0;  // Changed from PB13 to PA0 for passive buzzer
const int SERVO_PIN = PB14;  // Servo pin for testing
const int MANUAL_OVERRIDE_PIN = PA1;  // Manual override button pin

// Optimized buzzer frequencies for maximum loudness with passive buzzer
// Frequencies between 2-3 kHz are typically the most efficient for passive buzzers
const int TONE_IDLE = 2800;     // Resonant frequency for maximum volume
const int TONE_TEST = 2500;     // Slightly different tone but still in loud range
const int TONE_ARMED = 2600;    // Another resonant frequency for passive buzzers
const int TONE_RECOVERY = 3000; // High-pitched, very loud alert tone
const int TONE_ERROR = 2400;    // Lower but still efficient tone for errors

// Buzzer durations - increased for maximum loudness perception
const unsigned long SHORT_BEEP = 150;  // Longer short beep for more sound energy
const unsigned long LONG_BEEP = 400;   // Extended long beep for maximum impact

// This defines a preprocessor constant for maximum buzzer volume
#define MAX_VOLUME_PWM_DUTY 32768 // 50% duty cycle for maximum volume

// Forward declarations for new maximum volume buzzer functions
void toneMaxVolume(uint8_t pin, unsigned int frequency);
void noToneMaxVolume(uint8_t pin);

// Global variables for the hardware timer
HardwareTimer* buzzerTimer = nullptr;
uint8_t activeBuzzerPin = 0;

StateManager::StateManager() : 
    currentState(STATE_IDLE), 
    armedTimestamp(0), 
    lastMotionTimestamp(0),
    inMotion(false),
    lastManualOverrideState(false),
    lastManualOverrideCheck(0),
    buzzerStartTime(0),
    buzzerDuration(0),
    buzzerActive(false),
    currentBuzzerTone(0),
    pendingSoundState(STATE_IDLE),
    soundSequenceStep(0),
    altitudeTestEnabled(false),
    altitudeTestThreshold(0.0f),
    velocityTestEnabled(false),
    velocityTestThreshold(500),  // Default 500 cm/s (5.0 m/s)
    velocityTestMinAltitude(121920), // Default 4000ft (121920 cm) minimum altitude
    previousAltitude(0),
    previousTimestamp(0),
    velocityTestTriggered(false),
    currentVelocity(0) {  // Initialize current velocity
    // Initialize buzzer pin
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Initialize manual override pin
    pinMode(MANUAL_OVERRIDE_PIN, INPUT);  // A1 is 0 by default, button makes it 1
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
    switch (newState) {        
        case STATE_IDLE:
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

bool StateManager::processCommand(CommandType cmd, const char* cmdBuffer) {
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
            break;              case CMD_TEST_DEVICE:
            // This command sends a buzzer test request to NAVC
            if (isCommandAllowed(cmd)) {
                // Send the command to NAVC to test buzzer
                // Use a proper command format the NAVC will recognize
                Serial2.println("<ECHO:BUZZER_TEST>");
                
                char buffer[64];
                FrameCodec::formatDebug(buffer, sizeof(buffer), "TEST_DEVICE_REQUESTED");                Serial.println(buffer);
                return true;
            }
            break;        case CMD_TEST_SERVO:
            // This command tests the servo movement on pin PB14
            if (isCommandAllowed(cmd)) {
                // Create a servo object and attach it to the pin
                Servo testServo;
                testServo.attach(SERVO_PIN);
                
                // Move to 90 degrees
                testServo.write(70);
                delay(5000); // Wait for servo to reach position
                
                // Move back to 0 degrees
                testServo.write(0);
                delay(1000); // Wait for servo to reach position
                
                // Detach servo to prevent jitter
                testServo.detach();
                
                char buffer[64];
                FrameCodec::formatDebug(buffer, sizeof(buffer), "SERVO_TEST_COMPLETED");
                Serial.println(buffer);                return true;
            }
            break;
              case CMD_TEST_ALTITUDE:
            // This command tests altitude threshold, buzzer and servo in TEST state
            if (isCommandAllowed(cmd)) {
                // Get the current altitude from the NAVC's latest packet
                const SensorPacket& packet = uartManager.getLatestPacket();
                
                // Get altitude in meters (packet.altitude is in cm)
                float altitudeMeters = packet.altitude / 100.0f;                // Default threshold (in meters) - can be overridden by command parameter
                float altitudeThreshold = 2.0f;
                
                // Check if we received a threshold parameter with the command
                const char* params = strchr(cmdBuffer + 5, ':');
                if (params) {
                    // Skip the colon
                    params++;
                    
                    // Look for threshold parameter
                    if (strstr(params, "threshold=") != nullptr) {
                        // Use CmdParser to extract the value
                        char paramKey[16];
                        int32_t thresholdCm;
                        
                        // Make a copy of the parameter string
                        char paramCopy[64];
                        strncpy(paramCopy, params, sizeof(paramCopy) - 1);
                        paramCopy[sizeof(paramCopy) - 1] = '\0';
                        
                        // Find the end of the parameters (could be a colon for checksum or right bracket)
                        char* end = strchr(paramCopy, ':');
                        if (end) *end = '\0';
                        end = strchr(paramCopy, '>');
                        if (end) *end = '\0';
                        
                        // Use strtok to get threshold parameter
                        char* token = strtok(paramCopy, ",");
                        while (token != NULL) {
                            char* equals = strchr(token, '=');
                            if (equals) {
                                // Extract key
                                size_t keyLen = equals - token;
                                if (keyLen < sizeof(paramKey)) {
                                    strncpy(paramKey, token, keyLen);
                                    paramKey[keyLen] = '\0';
                                    
                                    // If key is threshold, parse value
                                    if (strcmp(paramKey, "threshold") == 0) {
                                        thresholdCm = atol(equals + 1);
                                        // Convert from cm to meters
                                        altitudeThreshold = thresholdCm / 100.0f;
                                        break;
                                    }
                                }
                            }
                            token = strtok(NULL, ",");
                        }
                    }
                }
                
                char buffer[64];
                snprintf(buffer, sizeof(buffer), "<DEBUG:ALTITUDE_TEST:CURRENT_ALT=%.2fm,THRESHOLD=%.2fm>", 
                         altitudeMeters, altitudeThreshold);
                Serial.println(buffer);
                  // Check if above threshold
                if (altitudeMeters >= altitudeThreshold) {
                    // Sound the buzzer
                    toneMaxVolume(BUZZER_PIN, TONE_TEST);
                    delay(500); // Buzzer sound duration
                    noToneMaxVolume(BUZZER_PIN);
                    
                    // Move the servo to 90 degrees and back
                    Servo testServo;
                    testServo.attach(SERVO_PIN);
                    
                    // Move to 90 degrees
                    testServo.write(90);
                    delay(500); // Wait for servo to reach position
                    
                    // Move back to 0 degrees
                    testServo.write(0);
                    delay(500); // Wait for servo to reach position
                    
                    // Detach servo to prevent jitter
                    testServo.detach();
                      FrameCodec::formatDebug(buffer, sizeof(buffer), "ALTITUDE_TEST_TRIGGERED");
                } else {
                    FrameCodec::formatDebug(buffer, sizeof(buffer), "ALTITUDE_BELOW_THRESHOLD");
                }
                
                Serial.println(buffer);
                return true;
            }
            break;
            
        case CMD_ENABLE_ALTITUDE_TEST:
            // Enable background altitude test with specified threshold
            if (isCommandAllowed(cmd)) {
                // Default threshold (in meters) - can be overridden by command parameter
                float threshold = 2.0f;
                
                // Check if we received a threshold parameter with the command
                const char* params = strchr(cmdBuffer + 5, ':');
                if (params) {
                    // Skip the colon
                    params++;
                    
                    // Look for threshold parameter
                    if (strstr(params, "threshold=") != nullptr) {
                        // Use same parsing logic as before
                        char paramKey[16];
                        int32_t thresholdCm;
                        
                        // Make a copy of the parameter string
                        char paramCopy[64];
                        strncpy(paramCopy, params, sizeof(paramCopy) - 1);
                        paramCopy[sizeof(paramCopy) - 1] = '\0';
                        
                        // Find the end of the parameters
                        char* end = strchr(paramCopy, ':');
                        if (end) *end = '\0';
                        end = strchr(paramCopy, '>');
                        if (end) *end = '\0';
                        
                        // Use strtok to get threshold parameter
                        char* token = strtok(paramCopy, ",");
                        while (token != NULL) {
                            char* equals = strchr(token, '=');
                            if (equals) {
                                // Extract key
                                size_t keyLen = equals - token;
                                if (keyLen < sizeof(paramKey)) {
                                    strncpy(paramKey, token, keyLen);
                                    paramKey[keyLen] = '\0';
                                    
                                    // If key is threshold, parse value
                                    if (strcmp(paramKey, "threshold") == 0) {
                                        thresholdCm = atol(equals + 1);
                                        // Convert from cm to meters - no max limit
                                        threshold = thresholdCm / 100.0f;
                                        break;
                                    }
                                }
                            }
                            token = strtok(NULL, ",");
                        }
                    }
                }
                
                // Enable the background altitude test
                altitudeTestEnabled = true;
                altitudeTestThreshold = threshold;
                
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "<DEBUG:ALTITUDE_TEST_ENABLED:THRESHOLD=%.2fm>", threshold);
                Serial.println(buffer);
                FrameCodec::formatDebug(buffer, sizeof(buffer), "ALTITUDE_TEST_ENABLED");
                Serial.println(buffer);
                return true;
            }
            break;
              case CMD_DISABLE_ALTITUDE_TEST:
            // Disable background altitude test
            if (isCommandAllowed(cmd)) {
                altitudeTestEnabled = false;
                altitudeTestThreshold = 0.0f;
                
                char buffer[64];
                FrameCodec::formatDebug(buffer, sizeof(buffer), "ALTITUDE_TEST_DISABLED");
                Serial.println(buffer);
                return true;
            }
            break;
            
        case CMD_VELOCITY_TEST:
            // This command tests velocity threshold and servo in TEST state (one-time test)
            if (isCommandAllowed(cmd)) {
                // Get the current altitude from the NAVC's latest packet
                const SensorPacket& packet = uartManager.getLatestPacket();
                
                // Get altitude in meters (packet.altitude is in cm)
                float currentAltitude = packet.altitude / 100.0f;
                
                // Default threshold (in m/s) - can be overridden by command parameter
                float velocityThreshold = 5.0f;
                
                // Check if we received a threshold parameter with the command
                const char* params = strchr(cmdBuffer + 5, ':');
                if (params) {
                    // Skip the colon
                    params++;
                    
                    // Look for threshold parameter
                    if (strstr(params, "threshold=") != nullptr) {
                        // Parse the threshold value (expecting integer m/s)
                        char paramKey[16];
                        int32_t thresholdInt;
                        
                        // Make a copy of the parameter string
                        char paramCopy[64];
                        strncpy(paramCopy, params, sizeof(paramCopy) - 1);
                        paramCopy[sizeof(paramCopy) - 1] = '\0';
                        
                        // Find the end of the parameters
                        char* endParams = strchr(paramCopy, '>');
                        if (endParams) {
                            *endParams = '\0';
                        }
                        
                        // Parse parameters using strtok
                        char* token = strtok(paramCopy, ",");
                        while (token != nullptr) {
                            // Parse each parameter key=value pair
                            char* equals = strchr(token, '=');
                            if (equals) {
                                *equals = '\0';
                                if (strcmp(token, "threshold") == 0) {
                                    thresholdInt = atol(equals + 1);
                                    velocityThreshold = (float)thresholdInt;
                                    break;
                                }
                            }
                            token = strtok(nullptr, ",");
                        }
                    }
                }
                
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "<DEBUG:VELOCITY_TEST:CURRENT_ALT=%.2fm,THRESHOLD=%.1fm/s>", 
                         currentAltitude, velocityThreshold);
                Serial.println(buffer);
                
                // For one-time test, calculate velocity if we have previous data
                bool canCalculateVelocity = (previousTimestamp > 0);
                
                if (canCalculateVelocity) {
                    // Calculate current velocity
                    unsigned long timeDelta = packet.timestamp - previousTimestamp;
                    
                    if (timeDelta > 0 && timeDelta < 1000) { // Valid time delta
                        float currentVelocity = (currentAltitude - previousAltitude) / (timeDelta / 1000.0f);
                        
                        snprintf(buffer, sizeof(buffer), "<DEBUG:VELOCITY_TEST:CALCULATED_VEL=%.2fm/s>", currentVelocity);
                        Serial.println(buffer);
                        
                        // Check if velocity is positive and at/below threshold (simulating apogee approach)
                        if (currentVelocity > 0 && currentVelocity <= velocityThreshold && currentAltitude >= velocityTestMinAltitude) {
                            // Sound the buzzer
                            toneMaxVolume(BUZZER_PIN, TONE_TEST);
                            delay(500); // Buzzer sound duration
                            noToneMaxVolume(BUZZER_PIN);
                            
                            // Move the servo to 90 degrees and back
                            Servo testServo;
                            testServo.attach(SERVO_PIN);
                            
                            // Move to 90 degrees
                            testServo.write(90);
                            delay(500); // Wait for servo to reach position
                            
                            // Move back to 0 degrees
                            testServo.write(0);
                            delay(500); // Wait for servo to reach position
                            
                            // Detach servo to prevent jitter
                            testServo.detach();
                            
                            FrameCodec::formatDebug(buffer, sizeof(buffer), "VELOCITY_TEST_TRIGGERED");
                        } else {
                            FrameCodec::formatDebug(buffer, sizeof(buffer), "VELOCITY_CONDITIONS_NOT_MET");
                        }
                    } else {
                        FrameCodec::formatDebug(buffer, sizeof(buffer), "VELOCITY_INVALID_TIME_DELTA");
                    }
                } else {
                    FrameCodec::formatDebug(buffer, sizeof(buffer), "VELOCITY_NO_PREVIOUS_DATA");
                }
                
                // Update previous data for next calculation
                previousAltitude = currentAltitude;
                previousTimestamp = packet.timestamp;
                
                Serial.println(buffer);
                return true;
            }
            break;
              case CMD_ENABLE_VELOCITY_TEST:
            // Enable background velocity test with specified threshold
            if (isCommandAllowed(cmd)) {
                // Default threshold (in cm/s) - 500 cm/s = 5.0 m/s
                int32_t threshold = 500;
                
                // Check if we received a threshold parameter with the command
                const char* params = strchr(cmdBuffer + 5, ':');
                if (params) {
                    // Skip the colon
                    params++;
                    
                    // Look for threshold parameter
                    if (strstr(params, "threshold=") != nullptr) {
                        // Parse the threshold value (expecting integer m/s, convert to cm/s)
                        char paramKey[16];
                        int32_t thresholdMetersPerSec;
                        
                        // Make a copy of the parameter string
                        char paramCopy[64];
                        strncpy(paramCopy, params, sizeof(paramCopy) - 1);
                        paramCopy[sizeof(paramCopy) - 1] = '\0';
                        
                        // Find the end of the parameters
                        char* endParams = strchr(paramCopy, '>');
                        if (endParams) {
                            *endParams = '\0';
                        }
                        
                        // Parse parameters using strtok
                        char* token = strtok(paramCopy, ",");
                        while (token != nullptr) {
                            // Parse each parameter key=value pair
                            char* equals = strchr(token, '=');
                            if (equals) {
                                *equals = '\0';
                                if (strcmp(token, "threshold") == 0) {
                                    thresholdMetersPerSec = atol(equals + 1);
                                    // Convert m/s to cm/s
                                    threshold = thresholdMetersPerSec * 100;
                                    break;
                                }
                            }
                            token = strtok(nullptr, ",");
                        }
                    }
                }
                
                // Enable the background velocity test
                velocityTestEnabled = true;
                velocityTestThreshold = threshold;
                velocityTestTriggered = false;
                previousTimestamp = 0; // Reset for fresh calculations
                
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "<DEBUG:VELOCITY_TEST_ENABLED:THRESHOLD=%ldcm/s,MIN_ALT=%ldcm>", 
                         threshold, velocityTestMinAltitude);
                Serial.println(buffer);
                FrameCodec::formatDebug(buffer, sizeof(buffer), "VELOCITY_TEST_ENABLED");
                Serial.println(buffer);
                return true;
            }
            break;
              case CMD_DISABLE_VELOCITY_TEST:
            // Disable background velocity test
            if (isCommandAllowed(cmd)) {
                velocityTestEnabled = false;
                velocityTestThreshold = 500; // Reset to default 500 cm/s (5.0 m/s)
                velocityTestTriggered = false;
                previousTimestamp = 0;
                
                char buffer[64];
                FrameCodec::formatDebug(buffer, sizeof(buffer), "VELOCITY_TEST_DISABLED");
                Serial.println(buffer);
                return true;
            }
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
    switch (currentState) {        case STATE_IDLE:
            // In IDLE, all commands except TEST_DEVICE, TEST_SERVO, TEST_ALTITUDE, and velocity test commands are allowed
            return (cmd != CMD_TEST_DEVICE && cmd != CMD_TEST_SERVO && cmd != CMD_TEST_ALTITUDE && 
                    cmd != CMD_ENABLE_ALTITUDE_TEST && cmd != CMD_DISABLE_ALTITUDE_TEST &&
                    cmd != CMD_VELOCITY_TEST && cmd != CMD_ENABLE_VELOCITY_TEST && cmd != CMD_DISABLE_VELOCITY_TEST);
            
        case STATE_TEST:
            // In TEST, only TEST, QUERY, ARM, TEST_DEVICE, TEST_SERVO, TEST_ALTITUDE, altitude test, and velocity test commands are allowed
            return (cmd == CMD_TEST || cmd == CMD_ARM || cmd == CMD_TEST_DEVICE || cmd == CMD_TEST_SERVO || 
                    cmd == CMD_TEST_ALTITUDE || cmd == CMD_ENABLE_ALTITUDE_TEST || cmd == CMD_DISABLE_ALTITUDE_TEST ||
                    cmd == CMD_VELOCITY_TEST || cmd == CMD_ENABLE_VELOCITY_TEST || cmd == CMD_DISABLE_VELOCITY_TEST);
            
        case STATE_ARMED:
            // In ARMED, all commands except TEST, TEST_DEVICE, TEST_SERVO, TEST_ALTITUDE, and velocity test commands are allowed
            // Note: Background altitude test can run in ARMED state, but enabling/disabling is not allowed
            return (cmd != CMD_TEST && cmd != CMD_TEST_DEVICE && cmd != CMD_TEST_SERVO && cmd != CMD_TEST_ALTITUDE &&
                    cmd != CMD_ENABLE_ALTITUDE_TEST && cmd != CMD_DISABLE_ALTITUDE_TEST &&
                    cmd != CMD_VELOCITY_TEST && cmd != CMD_ENABLE_VELOCITY_TEST && cmd != CMD_DISABLE_VELOCITY_TEST);
              case STATE_RECOVERY:
            // In RECOVERY, no commands are allowed except the always-allowed ones (DISARM already handled)
            return false;
            
        default:
            return false;
    }
}

void StateManager::updateState() {
    // Check for manual override (IDLE to ARMED via button press on A1)
    checkManualOverride();
    
    // Check for auto-recovery condition when in ARMED state
    if (currentState == STATE_ARMED && shouldAutoRecovery()) {
        changeState(STATE_RECOVERY);
        
        char buffer[64];
        FrameCodec::formatDebug(buffer, sizeof(buffer), "AUTO_RECOVERY_TRIGGERED");
        Serial.println(buffer);
    }
    
    // Check background altitude test if enabled
    checkBackgroundAltitudeTest();
    
    // Check background velocity test if enabled
    checkBackgroundVelocityTest();
    
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

// Velocity test data access methods
bool StateManager::isVelocityTestEnabled() const {
    return velocityTestEnabled;
}

int32_t StateManager::getCurrentVelocity() const {
    return currentVelocity;
}

int32_t StateManager::getVelocityTestThreshold() const {
    return velocityTestThreshold;
}

bool StateManager::hasVelocityData() const {
    return (previousTimestamp > 0);
}

// Check for manual override button press (IDLE to ARMED transition)
void StateManager::checkManualOverride() {
    // Only check manual override when in IDLE state
    if (currentState != STATE_IDLE) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // Debounce the button reading
    if (currentTime - lastManualOverrideCheck < MANUAL_OVERRIDE_DEBOUNCE) {
        return;
    }
    
    lastManualOverrideCheck = currentTime;
    
    // Read the current state of the manual override pin
    bool currentPinState = digitalRead(MANUAL_OVERRIDE_PIN);
    
    // Check for button press (transition from LOW to HIGH)
    if (currentPinState && !lastManualOverrideState) {
        // Button pressed! Transition from IDLE to ARMED
        if (changeState(STATE_ARMED)) {
            char buffer[128];
            FrameCodec::formatDebug(buffer, sizeof(buffer), "MANUAL_OVERRIDE_TRIGGERED:IDLE_TO_ARMED");
            Serial.println(buffer);
            
            // Additional debug info
            Serial.println("<DEBUG:MANUAL_OVERRIDE:BUTTON_PRESSED_A1>");
        }
    }
    
    // Update the last state for next comparison
    lastManualOverrideState = currentPinState;
}

// Check background velocity test and trigger servo sequence if velocity threshold conditions are met
void StateManager::checkBackgroundVelocityTest() {
    // Only check if velocity test is enabled and not already triggered
    if (!velocityTestEnabled || velocityTestTriggered) {
        return;
    }
    
    // Get the current altitude from the NAVC's latest packet
    const SensorPacket& packet = uartManager.getLatestPacket();
    
    // Work with altitude directly in cm to avoid float arithmetic
    int32_t currentAltitudeCm = packet.altitude;
    
    // Check if we have previous data to calculate velocity
    if (previousTimestamp == 0) {
        // First measurement - initialize tracking data
        previousAltitude = currentAltitudeCm;
        previousTimestamp = packet.timestamp;
        return;
    }
    
    // Calculate time delta
    unsigned long timeDelta = packet.timestamp - previousTimestamp;
    
    // Validate data - skip invalid packets
    if (timeDelta == 0 || timeDelta > 1000 || packet.timestamp <= previousTimestamp) {
        return; // Skip invalid data
    }
    
    // Validate altitude - basic sanity check (within 100km of ground)
    if (abs(packet.altitude) > 10000000) {
        return; // Skip obviously invalid altitude
    }
    
    // Calculate current velocity in cm/s (positive = ascending, negative = descending)
    // velocity = (altitudeDelta in cm) / (timeDelta in seconds)
    int32_t altitudeDeltaCm = currentAltitudeCm - previousAltitude;
    currentVelocity = (altitudeDeltaCm * 1000) / (int32_t)timeDelta; // cm/s
    
    // Check velocity test conditions (must be positive velocity while going up)
    bool velocityAtThreshold = (currentVelocity > 0 && currentVelocity <= velocityTestThreshold);
    bool altitudeMinimumMet = (currentAltitudeCm >= velocityTestMinAltitude);
    bool velocityPositive = (currentVelocity > 0);      // Debug output every 1 second (1Hz)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
        char debugBuffer[128];
        snprintf(debugBuffer, sizeof(debugBuffer), 
                "<DEBUG:VELOCITY_TEST:ALT=%ldcm,VEL=%ldcm/s,THRESH=%ldcm/s,MIN_ALT=%ldcm>",
                currentAltitudeCm, currentVelocity, velocityTestThreshold, velocityTestMinAltitude);
        Serial.println(debugBuffer);
        lastDebugTime = millis();
    }
    
    // Check if all conditions are met for triggering
    if (velocityAtThreshold && altitudeMinimumMet && velocityPositive) {
        // Threshold reached! Execute servo sequence
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "<DEBUG:BACKGROUND_VELOCITY_TEST_TRIGGERED:ALT=%ldcm,VEL=%ldcm/s,THRESHOLD=%ldcm/s>", 
                currentAltitudeCm, currentVelocity, velocityTestThreshold);
        Serial.println(buffer);
        
        // Sound the buzzer
        toneMaxVolume(BUZZER_PIN, TONE_TEST);
        delay(500); // Buzzer sound duration
        noToneMaxVolume(BUZZER_PIN);
        
        // Move the servo to 90 degrees and back
        Servo testServo;
        testServo.attach(SERVO_PIN);
        
        // Move to 90 degrees
        testServo.write(90);
        delay(500); // Wait for servo to reach position
        
        // Move back to 0 degrees
        testServo.write(0);
        delay(500); // Wait for servo to reach position
        
        // Detach servo to prevent jitter
        testServo.detach();
        
        // Mark as triggered and disable the velocity test after triggering (one-time use)
        velocityTestTriggered = true;
        velocityTestEnabled = false;
        
        FrameCodec::formatDebug(buffer, sizeof(buffer), "BACKGROUND_VELOCITY_TEST_COMPLETE_DISABLED");
        Serial.println(buffer);
    }
    
    // Update previous data for next calculation
    previousAltitude = currentAltitudeCm;
    previousTimestamp = packet.timestamp;
}

// Check background altitude test and trigger servo sequence if altitude threshold is met
void StateManager::checkBackgroundAltitudeTest() {
    // Only check if altitude test is enabled
    if (!altitudeTestEnabled) {
        return;
    }
    
    // Get the current altitude from the NAVC's latest packet
    const SensorPacket& packet = uartManager.getLatestPacket();
    
    // Convert altitude from cm to meters for comparison with threshold
    float currentAltitudeM = packet.altitude / 100.0f;
    
    // Check if altitude meets or exceeds threshold
    if (currentAltitudeM >= altitudeTestThreshold) {
        // Threshold reached! Execute servo sequence
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "<DEBUG:BACKGROUND_ALTITUDE_TEST_TRIGGERED:ALT=%.2fm,THRESHOLD=%.2fm>", 
                currentAltitudeM, altitudeTestThreshold);
        Serial.println(buffer);
        
        // Sound the buzzer
        toneMaxVolume(BUZZER_PIN, TONE_TEST);
        delay(500); // Buzzer sound duration
        noToneMaxVolume(BUZZER_PIN);
        
        // Move the servo to 90 degrees and back
        Servo testServo;
        testServo.attach(SERVO_PIN);
        
        // Move to 90 degrees
        testServo.write(90);
        delay(500); // Wait for servo to reach position
        
        // Move back to 0 degrees
        testServo.write(0);
        delay(500); // Wait for servo to reach position
        
        // Detach servo to prevent jitter
        testServo.detach();
        
        // Disable the altitude test after triggering (one-time use)
        altitudeTestEnabled = false;
        
        FrameCodec::formatDebug(buffer, sizeof(buffer), "BACKGROUND_ALTITUDE_TEST_COMPLETE_DISABLED");
        Serial.println(buffer);
    }
}

// Start a buzzer sound pattern for the specific state
void StateManager::startBuzzerSound(SystemState state) {
    // Stop any current sound
    noToneMaxVolume(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Set up for the new sound sequence
    pendingSoundState = state;
    soundSequenceStep = 0;
    buzzerActive = false;
    buzzerStartTime = millis();
    buzzerDuration = 0;
    
    // Immediate first beep based on state - set to maximum volume frequencies
    switch (state) {
        case STATE_IDLE:
            // Loud initial tone for IDLE
            toneMaxVolume(BUZZER_PIN, TONE_IDLE); // Using custom max volume function
            buzzerActive = true;
            buzzerDuration = SHORT_BEEP;
            break;
            
        case STATE_TEST:
            // Loud initial tone for TEST
            toneMaxVolume(BUZZER_PIN, TONE_TEST); // Using custom max volume function
            buzzerActive = true;
            buzzerDuration = SHORT_BEEP;
            break;
            
        case STATE_ARMED:
            // Loud initial tone for ARMED
            toneMaxVolume(BUZZER_PIN, TONE_ARMED); // Using custom max volume function
            buzzerActive = true;
            buzzerDuration = LONG_BEEP;
            break;
            
        case STATE_RECOVERY:
            // Extra loud tone for RECOVERY
            toneMaxVolume(BUZZER_PIN, TONE_RECOVERY); // Using custom max volume function
            buzzerActive = true;
            buzzerDuration = SHORT_BEEP;
            break;
            
        default:
            // Error tone - distinctive loud tone
            toneMaxVolume(BUZZER_PIN, TONE_ERROR); // Using custom max volume function
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
        noToneMaxVolume(BUZZER_PIN); // Use our custom max volume function
        digitalWrite(BUZZER_PIN, LOW);
        buzzerActive = false;
        
        // Move to next step in the sequence
        soundSequenceStep++;
        
        // A minimal delay between tones (non-blocking) - shorter for more continuous sound
        buzzerStartTime = currentTime;
        buzzerDuration = 30; // 30ms gap between tones - reduced for maximum perceived volume
        return;
    }
    
    // If we're in the pause between tones
    if (!buzzerActive && (currentTime - buzzerStartTime >= buzzerDuration)) {
        // Play the next tone based on current state and sequence step
        switch (pendingSoundState) {
            case STATE_IDLE:
                // IDLE: Two-tone notification (maximum volume)
                if (soundSequenceStep < 2) {
                    // Two loud tones at resonant frequencies
                    int frequencies[] = {2800, 2600};
                    toneMaxVolume(BUZZER_PIN, frequencies[soundSequenceStep]); // Max volume
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = 250; // Longer duration for more perceived volume
                    currentBuzzerTone = frequencies[soundSequenceStep];
                }
                break;
                
            case STATE_TEST:
                // TEST: Maximum volume resonant frequency sweep
                if (soundSequenceStep < 5) {
                    // Frequencies in the 2.5-3kHz range for maximum loudness
                    int frequencies[] = {2500, 2600, 2700, 2800, 2900};
                    toneMaxVolume(BUZZER_PIN, frequencies[soundSequenceStep]); // Max volume
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = 150; // Slightly longer for more sound energy
                    currentBuzzerTone = frequencies[soundSequenceStep];
                }
                break;
                  case STATE_ARMED:
                // ARMED: 3 long beeps at IDLE frequency (2800 Hz)
                if (soundSequenceStep < 3) {
                    // Use same frequency as IDLE (2800 Hz) for 3 long beeps
                    int frequency = TONE_IDLE; // 2800 Hz - same as IDLE
                    int duration = 400; // Long beep duration
                    
                    toneMaxVolume(BUZZER_PIN, frequency); // Max volume
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = duration;
                    currentBuzzerTone = frequency;
                }
                break;                case STATE_RECOVERY:                
                // RECOVERY: SOS pattern using IDLE frequency (2800 Hz) for maximum loudness
                // SOS = · · · — — — · · · (3 short, 3 long, 3 short)
                if (soundSequenceStep < 9) {
                    int frequency = TONE_IDLE; // Use IDLE frequency (2800 Hz) - loudest measured
                    int duration;
                    
                    // SOS pattern: short-short-short-long-long-long-short-short-short
                    if (soundSequenceStep < 3 || soundSequenceStep >= 6) {
                        duration = 150; // Short beeps (dots)
                    } else {
                        duration = 400; // Long beeps (dashes)
                    }
                    
                    toneMaxVolume(BUZZER_PIN, frequency);
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = duration;
                    currentBuzzerTone = frequency;
                } else {
                    // After SOS sequence, wait 2 seconds before repeating
                    if (currentTime - buzzerStartTime >= 2000) {
                        soundSequenceStep = 0; // Reset sequence to repeat SOS
                        // Immediate restart - no delay
                        toneMaxVolume(BUZZER_PIN, TONE_IDLE);
                        buzzerActive = true;
                        buzzerStartTime = currentTime;
                        buzzerDuration = 150; // Start with first short beep
                        currentBuzzerTone = TONE_IDLE;
                    }
                }
                break;
                
            default:
                break;
        }
    }
}

// Stop the buzzer immediately
void StateManager::stopBuzzer() {
    noToneMaxVolume(BUZZER_PIN); // Use our custom function to properly clean up the timer
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
    buzzerDuration = 0;
}

/**
 * Plays a tone at maximum possible volume by directly configuring 
 * the PWM with 50% duty cycle for maximum power output
 * 
 * @param pin The pin connected to the buzzer 
 * @param frequency The frequency to play
 */
void toneMaxVolume(uint8_t pin, unsigned int frequency) {
    // Stop any previous tone
    noToneMaxVolume(pin);
    
    // Get the timer channel attached to this pin
    TIM_TypeDef* timer_tmp = (TIM_TypeDef*)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));
    
    // Create a hardware timer instance
    buzzerTimer = new HardwareTimer(timer_tmp);
    
    // Configure PWM with 50% duty cycle for maximum volume
    buzzerTimer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
    buzzerTimer->setOverflow(frequency, HERTZ_FORMAT);
    buzzerTimer->setCaptureCompare(channel, MAX_VOLUME_PWM_DUTY, RESOLUTION_16B_COMPARE_FORMAT);
    
    // Start the timer
    buzzerTimer->resume();
    
    // Remember the active pin
    activeBuzzerPin = pin;
}

/**
 * Stops the tone on the specified pin
 * 
 * @param pin The pin connected to the buzzer
 */
void noToneMaxVolume(uint8_t pin) {
    // Only proceed if we have an active timer and it's the right pin
    if (buzzerTimer != nullptr && activeBuzzerPin == pin) {
        buzzerTimer->pause();
        delete buzzerTimer;
        buzzerTimer = nullptr;
        activeBuzzerPin = 0;
        
        // Reset the pin
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}
