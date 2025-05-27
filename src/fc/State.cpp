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
    buzzerStartTime(0),
    buzzerDuration(0),
    buzzerActive(false),
    currentBuzzerTone(0),
    pendingSoundState(STATE_IDLE),
    soundSequenceStep(0),
    altitudeTestEnabled(false),
    altitudeTestThreshold(0.0f) {
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
            break;
              case CMD_TEST_SERVO:
            // This command tests the servo movement on pin PB14
            if (isCommandAllowed(cmd)) {
                // Create a servo object and attach it to the pin
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
            // In IDLE, all commands except TEST_DEVICE, TEST_SERVO, and TEST_ALTITUDE are allowed
            return (cmd != CMD_TEST_DEVICE && cmd != CMD_TEST_SERVO && cmd != CMD_TEST_ALTITUDE && 
                    cmd != CMD_ENABLE_ALTITUDE_TEST && cmd != CMD_DISABLE_ALTITUDE_TEST);
            
        case STATE_TEST:
            // In TEST, only TEST, QUERY, ARM, TEST_DEVICE, TEST_SERVO, TEST_ALTITUDE and altitude test commands are allowed
            return (cmd == CMD_TEST || cmd == CMD_ARM || cmd == CMD_TEST_DEVICE || cmd == CMD_TEST_SERVO || 
                    cmd == CMD_TEST_ALTITUDE || cmd == CMD_ENABLE_ALTITUDE_TEST || cmd == CMD_DISABLE_ALTITUDE_TEST);
            
        case STATE_ARMED:
            // In ARMED, all commands except TEST, TEST_DEVICE, TEST_SERVO, and TEST_ALTITUDE are allowed
            // Note: Background altitude test can run in ARMED state, but enabling/disabling is not allowed
            return (cmd != CMD_TEST && cmd != CMD_TEST_DEVICE && cmd != CMD_TEST_SERVO && cmd != CMD_TEST_ALTITUDE &&
                    cmd != CMD_ENABLE_ALTITUDE_TEST && cmd != CMD_DISABLE_ALTITUDE_TEST);
              case STATE_RECOVERY:
            // In RECOVERY, no commands are allowed except the always-allowed ones (DISARM already handled)
            return false;
            
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
    
    // Check background altitude test if enabled
    checkBackgroundAltitudeTest();
    
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

// Check background altitude test and trigger servo sequence if threshold reached
void StateManager::checkBackgroundAltitudeTest() {
    // Only check if altitude test is enabled
    if (!altitudeTestEnabled) {
        return;
    }
    
    // Get the current altitude from the NAVC's latest packet
    const SensorPacket& packet = uartManager.getLatestPacket();
    
    // Get altitude in meters (packet.altitude is in cm)
    float altitudeMeters = packet.altitude / 100.0f;
    
    // Check if above threshold
    if (altitudeMeters >= altitudeTestThreshold) {
        // Threshold reached! Execute servo sequence
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "<DEBUG:BACKGROUND_ALTITUDE_TEST_TRIGGERED:ALT=%.2fm,THRESHOLD=%.2fm>", 
                 altitudeMeters, altitudeTestThreshold);
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
        altitudeTestThreshold = 0.0f;
        
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
                // ARMED: Maximum volume ascending pattern
                if (soundSequenceStep < 4) {
                    // Frequencies in the high-efficiency range (2.4-3.0 kHz)
                    int frequencies[] = {2400, 2600, 2800, 3000};
                    int durations[] = {200, 200, 250, 350}; // Longer durations for more volume
                    
                    toneMaxVolume(BUZZER_PIN, frequencies[soundSequenceStep]); // Max volume
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = durations[soundSequenceStep];
                    currentBuzzerTone = frequencies[soundSequenceStep];
                }
                break;
                  case STATE_RECOVERY:                
                // RECOVERY: Continuous loud buzzing every 3 seconds
                // Simplified to just continuous loud beeps for maximum audibility
                if (soundSequenceStep < 3) {
                    // Three loud beeps every 3 seconds
                    int baseFreq = 3000; // Maximum volume frequency for most passive buzzers
                    int duration = 200; // Short, loud beeps
                    
                    toneMaxVolume(BUZZER_PIN, baseFreq); // Max volume
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = duration;
                    currentBuzzerTone = baseFreq;
                } else {
                    // After 3 beeps, wait 3 seconds before repeating
                    // Check if 3 seconds have passed since the last beep ended
                    if (currentTime - buzzerStartTime >= 3000) {
                        soundSequenceStep = 0; // Reset sequence to repeat
                        // Immediate restart - no delay
                        toneMaxVolume(BUZZER_PIN, 3000);
                        buzzerActive = true;
                        buzzerStartTime = currentTime;
                        buzzerDuration = 200;
                        currentBuzzerTone = 3000;
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
