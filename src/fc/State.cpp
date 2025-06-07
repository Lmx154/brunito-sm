/**
 * State.cpp - Thread-safe Finite-State Machine implementation
 * 
 * Implements the state machine logic with FreeRTOS mutex protection
 * for thread-safe state transitions and command validation.
 */

#include "../include/fc/State.h"
#include "../include/utils/FrameCodec.h"
#include "../include/navc/Sensors.h"
#include "../include/fc/UartManager.h"
#include <HardwareTimer.h>
#include <Servo.h>
#include <STM32FreeRTOS.h>

// External objects and mutexes
extern void startTelemetry(uint8_t hz);
extern void adjustTelemRate();
extern UartManager uartManager;
extern class CmdParser cmdParser;
extern SemaphoreHandle_t uartMutex;
extern SemaphoreHandle_t stateMutex;

// Pin definitions
const int BUZZER_PIN = PA0;
const int SERVO_PIN = PB14;

// Buzzer frequencies
const int TONE_IDLE = 2800;
const int TONE_TEST = 2500;
const int TONE_ARMED = 2600;
const int TONE_RECOVERY = 3000;
const int TONE_ERROR = 2400;

// Buzzer durations
const unsigned long SHORT_BEEP = 150;
const unsigned long LONG_BEEP = 400;

#define MAX_VOLUME_PWM_DUTY 32768

// Global variables for the hardware timer
HardwareTimer* buzzerTimer = nullptr;
uint8_t activeBuzzerPin = 0;

// Forward declarations
void toneMaxVolume(uint8_t pin, unsigned int frequency);
void noToneMaxVolume(uint8_t pin);

StateManager::StateManager() : 
    currentState(STATE_IDLE), 
    armedTimestamp(0), 
    idleTimestamp(millis()),  // Initialize with current time since we start in IDLE
    lastMotionTimestamp(0),
    inMotion(false),
    buzzerStartTime(0),
    buzzerDuration(0),
    buzzerActive(false),
    currentBuzzerTone(0),
    pendingSoundState(STATE_IDLE),
    soundSequenceStep(0),
    altitudeTestEnabled(false),
    altitudeTestThreshold(0.0f),
    velocityTestEnabled(false),
    velocityTestThreshold(500),
    velocityTestMinAltitude(121920),
    previousAltitude(0),
    previousTimestamp(0),
    velocityTestTriggered(false),
    currentVelocity(0) {    
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
}

SystemState StateManager::getCurrentState() const {
    // This function is called from multiple threads
    // The caller should hold the stateMutex
    return currentState;
}

bool StateManager::changeState(SystemState newState) {
    // This function should be called with stateMutex held
    if (currentState == newState) {
        return true;
    }
    
    // Validate transitions
    switch (currentState) {
        case STATE_IDLE:
            if (newState != STATE_TEST && newState != STATE_ARMED) {
                return false;
            }
            break;
            
        case STATE_TEST:
            if (newState != STATE_IDLE && newState != STATE_ARMED) {
                return false;
            }
            break;
            
        case STATE_ARMED:
            if (newState != STATE_IDLE && newState != STATE_RECOVERY) {
                return false;
            }
            break;
            
        case STATE_RECOVERY:
            if (newState != STATE_IDLE) {
                return false;
            }
            break;
    }
      // Execute state transitions
    switch (newState) {
        case STATE_IDLE:
            stopBuzzer();
            idleTimestamp = millis();  // Record when we entered IDLE state
            startBuzzerSound(STATE_IDLE);
            Serial.println("<DEBUG:STATE_CHANGE:IDLE>");
            Serial.println("<DEBUG:TELEMETRY:STOPPED>");
            break;
            
        case STATE_TEST:
            startBuzzerSound(STATE_TEST);
            Serial.println("<DEBUG:STATE_CHANGE:TEST>");
            Serial.println("<DEBUG:TELEMETRY:TEST_MODE>");
            break;
            
        case STATE_ARMED:
            armedTimestamp = millis();
            lastMotionTimestamp = millis();
            inMotion = false;
            startBuzzerSound(STATE_ARMED);
            Serial.println("<DEBUG:STATE_CHANGE:ARMED>");
            Serial.println("<DEBUG:TELEMETRY:STARTING_20HZ>");
            startTelemetry(20);
            break;
            
        case STATE_RECOVERY:
            Serial.println("<DEBUG:STATE_CHANGE:RECOVERY>");
            startBuzzerSound(STATE_RECOVERY);
            Serial.println("<DEBUG:TELEMETRY:RECOVERY_MODE_1HZ>");
            break;
    }
    
    currentState = newState;
    return true;
}

bool StateManager::processCommand(CommandType cmd, const char* cmdBuffer) {
    // This function should be called with stateMutex held
    switch (cmd) {
        case CMD_DISARM:
            if (currentState != STATE_IDLE) {
                changeState(STATE_IDLE);
            }
            return true;
            
        case CMD_ARM:
            if (currentState == STATE_IDLE || currentState == STATE_TEST) {
                return changeState(STATE_ARMED);
            } else if (currentState == STATE_ARMED) {
                return true;
            }
            break;
            
        case CMD_ENTER_TEST:
            if (currentState == STATE_IDLE) {
                return changeState(STATE_TEST);
            } else if (currentState == STATE_TEST) {
                return true;
            }
            break;
            
        case CMD_ENTER_RECOVERY:
            if (currentState == STATE_ARMED) {
                return changeState(STATE_RECOVERY);
            } else if (currentState == STATE_RECOVERY) {
                return true;
            }
            break;
            
        case CMD_NAVC_RESET_STATS:
            // Send command to NAVC via UART2
            // Need to be careful with thread safety here
            if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial2.println("RESET_STATS");
                xSemaphoreGive(uartMutex);
            }
            
            char buffer[64];
            FrameCodec::formatDebug(buffer, sizeof(buffer), "NAVC_STATS_RESET_REQUESTED");
            Serial.println(buffer);
            return true;
            
        case CMD_TEST_DEVICE:
            if (isCommandAllowed(cmd)) {
                if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial2.println("<ECHO:BUZZER_TEST>");
                    xSemaphoreGive(uartMutex);
                }
                
                char buffer[64];
                FrameCodec::formatDebug(buffer, sizeof(buffer), "TEST_DEVICE_REQUESTED");
                Serial.println(buffer);
                return true;
            }
            break;
            
        case CMD_TEST_SERVO:
            if (isCommandAllowed(cmd)) {
                // Servo test needs to be blocking, so we'll do it here
                Servo testServo;
                testServo.attach(SERVO_PIN);
                testServo.write(70);
                vTaskDelay(pdMS_TO_TICKS(5000));
                testServo.write(0);
                vTaskDelay(pdMS_TO_TICKS(1000));
                testServo.detach();
                
                char buffer[64];
                FrameCodec::formatDebug(buffer, sizeof(buffer), "SERVO_TEST_COMPLETED");
                Serial.println(buffer);
                return true;
            }
            break;
            
        case CMD_TEST_ALTITUDE:
            if (isCommandAllowed(cmd)) {
                // Get current altitude from NAVC
                SensorPacket packet;
                if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    packet = uartManager.getLatestPacket();
                    xSemaphoreGive(uartMutex);
                }
                
                float altitudeMeters = packet.altitude / 100.0f;
                float altitudeThreshold = 2.0f;
                
                // Parse threshold parameter if provided
                const char* params = strchr(cmdBuffer + 5, ':');
                if (params) {
                    params++;
                    if (strstr(params, "threshold=") != nullptr) {
                        char paramCopy[64];
                        strncpy(paramCopy, params, sizeof(paramCopy) - 1);
                        paramCopy[sizeof(paramCopy) - 1] = '\0';
                        
                        char* end = strchr(paramCopy, ':');
                        if (end) *end = '\0';
                        end = strchr(paramCopy, '>');
                        if (end) *end = '\0';
                        
                        char* token = strtok(paramCopy, ",");
                        while (token != NULL) {
                            char* equals = strchr(token, '=');
                            if (equals) {
                                *equals = '\0';
                                if (strcmp(token, "threshold") == 0) {
                                    int32_t thresholdCm = atol(equals + 1);
                                    altitudeThreshold = thresholdCm / 100.0f;
                                    break;
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
                
                if (altitudeMeters >= altitudeThreshold) {
                    toneMaxVolume(BUZZER_PIN, TONE_TEST);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    noToneMaxVolume(BUZZER_PIN);
                    
                    Servo testServo;
                    testServo.attach(SERVO_PIN);
                    testServo.write(90);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    testServo.write(0);
                    vTaskDelay(pdMS_TO_TICKS(500));
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
            if (isCommandAllowed(cmd)) {
                float threshold = 2.0f;
                
                // Parse threshold parameter
                const char* params = strchr(cmdBuffer + 5, ':');
                if (params) {
                    params++;
                    if (strstr(params, "threshold=") != nullptr) {
                        char paramCopy[64];
                        strncpy(paramCopy, params, sizeof(paramCopy) - 1);
                        paramCopy[sizeof(paramCopy) - 1] = '\0';
                        
                        char* end = strchr(paramCopy, ':');
                        if (end) *end = '\0';
                        end = strchr(paramCopy, '>');
                        if (end) *end = '\0';
                        
                        char* token = strtok(paramCopy, ",");
                        while (token != NULL) {
                            char* equals = strchr(token, '=');
                            if (equals) {
                                *equals = '\0';
                                if (strcmp(token, "threshold") == 0) {
                                    int32_t thresholdCm = atol(equals + 1);
                                    threshold = thresholdCm / 100.0f;
                                    break;
                                }
                            }
                            token = strtok(NULL, ",");
                        }
                    }
                }
                
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
            if (isCommandAllowed(cmd)) {
                // Get current altitude from NAVC
                SensorPacket packet;
                if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    packet = uartManager.getLatestPacket();
                    xSemaphoreGive(uartMutex);
                }
                
                float currentAltitude = packet.altitude / 100.0f;
                float velocityThreshold = 5.0f;
                
                // Parse threshold parameter
                const char* params = strchr(cmdBuffer + 5, ':');
                if (params) {
                    params++;
                    if (strstr(params, "threshold=") != nullptr) {
                        char paramCopy[64];
                        strncpy(paramCopy, params, sizeof(paramCopy) - 1);
                        paramCopy[sizeof(paramCopy) - 1] = '\0';
                        
                        char* endParams = strchr(paramCopy, '>');
                        if (endParams) {
                            *endParams = '\0';
                        }
                        
                        char* token = strtok(paramCopy, ",");
                        while (token != nullptr) {
                            char* equals = strchr(token, '=');
                            if (equals) {
                                *equals = '\0';
                                if (strcmp(token, "threshold") == 0) {
                                    int32_t thresholdInt = atol(equals + 1);
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
                
                bool canCalculateVelocity = (previousTimestamp > 0);
                
                if (canCalculateVelocity) {
                    unsigned long timeDelta = packet.timestamp - previousTimestamp;
                    
                    if (timeDelta > 0 && timeDelta < 1000) {
                        float currentVelocity = (currentAltitude - previousAltitude) / (timeDelta / 1000.0f);
                        
                        snprintf(buffer, sizeof(buffer), "<DEBUG:VELOCITY_TEST:CALCULATED_VEL=%.2fm/s>", currentVelocity);
                        Serial.println(buffer);
                        
                        if (currentVelocity > 0 && currentVelocity <= velocityThreshold && currentAltitude >= velocityTestMinAltitude) {
                            toneMaxVolume(BUZZER_PIN, TONE_TEST);
                            vTaskDelay(pdMS_TO_TICKS(500));
                            noToneMaxVolume(BUZZER_PIN);
                            
                            Servo testServo;
                            testServo.attach(SERVO_PIN);
                            testServo.write(90);
                            vTaskDelay(pdMS_TO_TICKS(500));
                            testServo.write(0);
                            vTaskDelay(pdMS_TO_TICKS(500));
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
                
                previousAltitude = currentAltitude;
                previousTimestamp = packet.timestamp;
                
                Serial.println(buffer);
                return true;
            }
            break;
            
        case CMD_ENABLE_VELOCITY_TEST:
            if (isCommandAllowed(cmd)) {
                int32_t threshold = 500;
                
                // Parse threshold parameter
                const char* params = strchr(cmdBuffer + 5, ':');
                if (params) {
                    params++;
                    if (strstr(params, "threshold=") != nullptr) {
                        char paramCopy[64];
                        strncpy(paramCopy, params, sizeof(paramCopy) - 1);
                        paramCopy[sizeof(paramCopy) - 1] = '\0';
                        
                        char* endParams = strchr(paramCopy, '>');
                        if (endParams) {
                            *endParams = '\0';
                        }
                        
                        char* token = strtok(paramCopy, ",");
                        while (token != nullptr) {
                            char* equals = strchr(token, '=');
                            if (equals) {
                                *equals = '\0';
                                if (strcmp(token, "threshold") == 0) {
                                    int32_t thresholdMetersPerSec = atol(equals + 1);
                                    threshold = thresholdMetersPerSec * 100;
                                    break;
                                }
                            }
                            token = strtok(nullptr, ",");
                        }
                    }
                }
                
                velocityTestEnabled = true;
                velocityTestThreshold = threshold;
                velocityTestTriggered = false;
                previousTimestamp = 0;
                
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
            if (isCommandAllowed(cmd)) {
                velocityTestEnabled = false;
                velocityTestThreshold = 500;
                velocityTestTriggered = false;
                previousTimestamp = 0;
                
                char buffer[64];
                FrameCodec::formatDebug(buffer, sizeof(buffer), "VELOCITY_TEST_DISABLED");
                Serial.println(buffer);
                return true;
            }
            break;
            
        default:
            return isCommandAllowed(cmd);
    }
    
    return false;
}

bool StateManager::isCommandAllowed(CommandType cmd) const {
    // This function should be called with stateMutex held
    if (cmd == CMD_DISARM || cmd == CMD_QUERY || cmd == CMD_NAVC_RESET_STATS) {
        return true;
    }
    
    switch (currentState) {
        case STATE_IDLE:
            return (cmd != CMD_TEST_DEVICE && cmd != CMD_TEST_SERVO && cmd != CMD_TEST_ALTITUDE && 
                    cmd != CMD_ENABLE_ALTITUDE_TEST && cmd != CMD_DISABLE_ALTITUDE_TEST &&
                    cmd != CMD_VELOCITY_TEST && cmd != CMD_ENABLE_VELOCITY_TEST && cmd != CMD_DISABLE_VELOCITY_TEST);
            
        case STATE_TEST:
            return (cmd == CMD_TEST || cmd == CMD_ARM || cmd == CMD_TEST_DEVICE || cmd == CMD_TEST_SERVO || 
                    cmd == CMD_TEST_ALTITUDE || cmd == CMD_ENABLE_ALTITUDE_TEST || cmd == CMD_DISABLE_ALTITUDE_TEST ||
                    cmd == CMD_VELOCITY_TEST || cmd == CMD_ENABLE_VELOCITY_TEST || cmd == CMD_DISABLE_VELOCITY_TEST);
            
        case STATE_ARMED:
            return (cmd != CMD_TEST && cmd != CMD_TEST_DEVICE && cmd != CMD_TEST_SERVO && cmd != CMD_TEST_ALTITUDE &&
                    cmd != CMD_ENABLE_ALTITUDE_TEST && cmd != CMD_DISABLE_ALTITUDE_TEST &&
                    cmd != CMD_VELOCITY_TEST && cmd != CMD_ENABLE_VELOCITY_TEST && cmd != CMD_DISABLE_VELOCITY_TEST);
            
        case STATE_RECOVERY:
            return false;
            
        default:
            return false;
    }
}

void StateManager::updateState() {
    // This function should be called with stateMutex held
    
    // Check for auto-arming condition in IDLE state
    if (currentState == STATE_IDLE && shouldAutoArm()) {
        changeState(STATE_ARMED);
        
        char buffer[64];
        FrameCodec::formatDebug(buffer, sizeof(buffer), "AUTO_ARM_TRIGGERED");
        Serial.println(buffer);
    }
    
    // Check for auto-recovery condition
    if (currentState == STATE_ARMED && shouldAutoRecovery()) {
        changeState(STATE_RECOVERY);
        
        char buffer[64];
        FrameCodec::formatDebug(buffer, sizeof(buffer), "AUTO_RECOVERY_TRIGGERED");
        Serial.println(buffer);
    }
    
    // Check background tests
    checkBackgroundAltitudeTest();
    checkBackgroundVelocityTest();
    
    // Update buzzer sound
    updateBuzzerSound();
}

bool StateManager::shouldAutoRecovery() {
    if (inMotion) {
        return false;
    }
    
    unsigned long now = millis();
    unsigned long armedDuration = now - armedTimestamp;
    unsigned long noMotionDuration = now - lastMotionTimestamp;
    
    return (armedDuration > AUTO_RECOVERY_TIMEOUT && noMotionDuration > AUTO_RECOVERY_TIMEOUT);
}

bool StateManager::shouldAutoArm() {
    unsigned long now = millis();
    unsigned long idleDuration = now - idleTimestamp;
    
    return (idleDuration >= AUTO_ARM_TIMEOUT);
}

void StateManager::reportMotion(float accelMagnitude) {
    // This function should be called with stateMutex held
    bool motionDetected = abs(accelMagnitude - 1.0) > NO_MOTION_THRESHOLD;
    
    if (motionDetected) {
        lastMotionTimestamp = millis();
        inMotion = true;
    } else {
        if (millis() - lastMotionTimestamp > 10000) {
            inMotion = false;
        }
    }
}

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

void StateManager::checkBackgroundVelocityTest() {
    if (!velocityTestEnabled || velocityTestTriggered) {
        return;
    }
    
    SensorPacket packet;
    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        packet = uartManager.getLatestPacket();
        xSemaphoreGive(uartMutex);
    } else {
        return;
    }
    
    int32_t currentAltitudeCm = packet.altitude;
    
    if (previousTimestamp == 0) {
        previousAltitude = currentAltitudeCm;
        previousTimestamp = packet.timestamp;
        return;
    }
    
    unsigned long timeDelta = packet.timestamp - previousTimestamp;
    
    if (timeDelta == 0 || timeDelta > 1000 || packet.timestamp <= previousTimestamp) {
        return;
    }
    
    if (abs(packet.altitude) > 10000000) {
        return;
    }
    
    int32_t altitudeDeltaCm = currentAltitudeCm - previousAltitude;
    currentVelocity = (altitudeDeltaCm * 1000) / (int32_t)timeDelta;
    
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
        char debugBuffer[128];
        snprintf(debugBuffer, sizeof(debugBuffer), 
                "<DEBUG:VELOCITY_TEST:ALT=%ldcm,VEL=%ldcm/s,THRESH=%ldcm/s,MIN_ALT=%ldcm>",
                currentAltitudeCm, currentVelocity, velocityTestThreshold, velocityTestMinAltitude);
        Serial.println(debugBuffer);
        lastDebugTime = millis();
    }
    
    bool velocityAtThreshold = (currentVelocity > 0 && currentVelocity <= velocityTestThreshold);
    bool altitudeMinimumMet = (currentAltitudeCm >= velocityTestMinAltitude);
    bool velocityPositive = (currentVelocity > 0);
    
    if (velocityAtThreshold && altitudeMinimumMet && velocityPositive) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "<DEBUG:BACKGROUND_VELOCITY_TEST_TRIGGERED:ALT=%ldcm,VEL=%ldcm/s,THRESHOLD=%ldcm/s>", 
                currentAltitudeCm, currentVelocity, velocityTestThreshold);
        Serial.println(buffer);
        
        toneMaxVolume(BUZZER_PIN, TONE_TEST);
        vTaskDelay(pdMS_TO_TICKS(500));
        noToneMaxVolume(BUZZER_PIN);
        
        Servo testServo;
        testServo.attach(SERVO_PIN);
        testServo.write(90);
        vTaskDelay(pdMS_TO_TICKS(500));
        testServo.write(0);
        vTaskDelay(pdMS_TO_TICKS(500));
        testServo.detach();
        
        velocityTestTriggered = true;
        velocityTestEnabled = false;
        
        FrameCodec::formatDebug(buffer, sizeof(buffer), "BACKGROUND_VELOCITY_TEST_COMPLETE_DISABLED");
        Serial.println(buffer);
    }
    
    previousAltitude = currentAltitudeCm;
    previousTimestamp = packet.timestamp;
}

void StateManager::checkBackgroundAltitudeTest() {
    if (!altitudeTestEnabled) {
        return;
    }
    
    SensorPacket packet;
    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        packet = uartManager.getLatestPacket();
        xSemaphoreGive(uartMutex);
    } else {
        return;
    }
    
    float currentAltitudeM = packet.altitude / 100.0f;
    
    if (currentAltitudeM >= altitudeTestThreshold) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "<DEBUG:BACKGROUND_ALTITUDE_TEST_TRIGGERED:ALT=%.2fm,THRESHOLD=%.2fm>", 
                currentAltitudeM, altitudeTestThreshold);
        Serial.println(buffer);
        
        toneMaxVolume(BUZZER_PIN, TONE_TEST);
        vTaskDelay(pdMS_TO_TICKS(500));
        noToneMaxVolume(BUZZER_PIN);
        
        Servo testServo;
        testServo.attach(SERVO_PIN);
        testServo.write(90);
        vTaskDelay(pdMS_TO_TICKS(500));
        testServo.write(0);
        vTaskDelay(pdMS_TO_TICKS(500));
        testServo.detach();
        
        altitudeTestEnabled = false;
        
        FrameCodec::formatDebug(buffer, sizeof(buffer), "BACKGROUND_ALTITUDE_TEST_COMPLETE_DISABLED");
        Serial.println(buffer);
    }
}

void StateManager::startBuzzerSound(SystemState state) {
    noToneMaxVolume(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
    
    pendingSoundState = state;
    soundSequenceStep = 0;
    buzzerActive = false;
    buzzerStartTime = millis();
    buzzerDuration = 0;
    
    switch (state) {
        case STATE_IDLE:
            toneMaxVolume(BUZZER_PIN, TONE_IDLE);
            buzzerActive = true;
            buzzerDuration = SHORT_BEEP;
            break;
            
        case STATE_TEST:
            toneMaxVolume(BUZZER_PIN, TONE_TEST);
            buzzerActive = true;
            buzzerDuration = SHORT_BEEP;
            break;
            
        case STATE_ARMED:
            toneMaxVolume(BUZZER_PIN, TONE_ARMED);
            buzzerActive = true;
            buzzerDuration = LONG_BEEP;
            break;
            
        case STATE_RECOVERY:
            toneMaxVolume(BUZZER_PIN, TONE_RECOVERY);
            buzzerActive = true;
            buzzerDuration = SHORT_BEEP;
            break;
            
        default:
            toneMaxVolume(BUZZER_PIN, TONE_ERROR);
            buzzerActive = true;
            buzzerDuration = LONG_BEEP;
            break;
    }
}

void StateManager::updateBuzzerSound() {
    unsigned long currentTime = millis();
    
    if (buzzerActive && (currentTime - buzzerStartTime >= buzzerDuration)) {
        noToneMaxVolume(BUZZER_PIN);
        digitalWrite(BUZZER_PIN, LOW);
        buzzerActive = false;
        
        soundSequenceStep++;
        buzzerStartTime = currentTime;
        buzzerDuration = 30;
        return;
    }
    
    if (!buzzerActive && (currentTime - buzzerStartTime >= buzzerDuration)) {
        switch (pendingSoundState) {
            case STATE_IDLE:
                if (soundSequenceStep < 2) {
                    int frequencies[] = {2800, 2600};
                    toneMaxVolume(BUZZER_PIN, frequencies[soundSequenceStep]);
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = 250;
                    currentBuzzerTone = frequencies[soundSequenceStep];
                }
                break;
                
            case STATE_TEST:
                if (soundSequenceStep < 5) {
                    int frequencies[] = {2500, 2600, 2700, 2800, 2900};
                    toneMaxVolume(BUZZER_PIN, frequencies[soundSequenceStep]);
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = 150;
                    currentBuzzerTone = frequencies[soundSequenceStep];
                }
                break;
                
            case STATE_ARMED:
                if (soundSequenceStep < 3) {
                    int frequency = TONE_IDLE;
                    int duration = 400;
                    
                    toneMaxVolume(BUZZER_PIN, frequency);
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = duration;
                    currentBuzzerTone = frequency;
                }
                break;
                
            case STATE_RECOVERY:
                if (soundSequenceStep < 9) {
                    int frequency = TONE_IDLE;
                    int duration;
                    
                    if (soundSequenceStep < 3 || soundSequenceStep >= 6) {
                        duration = 150;
                    } else {
                        duration = 400;
                    }
                    
                    toneMaxVolume(BUZZER_PIN, frequency);
                    buzzerActive = true;
                    buzzerStartTime = currentTime;
                    buzzerDuration = duration;
                    currentBuzzerTone = frequency;
                } else {
                    if (currentTime - buzzerStartTime >= 2000) {
                        soundSequenceStep = 0;
                        toneMaxVolume(BUZZER_PIN, TONE_IDLE);
                        buzzerActive = true;
                        buzzerStartTime = currentTime;
                        buzzerDuration = 150;
                        currentBuzzerTone = TONE_IDLE;
                    }
                }
                break;
                
            default:
                break;
        }
    }
}

void StateManager::stopBuzzer() {
    noToneMaxVolume(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
    buzzerDuration = 0;
}

void toneMaxVolume(uint8_t pin, unsigned int frequency) {
    noToneMaxVolume(pin);
    
    TIM_TypeDef* timer_tmp = (TIM_TypeDef*)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));
    
    buzzerTimer = new HardwareTimer(timer_tmp);
    
    buzzerTimer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
    buzzerTimer->setOverflow(frequency, HERTZ_FORMAT);
    buzzerTimer->setCaptureCompare(channel, MAX_VOLUME_PWM_DUTY, RESOLUTION_16B_COMPARE_FORMAT);
    
    buzzerTimer->resume();
    
    activeBuzzerPin = pin;
}

void noToneMaxVolume(uint8_t pin) {
    if (buzzerTimer != nullptr && activeBuzzerPin == pin) {
        buzzerTimer->pause();
        delete buzzerTimer;
        buzzerTimer = nullptr;
        activeBuzzerPin = 0;
        
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}