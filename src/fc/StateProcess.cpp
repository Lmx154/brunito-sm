/**
 * StateProcess.cpp - Implementation of state-based background processes
 */

#include "../include/fc/StateProcess.h"
#include "../include/fc/State.h"
#include "../include/fc/UartManager.h"
#include "../include/utils/FrameCodec.h"
#include <Servo.h>

// External references to global objects from FC.cpp
extern StateManager stateManager;
extern UartManager uartManager;

// Buzzer tone constants (must match values from State.cpp)
const int TONE_IDLE = 2800;     // Resonant frequency for maximum volume
const int TONE_TEST = 2500;     // Slightly different tone but still in loud range

// Initialize static variables
bool StateProcessManager::payloadDeploymentTriggered = false;
int32_t StateProcessManager::previousAltitude = 0;
uint32_t StateProcessManager::previousTimestamp = 0;
uint32_t StateProcessManager::lastBuzzerBeep = 0;

// Constants for payload deployment
const int32_t PAYLOAD_VELOCITY_THRESHOLD = 500;     // 500 cm/s = 5.0 m/s
const int32_t PAYLOAD_MIN_ALTITUDE = 121920;        // 1219m = 121920 cm
const uint32_t ARMED_BEEP_INTERVAL = 2000;          // 2 seconds between beeps

StateProcessManager::StateProcessManager() : 
    armedProcessHandle(NULL),
    processMutex(NULL),
    currentProcessState(STATE_IDLE) {
}

StateProcessManager::~StateProcessManager() {
    stopAllProcesses();
    if (processMutex) {
        vSemaphoreDelete(processMutex);
    }
}

bool StateProcessManager::begin() {
    // Create mutex for thread safety
    processMutex = xSemaphoreCreateMutex();
    if (processMutex == NULL) {
        return false;
    }
    
    // Initialize variables
    payloadDeploymentTriggered = false;
    previousAltitude = 0;
    previousTimestamp = 0;
    lastBuzzerBeep = 0;
    
    return true;
}

void StateProcessManager::updateProcesses(SystemState newState) {
    // Take mutex
    if (xSemaphoreTake(processMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    // Check if state changed
    if (newState != currentProcessState) {
        // Stop current processes
        stopAllProcesses();
        
        // Start new processes based on state
        switch (newState) {
            case STATE_ARMED:
                startArmedProcesses();
                break;
            
            // Other states don't have background processes yet
            case STATE_IDLE:
            case STATE_TEST:
            case STATE_RECOVERY:
            default:
                break;
        }
        
        currentProcessState = newState;
    }
    
    xSemaphoreGive(processMutex);
}

void StateProcessManager::startArmedProcesses() {
    // Reset payload deployment variables
    payloadDeploymentTriggered = false;
    previousAltitude = 0;
    previousTimestamp = 0;
    lastBuzzerBeep = millis();
    
    // Create armed background process task
    BaseType_t result = xTaskCreate(
        armedProcessTask,
        "ArmedProcess",
        ARMED_PROCESS_STACK_SIZE,
        this,  // Pass this as parameter
        ARMED_PROCESS_PRIORITY,
        &armedProcessHandle
    );
    
    if (result == pdPASS) {
        char buffer[64];
        FrameCodec::formatDebug(buffer, sizeof(buffer), "ARMED_PROCESSES_STARTED");
        Serial.println(buffer);
    }
}

void StateProcessManager::stopArmedProcesses() {
    if (armedProcessHandle != NULL) {
        vTaskDelete(armedProcessHandle);
        armedProcessHandle = NULL;
        
        // Turn off buzzer
        noToneMaxVolume(PA0);
        
        char buffer[64];
        FrameCodec::formatDebug(buffer, sizeof(buffer), "ARMED_PROCESSES_STOPPED");
        Serial.println(buffer);
    }
}

void StateProcessManager::stopAllProcesses() {
    stopArmedProcesses();
}

// Armed state background process task
void StateProcessManager::armedProcessTask(void* pvParameters) {
    StateProcessManager* manager = (StateProcessManager*)pvParameters;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Check if we're still in ARMED state
        if (stateManager.getCurrentState() != STATE_ARMED) {
            // State changed, task will be deleted by manager
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        uint32_t currentTime = millis();
        
        // 1. Handle periodic buzzer beep (every 2 seconds)
        if (currentTime - lastBuzzerBeep >= ARMED_BEEP_INTERVAL) {
            toneMaxVolume(PA0, TONE_IDLE);
            vTaskDelay(pdMS_TO_TICKS(100)); // 100ms beep
            noToneMaxVolume(PA0);
            
            lastBuzzerBeep = currentTime;
        }
        
        // 2. Payload deployment monitoring (similar to velocity test)
        if (!payloadDeploymentTriggered && uartManager.isPacketReady()) {
            const SensorPacket& packet = uartManager.getLatestPacket();
            
            // Get current altitude in cm
            int32_t currentAltitudeCm = packet.altitude;
            
            // Check if we have previous data to calculate velocity
            if (previousTimestamp > 0 && packet.timestamp > previousTimestamp) {
                // Calculate time delta
                uint32_t timeDelta = packet.timestamp - previousTimestamp;
                
                // Validate data
                if (timeDelta > 0 && timeDelta < 1000 && abs(packet.altitude) < 10000000) {
                    // Calculate velocity in cm/s
                    int32_t altitudeDelta = currentAltitudeCm - previousAltitude;
                    int32_t velocity = (altitudeDelta * 1000) / (int32_t)timeDelta;
                    
                    // Check deployment conditions
                    bool velocityAtThreshold = (velocity > 0 && velocity <= PAYLOAD_VELOCITY_THRESHOLD);
                    bool altitudeMinimumMet = (currentAltitudeCm >= PAYLOAD_MIN_ALTITUDE);
                    
                    if (velocityAtThreshold && altitudeMinimumMet) {
                        // Deployment conditions met!
                        char buffer[128];
                        snprintf(buffer, sizeof(buffer), 
                                "<PAYLOAD_DEPLOYMENT:ALT=%ldcm,VEL=%ldcm/s>", 
                                currentAltitudeCm, velocity);
                        Serial.println(buffer);
                        
                        // Sound deployment buzzer
                        toneMaxVolume(PA0, TONE_TEST);
                        vTaskDelay(pdMS_TO_TICKS(500));
                        noToneMaxVolume(PA0);
                        
                        // Deploy payload (servo movement)
                        Servo payloadServo;
                        payloadServo.attach(PB14);
                        
                        payloadServo.write(90);
                        vTaskDelay(pdMS_TO_TICKS(500));
                        
                        payloadServo.write(0);
                        vTaskDelay(pdMS_TO_TICKS(500));
                        
                        payloadServo.detach();
                        
                        // Mark as deployed
                        payloadDeploymentTriggered = true;
                        
                        FrameCodec::formatDebug(buffer, sizeof(buffer), "PAYLOAD_DEPLOYED");
                        Serial.println(buffer);
                    }
                }
            }
            
            // Update previous values for next calculation
            previousAltitude = currentAltitudeCm;
            previousTimestamp = packet.timestamp;
        }
        
        // Run at ~20Hz for responsive monitoring
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}