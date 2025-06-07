/**
 * BackgroundTask.cpp - FreeRTOS Background Task Manager
 * 
 * This file implements the BackgroundTask system using FreeRTOS
 * with proper synchronization and thread safety.
 */

#include <STM32FreeRTOS.h>
#include "../include/fc/BackgroundTask.h"
#include "../include/fc/State.h"
#include "../include/fc/CmdParser.h"
#include "../include/utils/FrameCodec.h"
#include "../include/config/lora.h"

// External mutex for thread safety
extern SemaphoreHandle_t stateMutex;

// Static member definitions
TaskHandle_t BackgroundTask::backgroundTaskHandle = NULL;
StateManager* BackgroundTask::stateManager = nullptr;
CmdParser* BackgroundTask::cmdParser = nullptr;
unsigned long BackgroundTask::lastBeepTime = 0;
bool BackgroundTask::initialized = false;

// Task configuration
const uint16_t BACKGROUND_TASK_STACK = 1024;  // Stack size in words
const UBaseType_t BACKGROUND_TASK_PRIORITY = 2;  // Medium priority

bool BackgroundTask::begin(StateManager* stateMgr, CmdParser* cmdPsr) {
    if (initialized) {
        return true;
    }
    
    if (stateMgr == nullptr || cmdPsr == nullptr) {
        Serial.println("<DEBUG:BACKGROUND_TASK_INIT_FAILED:NULL_PARAMS>");
        return false;
    }
      // Store references
    stateManager = stateMgr;
    cmdParser = cmdPsr;
    
    // Initialize timing
    lastBeepTime = millis();
    
    // Create the background task
    BaseType_t taskResult = xTaskCreate(
        backgroundTaskFunction,
        "Background",
        BACKGROUND_TASK_STACK,
        NULL,
        BACKGROUND_TASK_PRIORITY,
        &backgroundTaskHandle
    );
    
    if (taskResult != pdPASS) {
        Serial.println("<DEBUG:BACKGROUND_TASK_CREATE_FAILED>");
        return false;
    }
    
    initialized = true;
    Serial.println("<DEBUG:BACKGROUND_TASK_CREATED>");
    return true;
}

void BackgroundTask::stop() {
    if (backgroundTaskHandle != NULL) {
        vTaskDelete(backgroundTaskHandle);
        backgroundTaskHandle = NULL;
        initialized = false;
        Serial.println("<DEBUG:BACKGROUND_TASK_STOPPED>");
    }
}

bool BackgroundTask::isRunning() {
    return (backgroundTaskHandle != NULL && initialized);
}

void BackgroundTask::getTaskInfo(char* buffer, size_t bufferSize) {
    if (buffer == nullptr || bufferSize == 0) {
        return;
    }
    
    if (isRunning()) {
        snprintf(buffer, bufferSize, "<BACKGROUND_TASK:RUNNING,STACK_FREE:%u>", 
                 uxTaskGetStackHighWaterMark(backgroundTaskHandle));
    } else {
        snprintf(buffer, bufferSize, "<BACKGROUND_TASK:STOPPED>");
    }
}

void BackgroundTask::backgroundTaskFunction(void *pvParameters) {
    Serial.println("<DEBUG:BACKGROUND_TASK_STARTED>");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xDelay = pdMS_TO_TICKS(TASK_DELAY_MS);
    
    for (;;) {
        // Safety check
        if (stateManager == nullptr) {
            Serial.println("<DEBUG:BACKGROUND_TASK_ERROR:NULL_STATE_MANAGER>");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Get current state with mutex protection
        SystemState currentState = STATE_IDLE;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            currentState = stateManager->getCurrentState();
            xSemaphoreGive(stateMutex);
        }
        
        // Process state-specific background tasks
        switch (currentState) {
            case STATE_IDLE:
                processIdleState();
                break;
                
            case STATE_ARMED:
                processArmedState();
                break;
                
            case STATE_TEST:
                processTestState();
                break;
                
            case STATE_RECOVERY:
                processRecoveryState();
                break;
                
            default:
                break;
        }
        
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

void BackgroundTask::processIdleState() {
    // No specific background processing needed for IDLE state
    // Auto-arming is now handled by StateManager::updateState()
}

void BackgroundTask::processArmedState() {
    unsigned long currentTime = millis();
    
    // Beep every 2 seconds
    if (currentTime - lastBeepTime >= ARMED_BEEP_INTERVAL_MS) {
        // Use the StateManager's buzzer functions
        // Note: We need to ensure these are thread-safe
        toneMaxVolume(BUZZER_PIN, TONE_IDLE);
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms beep
        noToneMaxVolume(BUZZER_PIN);
        
        lastBeepTime = currentTime;
    }
}

void BackgroundTask::processTestState() {
    // Currently no specific background processes for TEST state
    // Future TEST state background logic can be added here
}

void BackgroundTask::processRecoveryState() {
    // Currently no specific background processes for RECOVERY state
    // Future RECOVERY state background logic can be added here
}

// Note: The toneMaxVolume and noToneMaxVolume functions are defined in State.cpp
// We're declaring them here as extern to use them
extern void toneMaxVolume(uint8_t pin, unsigned int frequency);
extern void noToneMaxVolume(uint8_t pin);