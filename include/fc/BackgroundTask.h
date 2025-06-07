/**
 * BackgroundTask.h - State-based background process manager for Flight Controller
 * 
 * This header defines the BackgroundTask system which handles state-dependent
 * background processes using FreeRTOS tasks. It replaces the manual loop-based
 * approach with a dedicated task that runs background processes based on the
 * current system state.
 */

#ifndef BACKGROUND_TASK_H
#define BACKGROUND_TASK_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "State.h"

// Forward declarations
class CmdParser;

// Pin definitions
static const int ARM_BUTTON_PIN = PA1; // Pin A1 for external button (same as IdleLogic)

class BackgroundTask {
private:
    // Task handle for the background process task
    static TaskHandle_t backgroundTaskHandle;
    
    // References to required components
    static StateManager* stateManager;
    static CmdParser* cmdParser;
    
    // Timing variables for ARMED state buzzer
    static unsigned long lastBeepTime;
    
    // Timing variables for IDLE state button check
    static bool lastButtonState;
    static unsigned long lastButtonCheck;
    
    // Task initialization status
    static bool initialized;
    
    // Constants
    static const unsigned long BUTTON_DEBOUNCE_MS = 50;
    static const unsigned long ARMED_BEEP_INTERVAL_MS = 2000;
    static const unsigned long TASK_DELAY_MS = 10; // 100Hz task rate
    
    // Internal task functions
    static void backgroundTaskFunction(void *pvParameters);
    static void processIdleState();
    static void processArmedState();
    static void processTestState();
    static void processRecoveryState();
    
public:
    // Initialize the background task system
    static bool begin(StateManager* stateMgr, CmdParser* cmdPsr);
    
    // Stop the background task system
    static void stop();
    
    // Check if the background task is running
    static bool isRunning();
    
    // Get task status information
    static void getTaskInfo(char* buffer, size_t bufferSize);
};

#endif // BACKGROUND_TASK_H
