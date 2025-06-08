/**
 * StateProcess.h - State-based background process management for Flight Controller
 * 
 * Implements FreeRTOS tasks that run based on the current system state.
 * Each state can have its own set of background processes.
 */

#ifndef STATE_PROCESS_H
#define STATE_PROCESS_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "../fc/State.h"
#include "../navc/Sensors.h"

// Task priorities
#define ARMED_PROCESS_PRIORITY 2
#define IDLE_PRIORITY 0

// Task stack sizes (in words, not bytes)
#define ARMED_PROCESS_STACK_SIZE 512

// Forward declarations for buzzer functions
void toneMaxVolume(uint8_t pin, unsigned int frequency);
void noToneMaxVolume(uint8_t pin);

// Buzzer constants from State.cpp
extern const int TONE_IDLE;
extern const int TONE_TEST;

class StateProcessManager {
public:
    StateProcessManager();
    ~StateProcessManager();
    
    // Initialize FreeRTOS and create initial tasks
    bool begin();
    
    // Update tasks based on current state
    void updateProcesses(SystemState newState);
    
    // Stop all background processes
    void stopAllProcesses();
    
private:
    // Task handles
    TaskHandle_t armedProcessHandle;
    
    // Mutex for thread safety
    SemaphoreHandle_t processMutex;
    
    // Current active state
    SystemState currentProcessState;
    
    // Armed state process variables
    static bool payloadDeploymentTriggered;
    static int32_t previousAltitude;
    static uint32_t previousTimestamp;
    static uint32_t lastBuzzerBeep;
    
    // Task functions (must be static for FreeRTOS)
    static void armedProcessTask(void* pvParameters);
    
    // Helper functions
    void startArmedProcesses();
    void stopArmedProcesses();
};

#endif // STATE_PROCESS_H