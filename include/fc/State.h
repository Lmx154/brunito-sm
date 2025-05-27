/**
 * State.h - Finite-State Machine declarations for Flight Controller
 * 
 * Defines the four states of the Brunito Flight Controller:
 * IDLE, TEST, ARMED, and RECOVERY, with their transitions
 * and allowed commands.
 */

#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

// System States
enum SystemState {
    STATE_IDLE,
    STATE_TEST,
    STATE_ARMED,
    STATE_RECOVERY
};

// Command Types
enum CommandType {
    CMD_DISARM,
    CMD_ARM,
    CMD_ENTER_TEST,
    CMD_ENTER_RECOVERY,
    CMD_TEST,
    CMD_QUERY,
    CMD_FIND_ME,    CMD_CONTROL,  // For servo, buzzer, etc.
    CMD_NAVC_RESET_STATS,  // For resetting NAVC packet stats
    CMD_TEST_DEVICE,  // For generic device testing (only in TEST state)
    CMD_TEST_SERVO,   // For servo testing (only in TEST state)
    CMD_TEST_ALTITUDE, // For altitude-based servo and buzzer test (only in TEST state) - DEPRECATED - one-time test
    CMD_ENABLE_ALTITUDE_TEST, // Enable background altitude test with threshold
    CMD_DISABLE_ALTITUDE_TEST // Disable background altitude test
};

class StateManager {
private:
    SystemState currentState;
    unsigned long armedTimestamp;
    unsigned long lastMotionTimestamp;
    bool inMotion;
    
    // Constants
    static const unsigned long AUTO_RECOVERY_TIMEOUT = 1800000; // 30 minutes
    static constexpr float NO_MOTION_THRESHOLD = 0.02; // g
    
    // Buzzer sound variables
    unsigned long buzzerStartTime;
    unsigned long buzzerDuration;
    bool buzzerActive;
    int currentBuzzerTone;
    SystemState pendingSoundState;
    int soundSequenceStep;
    
    // Altitude test variables
    bool altitudeTestEnabled;
    float altitudeTestThreshold;  // in meters
      // Helper method to check inactivity timeout
    bool shouldAutoRecovery();
    
    // Buzzer sound utility functions
    void startBuzzerSound(SystemState state);
    void updateBuzzerSound();
    void stopBuzzer();
    
    // Altitude test utility functions
    void checkBackgroundAltitudeTest();
    
public:
    StateManager();
      // State management
    SystemState getCurrentState() const;
    bool changeState(SystemState newState);
    bool processCommand(CommandType cmd, const char* cmdBuffer = nullptr);
    
    // State-specific behavior
    void updateState();
    void reportMotion(float accelMagnitude);
    
    // Command authorization check
    bool isCommandAllowed(CommandType cmd) const;
      // Convert state to string for debug
    const char* getStateString() const;
    
    // Helper to convert a state enum to string
    const char* getStateStringFromEnum(SystemState state) const;
};

#endif // STATE_H
