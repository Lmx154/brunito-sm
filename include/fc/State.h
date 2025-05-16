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
    CMD_FIND_ME,
    CMD_CONTROL  // For servo, buzzer, etc.
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
    
    // Helper method to check inactivity timeout
    bool shouldAutoRecovery();
    
public:
    StateManager();
    
    // State management
    SystemState getCurrentState() const;
    bool changeState(SystemState newState);
    bool processCommand(CommandType cmd);
    
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
