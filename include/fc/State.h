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

// External declarations for buzzer functionality
extern const int BUZZER_PIN;
extern const int TONE_IDLE;
extern const int TONE_TEST;
extern const int TONE_ARMED;
extern const int TONE_RECOVERY;
extern const int TONE_ERROR;
extern void toneMaxVolume(uint8_t pin, unsigned int frequency);
extern void noToneMaxVolume(uint8_t pin);

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
    CMD_CONTROL,  // For servo, buzzer, etc.
    CMD_NAVC_RESET_STATS,  // For resetting NAVC packet stats
    CMD_TEST_DEVICE,  // For generic device testing (only in TEST state)
    CMD_TEST_SERVO,   // For servo testing (only in TEST state)
    CMD_TEST_ALTITUDE, // For altitude-based servo and buzzer test (only in TEST state) - DEPRECATED - one-time test
    CMD_ENABLE_ALTITUDE_TEST, // Enable background altitude test with threshold
    CMD_DISABLE_ALTITUDE_TEST, // Disable background altitude test
    CMD_VELOCITY_TEST, // For velocity-based servo test with manual threshold (only in TEST state) - one-time test
    CMD_ENABLE_VELOCITY_TEST, // Enable background velocity test with threshold
    CMD_DISABLE_VELOCITY_TEST // Disable background velocity test
};

class StateManager {
private:
    SystemState currentState;
    unsigned long armedTimestamp;
    unsigned long idleTimestamp;  // Timestamp when we entered IDLE state
    unsigned long lastMotionTimestamp;
    bool inMotion;
      // Constants
    static const unsigned long AUTO_RECOVERY_TIMEOUT = 1800000; // 30 minutes
    static const unsigned long AUTO_ARM_TIMEOUT = 5000; // 5 seconds in IDLE before auto-arming
    static constexpr float NO_MOTION_THRESHOLD = 0.02; // g
    
    // Manual override variables - DEPRECATED: replaced by IdleLogic
    // bool lastManualOverrideState;
    // unsigned long lastManualOverrideCheck;
    // static const unsigned long MANUAL_OVERRIDE_DEBOUNCE = 50; // 50ms debounce
    
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
      // Velocity test variables (using integer arithmetic to avoid float precision issues)
    bool velocityTestEnabled;
    int32_t velocityTestThreshold;  // in cm/s (default 500 = 5.0 m/s)
    int32_t velocityTestMinAltitude; // minimum altitude before checking (in cm, 121920 = 4000ft)
    int32_t previousAltitude;       // for velocity calculation (in cm, same as packet.altitude)
    unsigned long previousTimestamp; // for time delta calculation
    bool velocityTestTriggered;   // prevent multiple triggers
    int32_t currentVelocity;        // current calculated velocity for external access (in cm/s)    // Helper method to check inactivity timeout
    bool shouldAutoRecovery();
    
    // Helper method to check auto-arming timeout
    bool shouldAutoArm();
    
    // Manual override utility function - DEPRECATED: replaced by IdleLogic
    // void checkManualOverride();
    
    // Buzzer sound utility functions
    void startBuzzerSound(SystemState state);
    void updateBuzzerSound();
    void stopBuzzer();
      // Altitude test utility functions
    void checkBackgroundAltitudeTest();
    
    // Velocity test utility functions
    void checkBackgroundVelocityTest();
    
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
      // Velocity test data access
    bool isVelocityTestEnabled() const;
    int32_t getCurrentVelocity() const;  // returns velocity in cm/s
    int32_t getVelocityTestThreshold() const;  // returns threshold in cm/s
    bool hasVelocityData() const;
      // Convert state to string for debug
    const char* getStateString() const;
    
    // Helper to convert a state enum to string
    const char* getStateStringFromEnum(SystemState state) const;
};

#endif // STATE_H
