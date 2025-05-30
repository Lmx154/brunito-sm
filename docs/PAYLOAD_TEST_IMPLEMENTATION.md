# Payload Test Implementation Plan

## Overview

The `payload_test` function is an intelligent apogee detection system that uses velocity and altitude rate-of-change calculations to detect when the rocket reaches apogee (peak altitude) and triggers the payload servo at the optimal moment.

This document provides a comprehensive implementation plan for replacing the current hardcoded altitude-based payload deployment with a physics-based velocity detection system.

## Current System vs. Proposed System

### Current Implementation
- **Method**: Simple altitude threshold triggering
- **Trigger**: When altitude exceeds a preset value (e.g., 2m, 10m)
- **Issues**: 
  - Fixed altitude may not correspond to apogee
  - No consideration of flight dynamics
  - May trigger during ascent rather than at optimal deployment point

### Proposed Implementation
- **Method**: Physics-based apogee detection
- **Trigger**: When velocity decreases to threshold (~5 m/s) with negative acceleration
- **Advantages**:
  - Triggers at actual apogee regardless of absolute altitude
  - Considers rocket dynamics and flight profile
  - More reliable for varying flight conditions

## Data Analysis and Logic Verification

### Available Data Sources
- **Altitude**: `packet.altitude` (in cm, converted to meters by dividing by 100)
- **Accelerometer**: `packet.accelX/Y/Z` (in mg - milli-g units)
- **Timestamp**: `packet.timestamp` (milliseconds since boot)
- **Sampling Rate**: ~10-50Hz from NAVC

### Physics Logic Verification ✅

The proposed detection criteria are **correct** for apogee identification:

1. **Velocity decreasing** (negative rate of change) ✅
   - Indicates the rocket is slowing down during ascent
2. **Velocity approaching target threshold** (e.g., 5 m/s) ✅
   - Catches the rocket near apogee before velocity reaches zero
3. **Altitude rate of change approaching 0** ✅
   - Confirms the rocket is at the peak of its trajectory

This approach is more robust than simple altitude thresholding because it:
- Works regardless of launch altitude or atmospheric conditions
- Detects the actual flight dynamics rather than arbitrary position
- Provides optimal timing for payload deployment

## Implementation Architecture

### A. Command System Extensions

#### New Commands
Add to `CommandType` enum in `State.h`:

```cpp
CMD_ENABLE_PAYLOAD_TEST,  // Enable payload test with velocity threshold
CMD_DISABLE_PAYLOAD_TEST  // Disable payload test
```

#### Command Interface
```
<CMD:ENABLE_PAYLOAD_TEST:velocity=5.0>
```
- **Parameter**: `velocity` (float, m/s, default 5.0)
- **Range**: 1.0 - 20.0 m/s (reasonable bounds for rocket apogee)
- **Usage**: Only allowed in TEST state

```
<CMD:DISABLE_PAYLOAD_TEST>
```
- **Usage**: Disables active payload test
- **Safety**: Auto-disables after successful trigger

### B. State Manager Extensions

#### New Private Members
Add to `StateManager` class in `State.h`:

```cpp
// Payload test variables
bool payloadTestEnabled;
float velocityThreshold;          // Target velocity in m/s (default 5.0)
float previousAltitude;           // For velocity calculation
unsigned long previousTimestamp;  // For time delta calculation
float currentVelocity;            // Current calculated velocity
float previousVelocity;           // For acceleration calculation
bool apogeeDetected;              // Prevent multiple triggers

// Filtering and safety
static const int REQUIRED_DETECTIONS = 3;  // Consecutive confirmations needed
int consecutiveDetections;                  // Counter for confirmation
unsigned long testStartTime;               // For timeout protection
static const unsigned long MAX_TEST_DURATION = 300000;  // 5 minutes max
```

#### New Public Methods
Add to `StateManager` class:

```cpp
// Payload test methods
void updatePayloadTest(const SensorPacket& packet);  // Main update function
bool isPayloadTestEnabled() const;                   // Status query
```

#### New Private Methods
```cpp
void checkPayloadTest();                    // Main payload test logic
void calculateVelocity(const SensorPacket& packet);  // Velocity calculation
bool detectApogeeConditions();             // Apogee detection logic
void triggerPayloadServo();                 // Servo actuation
float parseVelocityParameter(const char* params);    // Parameter parsing
```

## Detailed Algorithm Implementation

### 1. Velocity Calculation

```cpp
void StateManager::calculateVelocity(const SensorPacket& packet) {
    if (previousTimestamp == 0) {
        // First measurement - initialize
        previousAltitude = packet.altitude / 100.0f;
        previousTimestamp = packet.timestamp;
        currentVelocity = 0.0f;
        previousVelocity = 0.0f;
        return;
    }
    
    float currentAltitude = packet.altitude / 100.0f;
    unsigned long timeDelta = packet.timestamp - previousTimestamp;
    
    // Validate data
    if (timeDelta == 0 || timeDelta > 1000) {  // Sanity check for reasonable time delta
        return;  // Skip invalid data
    }
    
    // Calculate velocity: v = Δh / Δt
    previousVelocity = currentVelocity;
    currentVelocity = (currentAltitude - previousAltitude) / (timeDelta / 1000.0f);
    
    // Update for next calculation
    previousAltitude = currentAltitude;
    previousTimestamp = packet.timestamp;
}
```

### 2. Apogee Detection Logic

```cpp
bool StateManager::detectApogeeConditions() {
    // Condition 1: Velocity is positive but decreasing (ascending but slowing)
    bool velocityDecreasing = (currentVelocity < previousVelocity);
    
    // Condition 2: Velocity is at or below threshold (e.g., 5 m/s)
    bool velocityNearThreshold = (currentVelocity <= velocityThreshold && currentVelocity > 0);
    
    // Condition 3: Rate of velocity change is negative (deceleration)
    float acceleration = currentVelocity - previousVelocity;
    bool decelerating = (acceleration < -0.5f);  // Require significant deceleration
    
    // Additional safety: Ensure we're at reasonable altitude (prevent ground triggers)
    bool reasonableAltitude = (previousAltitude > 50.0f);  // Above 50m
    
    // Additional safety: Ensure we've been ascending (prevent false triggers)
    bool hasBeenAscending = (currentVelocity > 1.0f || previousVelocity > 1.0f);
    
    return velocityDecreasing && velocityNearThreshold && decelerating && 
           reasonableAltitude && hasBeenAscending;
}
```

### 3. Main Payload Test Function

```cpp
void StateManager::checkPayloadTest() {
    if (!payloadTestEnabled || apogeeDetected) {
        return;
    }
    
    // Timeout protection
    if (testStartTime > 0 && (millis() - testStartTime > MAX_TEST_DURATION)) {
        payloadTestEnabled = false;
        Serial.println("<DEBUG:PAYLOAD_TEST_TIMEOUT_DISABLED>");
        return;
    }
    
    // Check apogee conditions
    if (detectApogeeConditions()) {
        consecutiveDetections++;
        
        // Require multiple consecutive confirmations
        if (consecutiveDetections >= REQUIRED_DETECTIONS) {
            triggerPayloadServo();
        }
    } else {
        consecutiveDetections = 0;  // Reset on any negative detection
    }
    
    // Debug output every second
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
        char debugBuf[128];
        snprintf(debugBuf, sizeof(debugBuf),
                "<PAYLOAD_DEBUG:ALT=%.1fm,VEL=%.2fm/s,PREV_VEL=%.2fm/s,THRESH=%.1fm/s,DETECTIONS=%d>",
                previousAltitude, currentVelocity, previousVelocity, velocityThreshold, consecutiveDetections);
        Serial.println(debugBuf);
        lastDebugTime = millis();
    }
}
```

### 4. Servo Actuation

```cpp
void StateManager::triggerPayloadServo() {
    if (apogeeDetected) return;  // Prevent multiple triggers
    
    Serial.println("<DEBUG:PAYLOAD_APOGEE_DETECTED_TRIGGERING_SERVO>");
    
    // Attempt servo actuation with error handling
    Servo payloadServo;
    if (payloadServo.attach(SERVO_PIN)) {
        // Move to deployment position
        payloadServo.write(90);
        delay(1000);  // Hold position for 1 second
        
        // Return to neutral position
        payloadServo.write(0);
        delay(500);   // Brief pause
        
        payloadServo.detach();
        
        // Mark as completed
        apogeeDetected = true;
        payloadTestEnabled = false;  // Auto-disable after trigger
        
        Serial.println("<DEBUG:PAYLOAD_SERVO_TRIGGERED_SUCCESS>");
        
        // Optional: Sound buzzer for confirmation
        toneMaxVolume(BUZZER_PIN, TONE_TEST);
        delay(200);
        noToneMaxVolume(BUZZER_PIN);
        
    } else {
        Serial.println("<ERROR:PAYLOAD_SERVO_ATTACH_FAILED>");
    }
}
```

## Safety and Reliability Features

### 1. Multiple Confirmation Strategy
```cpp
// Require 3 consecutive positive detections to prevent false triggers
static const int REQUIRED_DETECTIONS = 3;
int consecutiveDetections = 0;

if (detectApogeeConditions()) {
    consecutiveDetections++;
    if (consecutiveDetections >= REQUIRED_DETECTIONS) {
        triggerPayloadServo();
    }
} else {
    consecutiveDetections = 0;  // Reset on any negative detection
}
```

### 2. Timeout Protection
```cpp
// Maximum test duration to prevent indefinite waiting
static const unsigned long MAX_TEST_DURATION = 300000;  // 5 minutes

if (millis() - testStartTime > MAX_TEST_DURATION) {
    payloadTestEnabled = false;
    Serial.println("<DEBUG:PAYLOAD_TEST_TIMEOUT_DISABLED>");
}
```

### 3. Data Validation
```cpp
// Validate sensor data before calculations
if (packet.timestamp <= previousTimestamp || 
    abs(packet.altitude) > 10000000 ||  // Sanity check altitude
    timeDelta > 1000) {                 // Reasonable time delta
    return;  // Skip invalid packet
}
```

### 4. Noise Filtering (Optional Enhancement)
```cpp
// Simple moving average filter for velocity
class MovingAverage {
    static const int WINDOW_SIZE = 5;
    float values[WINDOW_SIZE];
    int index;
    int count;
    
public:
    MovingAverage() : index(0), count(0) {}
    
    float update(float newValue) {
        values[index] = newValue;
        index = (index + 1) % WINDOW_SIZE;
        if (count < WINDOW_SIZE) count++;
        
        float sum = 0;
        for (int i = 0; i < count; i++) {
            sum += values[i];
        }
        return sum / count;
    }
};

static MovingAverage velocityFilter;
currentVelocity = velocityFilter.update(rawVelocity);
```

## Integration Points

### 1. Command Processing
Add to `StateManager::processCommand()` in `State.cpp`:

```cpp
case CMD_ENABLE_PAYLOAD_TEST:
    if (isCommandAllowed(cmd)) {
        float threshold = 5.0f;  // Default 5 m/s
        
        // Parse velocity parameter if provided
        const char* params = strchr(cmdBuffer + 5, ':');
        if (params && strstr(params, "velocity=")) {
            threshold = parseVelocityParameter(params);
            // Validate range
            if (threshold < 1.0f) threshold = 1.0f;
            if (threshold > 20.0f) threshold = 20.0f;
        }
        
        // Enable payload test
        payloadTestEnabled = true;
        velocityThreshold = threshold;
        apogeeDetected = false;
        consecutiveDetections = 0;
        previousTimestamp = 0;  // Reset for fresh calculations
        testStartTime = millis();
        
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "<DEBUG:PAYLOAD_TEST_ENABLED:VELOCITY_THRESHOLD=%.1fm/s>", threshold);
        Serial.println(buffer);
        return true;
    }
    break;

case CMD_DISABLE_PAYLOAD_TEST:
    if (isCommandAllowed(cmd)) {
        payloadTestEnabled = false;
        apogeeDetected = false;
        consecutiveDetections = 0;
        
        Serial.println("<DEBUG:PAYLOAD_TEST_DISABLED>");
        return true;
    }
    break;
```

### 2. State Update Loop
Add to `StateManager::updateState()`:

```cpp
void StateManager::updateState() {
    // ... existing code ...
    
    // Check payload test if enabled (only in TEST state)
    if (currentState == STATE_TEST) {
        checkPayloadTest();
    }
    
    // ... rest of existing code ...
}
```

### 3. Packet Processing Integration
Add to `FC.cpp` in the `uartTask()` function, `STATE_TEST` case:

```cpp
case STATE_TEST:
    // ... existing test telemetry code ...
    
    // Update payload test with new sensor data
    if (stateManager.isPayloadTestEnabled()) {
        stateManager.updatePayloadTest(packet);
    }
    
    // Check background altitude test if enabled
    stateManager.checkBackgroundAltitudeTest();
    break;
```

### 4. Command Authorization
Add to `StateManager::isCommandAllowed()`:

```cpp
case STATE_TEST:
    // Allow payload test commands only in TEST state
    return (cmd != CMD_ARM && cmd != CMD_ENTER_RECOVERY);

case STATE_ARMED:
    // Payload test commands not allowed in ARMED state
    return (cmd != CMD_TEST && cmd != CMD_TEST_DEVICE && cmd != CMD_TEST_SERVO && 
            cmd != CMD_TEST_ALTITUDE && cmd != CMD_ENABLE_ALTITUDE_TEST && 
            cmd != CMD_DISABLE_ALTITUDE_TEST && cmd != CMD_ENABLE_PAYLOAD_TEST && 
            cmd != CMD_DISABLE_PAYLOAD_TEST);
```

## Testing and Validation Strategy

### 1. Ground Testing
1. **Simulated Motion**: 
   - Move device vertically by hand to simulate ascent/descent
   - Verify velocity calculations are reasonable
   - Test different motion profiles

2. **Threshold Validation**: 
   - Test various velocity thresholds (1, 3, 5, 10 m/s)
   - Verify appropriate triggering behavior
   - Confirm no false triggers during ground handling

3. **Safety Testing**:
   - Test timeout functionality
   - Verify multiple confirmation requirement
   - Test data validation and error handling

### 2. Debug and Monitoring
Comprehensive debug output during testing:

```cpp
// Real-time monitoring output
<PAYLOAD_DEBUG:ALT=45.2m,VEL=8.3m/s,PREV_VEL=9.1m/s,THRESH=5.0m/s,DETECTIONS=0>
<PAYLOAD_DEBUG:ALT=52.1m,VEL=6.2m/s,PREV_VEL=8.3m/s,THRESH=5.0m/s,DETECTIONS=1>
<PAYLOAD_DEBUG:ALT=56.8m,VEL=4.7m/s,PREV_VEL=6.2m/s,THRESH=5.0m/s,DETECTIONS=2>
<PAYLOAD_DEBUG:ALT=59.2m,VEL=3.1m/s,PREV_VEL=4.7m/s,THRESH=5.0m/s,DETECTIONS=3>
<DEBUG:PAYLOAD_APOGEE_DETECTED_TRIGGERING_SERVO>
<DEBUG:PAYLOAD_SERVO_TRIGGERED_SUCCESS>
```

### 3. Performance Validation
- **Computational Efficiency**: Verify calculations complete within packet processing time
- **Memory Usage**: Monitor additional state variable impact
- **Real-time Response**: Ensure immediate servo response when triggered

## Error Handling and Edge Cases

### 1. Invalid Data Protection
```cpp
// Validate timestamps
if (packet.timestamp <= previousTimestamp) {
    return;  // Skip packets with invalid timestamps
}

// Validate altitude readings
if (abs(packet.altitude) > 10000000) {  // 100km sanity check
    return;  // Skip obviously invalid altitude
}

// Validate time deltas
if (timeDelta > 1000 || timeDelta == 0) {  // 1 second max gap
    return;  // Skip packets with unreasonable timing
}
```

### 2. Servo Failure Recovery
```cpp
void StateManager::triggerPayloadServo() {
    Servo payloadServo;
    
    // Attempt attachment with timeout
    unsigned long attachStart = millis();
    if (!payloadServo.attach(SERVO_PIN)) {
        Serial.println("<ERROR:PAYLOAD_SERVO_ATTACH_FAILED>");
        return;
    }
    
    // Verify servo response
    payloadServo.write(90);
    delay(100);
    if (payloadServo.read() != 90) {
        Serial.println("<ERROR:PAYLOAD_SERVO_POSITIONING_FAILED>");
        payloadServo.detach();
        return;
    }
    
    // Complete sequence...
}
```

### 3. System Recovery
```cpp
// Auto-recovery from stuck states
if (payloadTestEnabled && !apogeeDetected && 
    (millis() - testStartTime > MAX_TEST_DURATION)) {
    
    // Force disable with diagnostic
    payloadTestEnabled = false;
    Serial.println("<ERROR:PAYLOAD_TEST_TIMEOUT_FORCE_DISABLED>");
    
    // Optional: Trigger anyway as safety measure
    if (previousAltitude > 100.0f) {  // Only if at reasonable altitude
        Serial.println("<DEBUG:TIMEOUT_TRIGGER_SAFETY_DEPLOYMENT>");
        triggerPayloadServo();
    }
}
```

## Performance Considerations

### 1. Computational Requirements
- **Floating-point operations**: 4-6 per packet (division, subtraction, comparison)
- **Memory overhead**: ~32 bytes additional state variables
- **Execution time**: <1ms per packet at 50Hz update rate

### 2. Real-time Constraints
- **Update frequency**: Processes at incoming packet rate (10-50Hz)
- **Response time**: <100ms from detection to servo actuation
- **Non-blocking**: All operations complete within packet processing window

### 3. Memory Efficiency
```cpp
// Minimize memory footprint
struct PayloadTestState {
    bool enabled : 1;
    bool apogeeDetected : 1;
    uint8_t consecutiveDetections : 4;  // 0-15 range sufficient
    float velocityThreshold;            // 4 bytes
    float previousAltitude;             // 4 bytes  
    float currentVelocity;              // 4 bytes
    float previousVelocity;             // 4 bytes
    unsigned long previousTimestamp;    // 4 bytes
    unsigned long testStartTime;        // 4 bytes
}; // Total: ~26 bytes
```

## Conclusion

This payload test implementation provides:

1. **Physics-based detection** using actual flight dynamics rather than arbitrary altitude
2. **Robust safety mechanisms** including multiple confirmations, timeouts, and data validation
3. **Clean integration** with existing state machine and command architecture
4. **Comprehensive debugging** and monitoring capabilities
5. **Graceful error handling** for edge cases and hardware failures

The system correctly identifies apogee by detecting the transition point where upward velocity decreases to the specified threshold with negative acceleration, providing optimal timing for payload deployment in rocket flight applications.

## RTOS Task-Based Implementation Strategy

### Why Use RTOS Task Approach?

Based on analysis of the current system architecture, implementing the payload test as a dedicated RTOS task provides significant advantages:

#### Current System Analysis
- **NAVC**: Uses FreeRTOS with dedicated tasks (sensorTask @200Hz, sdTask @50Hz, commandTask @20Hz)
- **FC**: Uses polling loop architecture with tasks called every loop iteration
- **Main loop load**: UART processing, LoRa communication (every 5ms), command processing, telemetry streaming

#### Benefits of RTOS Task Implementation

1. **Guaranteed Real-Time Response**
   - Task runs at exact intervals (50Hz) regardless of main loop load
   - Higher priority than radio/telemetry tasks ensures immediate processing
   - No interference from LoRa queue processing or command handling

2. **Predictable Performance**
   - FreeRTOS scheduler ensures deterministic timing
   - Payload calculations never blocked by other operations
   - Critical servo trigger happens in <10ms consistently

3. **System Isolation**
   - Payload test doesn't affect radio communication timing
   - Main loop remains responsive for commands
   - Telemetry streaming maintains consistent rates

4. **Scalability for Future Features**
   - Foundation for migrating to full TDMA system (as documented)
   - Aligns with planned task architecture redesign
   - Supports multiple concurrent background tests

#### Computational Requirements Analysis
```cpp
// Per packet (10-50Hz processing):
- Velocity calculation: 4-6 floating-point operations
- Apogee detection: Multiple condition checks  
- Filtering (optional): Moving average calculations
- Safety validations: Data integrity checks
- Memory overhead: ~32 bytes additional state variables
- Execution time: <1ms per packet at 50Hz update rate
```

This computational load could interfere with:
- LoRa communication (time-critical every 5ms)
- Command processing (safety-critical real-time)
- Telemetry streaming (affects link quality)

## Phased Implementation Strategy

### Phase 1: Basic Polling Implementation (Foundation)
**Objective**: Establish basic payload test functionality within existing architecture
**Duration**: 2-3 days
**Risk**: Low

#### Implementation Steps
1. **Add command enums** to `State.h`
2. **Add state variables** to `StateManager` class
3. **Implement basic velocity calculation** and apogee detection
4. **Add command processing** for enable/disable
5. **Integrate with existing polling loop** in `uartTask()`

#### Testing Strategy
- Ground testing with manual device movement
- Verify velocity calculations are reasonable
- Test command interface and parameter parsing
- Validate basic apogee detection logic

#### Success Criteria
- Commands respond correctly
- Velocity calculations produce sensible values
- Basic apogee detection triggers at appropriate times
- No interference with existing system operation

#### Code Integration Points
```cpp
// FC.cpp - uartTask() function
case STATE_TEST:
    // ... existing test telemetry code ...
    
    // Update payload test with new sensor data
    if (stateManager.isPayloadTestEnabled()) {
        stateManager.updatePayloadTest(packet);
    }
    break;
```

---

### Phase 2: Enhanced Safety and Reliability (Hardening)
**Objective**: Add robust safety mechanisms and error handling
**Duration**: 1-2 days
**Risk**: Low

#### Implementation Steps
1. **Add multiple confirmation strategy** (3 consecutive detections)
2. **Implement timeout protection** (5-minute max duration)
3. **Add comprehensive data validation** (timestamp, altitude sanity checks)
4. **Implement servo failure recovery** with error reporting
5. **Add detailed debug output** and monitoring

#### Testing Strategy
- Test safety mechanisms with invalid data
- Verify timeout functionality
- Test servo failure scenarios
- Validate error recovery procedures

#### Success Criteria
- No false triggers from invalid data
- Timeout protection works correctly
- Servo failures are handled gracefully
- Debug output provides useful information

---

### Phase 3: RTOS Task Migration (Performance)
**Objective**: Migrate from polling to dedicated RTOS task for guaranteed real-time performance
**Duration**: 3-4 days
**Risk**: Medium

#### Implementation Steps
1. **Create dedicated payload test task** with high priority
2. **Implement mutex-protected sensor data access**
3. **Add inter-task communication** for command handling
4. **Migrate payload logic** from polling loop to task
5. **Add task health monitoring** and watchdog

#### Task Architecture
```cpp
// New task priorities (higher number = higher priority)
// Priority 4: payloadTestTask    (highest - safety critical)
// Priority 3: loraServiceTask    (radio communication)  
// Priority 2: uartTask          (sensor data processing)
// Priority 1: commandTask       (command processing)
// Priority 0: statusTask        (background status)

void payloadTestTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz task rate
    
    while (1) {
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if (payloadTestEnabled && currentState == STATE_TEST) {
            // Take mutex for sensor data access
            if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Fast sensor data processing
                calculateVelocity(latestSensorPacket);
                
                if (detectApogeeConditions()) {
                    consecutiveDetections++;
                    if (consecutiveDetections >= REQUIRED_DETECTIONS) {
                        // Critical section - trigger payload immediately
                        triggerPayloadServo();
                    }
                } else {
                    consecutiveDetections = 0;
                }
                
                xSemaphoreGive(sensorDataMutex);
            }
        }
    }
}
```

#### Testing Strategy
- Compare performance between polling and task implementations
- Verify deterministic timing with oscilloscope
- Test under high system load conditions
- Validate mutex synchronization

#### Success Criteria
- Task runs at exact 50Hz intervals
- No timing jitter or missed cycles
- Payload response time <100ms guaranteed
- No interference with other system operations

---

### Phase 4: Advanced Filtering and Optimization (Enhancement)
**Objective**: Add noise filtering and performance optimizations
**Duration**: 2-3 days
**Risk**: Low

#### Implementation Steps
1. **Implement moving average filter** for velocity smoothing
2. **Add adaptive threshold adjustment** based on flight profile
3. **Optimize memory usage** with packed structures
4. **Add advanced apogee prediction** algorithms
5. **Implement telemetry data logging** for post-flight analysis

#### Advanced Features
```cpp
// Moving average filter for noise reduction
class MovingAverage {
    static const int WINDOW_SIZE = 5;
    float values[WINDOW_SIZE];
    int index;
    int count;
    
public:
    MovingAverage() : index(0), count(0) {}
    
    float update(float newValue) {
        values[index] = newValue;
        index = (index + 1) % WINDOW_SIZE;
        if (count < WINDOW_SIZE) count++;
        
        float sum = 0;
        for (int i = 0; i < count; i++) {
            sum += values[i];
        }
        return sum / count;
    }
};

// Memory-optimized state structure
struct PayloadTestState {
    bool enabled : 1;
    bool apogeeDetected : 1;
    uint8_t consecutiveDetections : 4;  // 0-15 range sufficient
    float velocityThreshold;            // 4 bytes
    float previousAltitude;             // 4 bytes  
    float currentVelocity;              // 4 bytes
    float previousVelocity;             // 4 bytes
    unsigned long previousTimestamp;    // 4 bytes
    unsigned long testStartTime;        // 4 bytes
}; // Total: ~26 bytes
```

#### Testing Strategy
- Compare filtered vs unfiltered velocity calculations
- Test performance optimizations under load
- Validate advanced algorithms with flight simulation data
- Verify memory usage optimizations

#### Success Criteria
- Reduced false positives from sensor noise
- Improved apogee detection accuracy
- Optimized memory footprint
- Enhanced debugging capabilities

---

### Phase 5: Integration with Future TDMA System (Future-Proofing)
**Objective**: Prepare architecture for future TDMA implementation
**Duration**: 1-2 days
**Risk**: Low

#### Implementation Steps
1. **Align task priorities** with planned TDMA architecture
2. **Add queue-based communication** for commands
3. **Implement time-sliced operation** for TDMA compatibility
4. **Add bandwidth monitoring** and optimization
5. **Create migration path** to full TDMA system

#### TDMA Compatibility
```cpp
// Task structure compatible with future TDMA implementation
void payloadTestTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Wait for TDMA time slot allocation
        vTaskDelayUntil(&xLastWakeTime, tdmaCyclePeriod);
        
        // Process payload test within allocated time window
        if (payloadTestEnabled) {
            processPayloadTestCycle();
        }
        
        // Yield remaining time slot for other TDMA operations
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
```

#### Testing Strategy
- Test compatibility with planned TDMA timing
- Verify queue-based command processing
- Validate time-sliced operation
- Test bandwidth optimization features

#### Success Criteria
- Compatible with TDMA architecture
- Queue-based communication works correctly
- Time-sliced operation maintains performance
- Ready for future TDMA migration

## Implementation Timeline and Dependencies

### Phase Dependencies
1. **Phase 1** → **Phase 2**: Basic functionality before safety enhancements
2. **Phase 2** → **Phase 3**: Robust polling implementation before RTOS migration
3. **Phase 3** → **Phase 4**: RTOS task stable before optimizations
4. **Phase 4** → **Phase 5**: Core features complete before TDMA preparation

### Estimated Timeline
- **Phase 1**: 2-3 days (basic polling implementation)
- **Phase 2**: 1-2 days (safety and reliability)
- **Phase 3**: 3-4 days (RTOS task migration)
- **Phase 4**: 2-3 days (filtering and optimization)
- **Phase 5**: 1-2 days (TDMA preparation)

**Total**: 9-14 days

### Risk Mitigation
- **Phase 1 & 2**: Low risk, builds on existing patterns
- **Phase 3**: Medium risk, requires careful RTOS integration
- **Phase 4 & 5**: Low risk, incremental improvements

### Testing Strategy Per Phase
Each phase includes:
- **Unit testing** of new functionality
- **Integration testing** with existing system
- **Performance validation** under load
- **Safety verification** with edge cases
- **Regression testing** to ensure no functionality loss

## Conclusion

This phased approach provides a safe, incremental path to implementing the physics-based payload test system:

1. **Phase 1-2**: Establish robust foundation with existing architecture
2. **Phase 3**: Migrate to high-performance RTOS task for guaranteed real-time response
3. **Phase 4-5**: Add advanced features and future-proof the implementation

The RTOS task-based approach in Phase 3 provides the critical real-time guarantees needed for safety-critical apogee detection while maintaining system stability and preparing for future architectural improvements.

### Next Steps for Implementation

1. **Start with Phase 1**: Basic polling implementation for immediate functionality
2. **Validate each phase thoroughly** before proceeding to the next
3. **Maintain backward compatibility** throughout the migration
4. **Document lessons learned** from each phase for future reference
5. **Plan integration with TDMA system** based on Phase 5 preparation

This approach provides a significant improvement over altitude-based triggering by using actual flight physics to determine the optimal deployment moment, implemented with industry-standard real-time practices for safety-critical systems.
