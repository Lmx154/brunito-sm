# Brunito Sensor Fusion Implementation Guide

## Overview

This document provides a comprehensive guide for implementing sensor fusion algorithms using the Brunito project's current sensor suite. The system combines data from multiple sensors to create more accurate, reliable, and meaningful telemetry for rocket flight applications.

## Current Sensor Suite

### Available Sensors
| Sensor | Model | Measurement | Raw Units | Output Rate | Purpose |
|--------|-------|-------------|-----------|-------------|---------|
| **IMU Accelerometer** | BMI088 | Linear acceleration | m/s² | 100Hz | Motion detection, attitude |
| **IMU Gyroscope** | BMI088 | Angular velocity | rad/s | 100Hz | Rotation rates, attitude |
| **Magnetometer** | BMM150 | Magnetic field | µT | 100Hz | Heading, attitude reference |
| **Barometer** | BMP280 | Pressure/Altitude | Pa/m | 100Hz | Altitude, vertical velocity |
| **GPS** | MAX-M10S | Position/Velocity | degrees/m/s | 1-10Hz | Position, ground track |
| **RTC** | DS3231 | Time | timestamp | 1Hz | Data logging timestamps |

### Data Format Reference
From telemetry string: `<05/27/2025,11:43:46,0.95,-37,-967,-3,128,-27,204,6,-53,20,1,1,0,24>`

```cpp
// Raw sensor packet structure (current implementation)
struct SensorPacket {
    uint32_t timestamp;      // Milliseconds since boot
    int32_t altitude;        // Barometric altitude (cm)
    int16_t accelX, accelY, accelZ;  // Acceleration (mg)
    int16_t gyroX, gyroY, gyroZ;     // Angular rates (0.01°/s)
    int16_t magX, magY, magZ;        // Magnetic field (0.1µT)
    int32_t latitude, longitude;     // GPS coordinates (degrees×10⁷)
    uint8_t satellites;      // GPS satellite count
    int16_t temperature;     // Temperature (°C)
};
```

## Sensor Fusion Algorithms

### 1. Complementary Filter for Attitude Estimation

#### Purpose
Combine accelerometer and gyroscope data to estimate pitch and roll angles while filtering out noise and drift.

#### Algorithm Implementation
```cpp
class AttitudeFilter {
private:
    float pitch, roll, yaw;
    float alpha;  // Complementary filter coefficient (0.02-0.05)
    uint32_t lastUpdate;
    
public:
    AttitudeFilter(float filterAlpha = 0.02) : 
        pitch(0), roll(0), yaw(0), alpha(filterAlpha), lastUpdate(0) {}
    
    void update(float accelX, float accelY, float accelZ,
                float gyroX, float gyroY, float gyroZ, uint32_t timestamp) {
        
        if (lastUpdate == 0) {
            lastUpdate = timestamp;
            return;
        }
        
        float dt = (timestamp - lastUpdate) / 1000.0f;  // Convert to seconds
        lastUpdate = timestamp;
        
        // Convert units (from your current format)
        float ax = accelX / 1000.0f * 9.81f;  // mg to m/s²
        float ay = accelY / 1000.0f * 9.81f;
        float az = accelZ / 1000.0f * 9.81f;
        float gx = gyroX / 100.0f * PI / 180.0f;  // 0.01°/s to rad/s
        float gy = gyroY / 100.0f * PI / 180.0f;
        float gz = gyroZ / 100.0f * PI / 180.0f;
        
        // Calculate accelerometer angles (gravity reference)
        float accel_pitch = atan2(-ax, sqrt(ay*ay + az*az));
        float accel_roll = atan2(ay, az);
        
        // Integrate gyroscope rates
        pitch += gx * dt;
        roll += gy * dt;
        yaw += gz * dt;
        
        // Apply complementary filter
        pitch = alpha * accel_pitch + (1.0f - alpha) * pitch;
        roll = alpha * accel_roll + (1.0f - alpha) * roll;
        
        // Yaw drift compensation using magnetometer (if available)
        // This would require magnetometer calibration first
    }
    
    float getPitch() { return pitch * 180.0f / PI; }  // Return in degrees
    float getRoll() { return roll * 180.0f / PI; }
    float getYaw() { return yaw * 180.0f / PI; }
};
```

### 2. Kalman Filter for Altitude and Velocity

#### Purpose
Fuse barometric altitude with accelerometer data to estimate both altitude and vertical velocity with reduced noise.

#### Algorithm Implementation
```cpp
class AltitudeKalmanFilter {
private:
    // State vector: [altitude, velocity]
    float state[2];
    float P[2][2];  // Error covariance matrix
    float Q[2][2];  // Process noise covariance
    float R;        // Measurement noise variance
    
public:
    AltitudeKalmanFilter() {
        // Initialize state
        state[0] = 0.0f;  // altitude
        state[1] = 0.0f;  // velocity
        
        // Initialize covariance matrices
        P[0][0] = 1000.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f;    P[1][1] = 1000.0f;
        
        Q[0][0] = 0.1f; Q[0][1] = 0.0f;
        Q[1][0] = 0.0f; Q[1][1] = 0.5f;
        
        R = 1.0f;  // Barometer measurement noise
    }
    
    void predict(float accelZ, float dt) {
        // Convert acceleration units
        float az = accelZ / 1000.0f * 9.81f - 9.81f;  // mg to m/s², remove gravity
        
        // Prediction step
        float new_altitude = state[0] + state[1] * dt + 0.5f * az * dt * dt;
        float new_velocity = state[1] + az * dt;
        
        state[0] = new_altitude;
        state[1] = new_velocity;
        
        // Update prediction covariance
        P[0][0] += Q[0][0] + dt * dt * P[1][1] + 2 * dt * P[0][1];
        P[0][1] += dt * P[1][1] + Q[0][1];
        P[1][0] += dt * P[1][1] + Q[1][0];
        P[1][1] += Q[1][1];
    }
    
    void update(float baroAltitude) {
        // Convert barometric altitude from cm to m
        float measurement = baroAltitude / 100.0f;
        
        // Kalman gain
        float S = P[0][0] + R;
        float K[2] = {P[0][0] / S, P[1][0] / S};
        
        // Update step
        float innovation = measurement - state[0];
        state[0] += K[0] * innovation;
        state[1] += K[1] * innovation;
        
        // Update covariance
        float I_KH[2][2] = {{1.0f - K[0], 0.0f}, {-K[1], 1.0f}};
        float temp[2][2];
        
        // P = (I - K*H) * P
        temp[0][0] = I_KH[0][0] * P[0][0] + I_KH[0][1] * P[1][0];
        temp[0][1] = I_KH[0][0] * P[0][1] + I_KH[0][1] * P[1][1];
        temp[1][0] = I_KH[1][0] * P[0][0] + I_KH[1][1] * P[1][0];
        temp[1][1] = I_KH[1][0] * P[0][1] + I_KH[1][1] * P[1][1];
        
        P[0][0] = temp[0][0]; P[0][1] = temp[0][1];
        P[1][0] = temp[1][0]; P[1][1] = temp[1][1];
    }
    
    float getAltitude() { return state[0]; }
    float getVelocity() { return state[1]; }
    float getAltitudeUncertainty() { return sqrt(P[0][0]); }
    float getVelocityUncertainty() { return sqrt(P[1][1]); }
};
```

### 3. Motion Detection and Flight Phase Classification

#### Purpose
Detect rocket flight phases based on acceleration patterns and altitude changes.

#### Algorithm Implementation
```cpp
enum FlightPhase {
    GROUND_IDLE,
    MOTOR_BURN,
    COASTING,
    APOGEE,
    DROGUE_DESCENT,
    MAIN_DESCENT,
    LANDED
};

class FlightPhaseDetector {
private:
    FlightPhase currentPhase;
    float accelerationBuffer[10];
    float altitudeBuffer[20];
    int bufferIndex;
    float launchThreshold;      // Acceleration threshold for launch detection
    float apogeeThreshold;      // Velocity threshold for apogee detection
    uint32_t phaseStartTime;
    
public:
    FlightPhaseDetector() : 
        currentPhase(GROUND_IDLE), bufferIndex(0), 
        launchThreshold(2.0f), apogeeThreshold(0.5f), phaseStartTime(0) {
        
        memset(accelerationBuffer, 0, sizeof(accelerationBuffer));
        memset(altitudeBuffer, 0, sizeof(altitudeBuffer));
    }
    
    FlightPhase updatePhase(float totalAccel, float altitude, float velocity, uint32_t timestamp) {
        // Update circular buffers
        accelerationBuffer[bufferIndex % 10] = totalAccel;
        altitudeBuffer[bufferIndex % 20] = altitude;
        bufferIndex++;
        
        FlightPhase previousPhase = currentPhase;
        
        switch (currentPhase) {
            case GROUND_IDLE:
                // Detect launch: sustained acceleration > threshold
                if (bufferIndex >= 10) {
                    float avgAccel = 0;
                    for (int i = 0; i < 10; i++) {
                        avgAccel += accelerationBuffer[i];
                    }
                    avgAccel /= 10.0f;
                    
                    if (avgAccel > (1.0f + launchThreshold)) {  // Above 1g + threshold
                        currentPhase = MOTOR_BURN;
                        phaseStartTime = timestamp;
                    }
                }
                break;
                
            case MOTOR_BURN:
                // Detect motor burnout: acceleration drops back to ~1g
                if (totalAccel < 1.5f && (timestamp - phaseStartTime) > 1000) {
                    currentPhase = COASTING;
                    phaseStartTime = timestamp;
                }
                break;
                
            case COASTING:
                // Detect apogee: velocity approaches zero
                if (abs(velocity) < apogeeThreshold) {
                    currentPhase = APOGEE;
                    phaseStartTime = timestamp;
                }
                break;
                
            case APOGEE:
                // Transition to descent when clearly descending
                if (velocity < -1.0f) {  // Descending at >1 m/s
                    currentPhase = DROGUE_DESCENT;
                    phaseStartTime = timestamp;
                }
                break;
                
            case DROGUE_DESCENT:
                // Detect main chute deployment (sudden deceleration)
                if (velocity > -5.0f && velocity < -2.0f) {  // Slower descent
                    currentPhase = MAIN_DESCENT;
                    phaseStartTime = timestamp;
                }
                break;
                
            case MAIN_DESCENT:
                // Detect landing (low altitude, low velocity)
                if (altitude < 10.0f && abs(velocity) < 1.0f) {
                    currentPhase = LANDED;
                    phaseStartTime = timestamp;
                }
                break;
                
            case LANDED:
                // Stay landed
                break;
        }
        
        return currentPhase;
    }
    
    const char* getPhaseString() {
        switch (currentPhase) {
            case GROUND_IDLE: return "GROUND_IDLE";
            case MOTOR_BURN: return "MOTOR_BURN";
            case COASTING: return "COASTING";
            case APOGEE: return "APOGEE";
            case DROGUE_DESCENT: return "DROGUE_DESCENT";
            case MAIN_DESCENT: return "MAIN_DESCENT";
            case LANDED: return "LANDED";
            default: return "UNKNOWN";
        }
    }
    
    FlightPhase getCurrentPhase() { return currentPhase; }
    uint32_t getTimeInPhase(uint32_t currentTime) { return currentTime - phaseStartTime; }
};
```

### 4. GPS-Aided Navigation

#### Purpose
Combine GPS position with IMU data for enhanced position and velocity estimation.

#### Algorithm Implementation
```cpp
class GPSAidedNavigator {
private:
    struct Position {
        double latitude, longitude;
        float altitude;
        float velocityN, velocityE, velocityD;  // North, East, Down
        uint32_t lastUpdate;
        bool valid;
    } position;
    
    float earthRadius = 6371000.0f;  // Earth radius in meters
    
public:
    GPSAidedNavigator() {
        position.latitude = 0.0;
        position.longitude = 0.0;
        position.altitude = 0.0;
        position.velocityN = 0.0;
        position.velocityE = 0.0;
        position.velocityD = 0.0;
        position.lastUpdate = 0;
        position.valid = false;
    }
    
    void updateGPS(int32_t lat, int32_t lon, float alt, uint8_t satellites, uint32_t timestamp) {
        if (satellites >= 4 && lat != 0 && lon != 0) {
            double newLat = lat / 10000000.0;  // Convert from degrees×10⁷
            double newLon = lon / 10000000.0;
            
            if (position.valid && position.lastUpdate > 0) {
                float dt = (timestamp - position.lastUpdate) / 1000.0f;
                
                if (dt > 0.1f && dt < 10.0f) {  // Reasonable time delta
                    // Calculate velocity from position change
                    float deltaLat = (newLat - position.latitude) * PI / 180.0f;
                    float deltaLon = (newLon - position.longitude) * PI / 180.0f;
                    
                    position.velocityN = (deltaLat * earthRadius) / dt;
                    position.velocityE = (deltaLon * earthRadius * cos(newLat * PI / 180.0f)) / dt;
                    position.velocityD = (position.altitude - alt) / dt;
                }
            }
            
            position.latitude = newLat;
            position.longitude = newLon;
            position.altitude = alt;
            position.lastUpdate = timestamp;
            position.valid = true;
        } else {
            position.valid = false;
        }
    }
    
    float getGroundSpeed() {
        return sqrt(position.velocityN * position.velocityN + 
                   position.velocityE * position.velocityE);
    }
    
    float getBearing() {
        if (position.velocityN == 0 && position.velocityE == 0) return 0;
        float bearing = atan2(position.velocityE, position.velocityN) * 180.0f / PI;
        return bearing < 0 ? bearing + 360.0f : bearing;
    }
    
    float getLatitude() { return position.latitude; }
    float getLongitude() { return position.longitude; }
    float getVerticalVelocity() { return -position.velocityD; }  // Positive = up
    bool isValid() { return position.valid; }
};
```

## Integrated Sensor Fusion Class

### Complete Implementation
```cpp
class BrunitoSensorFusion {
private:
    AttitudeFilter attitudeFilter;
    AltitudeKalmanFilter altitudeFilter;
    FlightPhaseDetector phaseDetector;
    GPSAidedNavigator navigator;
    
    // Derived telemetry
    struct FusedData {
        // Attitude (degrees)
        float pitch, roll, yaw;
        
        // Position and motion
        float altitude, velocity;
        float groundSpeed, bearing;
        
        // Accelerations
        float totalAcceleration;
        float verticalAcceleration;
        
        // Flight status
        FlightPhase phase;
        uint32_t timeInPhase;
        
        // Data quality
        bool gpsValid;
        bool imuValid;
        float altitudeUncertainty;
    } fusedData;
    
public:
    void update(const SensorPacket& packet) {
        uint32_t timestamp = packet.timestamp;
        
        // Update attitude estimation
        attitudeFilter.update(packet.accelX, packet.accelY, packet.accelZ,
                             packet.gyroX, packet.gyroY, packet.gyroZ, timestamp);
        
        // Calculate total acceleration magnitude
        float ax = packet.accelX / 1000.0f;
        float ay = packet.accelY / 1000.0f;
        float az = packet.accelZ / 1000.0f;
        fusedData.totalAcceleration = sqrt(ax*ax + ay*ay + az*az);
        
        // Update altitude filter
        static uint32_t lastAltUpdate = 0;
        if (lastAltUpdate > 0) {
            float dt = (timestamp - lastAltUpdate) / 1000.0f;
            altitudeFilter.predict(packet.accelZ, dt);
        }
        altitudeFilter.update(packet.altitude);
        lastAltUpdate = timestamp;
        
        // Update GPS navigation
        navigator.updateGPS(packet.latitude, packet.longitude, 
                           packet.altitude / 100.0f, packet.satellites, timestamp);
        
        // Update flight phase
        fusedData.phase = phaseDetector.updatePhase(
            fusedData.totalAcceleration,
            altitudeFilter.getAltitude(),
            altitudeFilter.getVelocity(),
            timestamp
        );
        
        // Update fused data structure
        fusedData.pitch = attitudeFilter.getPitch();
        fusedData.roll = attitudeFilter.getRoll();
        fusedData.yaw = attitudeFilter.getYaw();
        
        fusedData.altitude = altitudeFilter.getAltitude();
        fusedData.velocity = altitudeFilter.getVelocity();
        fusedData.altitudeUncertainty = altitudeFilter.getAltitudeUncertainty();
        
        fusedData.groundSpeed = navigator.getGroundSpeed();
        fusedData.bearing = navigator.getBearing();
        
        fusedData.timeInPhase = phaseDetector.getTimeInPhase(timestamp);
        
        fusedData.gpsValid = navigator.isValid();
        fusedData.imuValid = (fusedData.totalAcceleration > 0.1f && 
                             fusedData.totalAcceleration < 50.0f);
        
        // Calculate vertical acceleration (remove gravity component)
        fusedData.verticalAcceleration = az - 1.0f;  // Assume Z-axis is up
    }
    
    // Getter methods for all fused data
    float getPitch() { return fusedData.pitch; }
    float getRoll() { return fusedData.roll; }
    float getYaw() { return fusedData.yaw; }
    float getAltitude() { return fusedData.altitude; }
    float getVelocity() { return fusedData.velocity; }
    float getGroundSpeed() { return fusedData.groundSpeed; }
    float getBearing() { return fusedData.bearing; }
    float getTotalAcceleration() { return fusedData.totalAcceleration; }
    FlightPhase getFlightPhase() { return fusedData.phase; }
    const char* getFlightPhaseString() { return phaseDetector.getPhaseString(); }
    bool isGPSValid() { return fusedData.gpsValid; }
    bool isIMUValid() { return fusedData.imuValid; }
    
    // Enhanced telemetry output
    void formatEnhancedTelemetry(char* buffer, size_t bufferSize, const SensorPacket& packet) {
        snprintf(buffer, bufferSize,
            "<ENHANCED:%02u/%02u/20%02u,%02u:%02u:%02u,"
            "ALT=%.2f,VEL=%.2f,PITCH=%.1f,ROLL=%.1f,YAW=%.1f,"
            "GSPD=%.1f,BRG=%.0f,ACCEL=%.2f,PHASE=%s,"
            "GPS=%s,IMU=%s,SAT=%u,TEMP=%d>",
            packet.month, packet.day, packet.year,
            packet.hour, packet.minute, packet.second,
            fusedData.altitude, fusedData.velocity,
            fusedData.pitch, fusedData.roll, fusedData.yaw,
            fusedData.groundSpeed, fusedData.bearing,
            fusedData.totalAcceleration, getFlightPhaseString(),
            fusedData.gpsValid ? "OK" : "NO", fusedData.imuValid ? "OK" : "NO",
            packet.satellites, packet.temperature);
    }
};
```

## Integration with Current System

### Modifying Sensors.cpp
Add the sensor fusion to your existing sensor processing:

```cpp
// In SensorManager class (add to private section)
BrunitoSensorFusion sensorFusion;

// In processSensorData() function (after currentPacket is populated)
void SensorManager::processSensorData() {
    // ... existing packet preparation code ...
    
    // Update sensor fusion with current packet
    sensorFusion.update(currentPacket);
    
    // Optionally output enhanced telemetry
    char enhancedBuffer[256];
    sensorFusion.formatEnhancedTelemetry(enhancedBuffer, sizeof(enhancedBuffer), currentPacket);
    
    #if DEBUG_SENSORS
    Serial.println(enhancedBuffer);
    #endif
    
    // ... rest of existing code ...
}
```

## Calibration Requirements

### Magnetometer Calibration
```cpp
class MagnetometerCalibrator {
private:
    float hardIronOffset[3];    // Offset correction
    float softIronMatrix[3][3]; // Scale/rotation correction
    bool calibrated;
    
public:
    void calibrate(float magX, float magY, float magZ) {
        // Implement hard/soft iron calibration
        // Collect data while rotating in all orientations
        // Calculate offset and scaling corrections
    }
    
    void applyCorrectionA(float& magX, float& magY, float& magZ) {
        if (!calibrated) return;
        
        // Apply hard iron correction
        magX -= hardIronOffset[0];
        magY -= hardIronOffset[1];
        magZ -= hardIronOffset[2];
        
        // Apply soft iron correction (simplified)
        float correctedX = softIronMatrix[0][0] * magX + softIronMatrix[0][1] * magY + softIronMatrix[0][2] * magZ;
        float correctedY = softIronMatrix[1][0] * magX + softIronMatrix[1][1] * magY + softIronMatrix[1][2] * magZ;
        float correctedZ = softIronMatrix[2][0] * magX + softIronMatrix[2][1] * magY + softIronMatrix[2][2] * magZ;
        
        magX = correctedX;
        magY = correctedY;
        magZ = correctedZ;
    }
};
```

## Performance Considerations

### Computational Efficiency
- **Update Rates**: Run attitude filter at 100Hz, altitude filter at 50Hz, GPS at 1-10Hz
- **Memory Usage**: ~200 bytes for filter states
- **CPU Usage**: ~5-10% of available cycles on STM32F4

### Real-Time Constraints
- Prioritize critical calculations (attitude, altitude)
- Use fixed-point arithmetic for time-critical sections
- Buffer GPS updates for processing during low-activity periods

---

*This sensor fusion implementation provides enhanced telemetry data for rocket flight analysis, state estimation, and autonomous flight control systems.*
