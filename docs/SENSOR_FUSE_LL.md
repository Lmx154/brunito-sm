## Suggestions for Improving Sensors.cpp

### 1. **Add Low-Pass Filtering for Noise Reduction**

Your current implementation reads raw sensor data. Add digital filters to reduce noise:

```cpp
// Add to SensorManager class
class LowPassFilter {
private:
    float alpha;
    float filteredValue;
    bool initialized;
    
public:
    LowPassFilter(float cutoffFreq, float sampleRate) : 
        initialized(false), filteredValue(0) {
        float RC = 1.0f / (2.0f * PI * cutoffFreq);
        float dt = 1.0f / sampleRate;
        alpha = dt / (RC + dt);
    }
    
    float update(float newValue) {
        if (!initialized) {
            filteredValue = newValue;
            initialized = true;
        } else {
            filteredValue = alpha * newValue + (1.0f - alpha) * filteredValue;
        }
        return filteredValue;
    }
};

// Add filter instances to SensorManager
LowPassFilter accelFilters[3];      // 20Hz cutoff for accelerometer
LowPassFilter gyroFilters[3];       // 50Hz cutoff for gyroscope
LowPassFilter altitudeFilter;       // 5Hz cutoff for barometer
```

### 2. **Implement Moving Average for Barometric Altitude**

Your altitude calculation could be much more stable:

```cpp
class MovingAverage {
private:
    float buffer[20];  // 20-sample buffer for 200ms averaging at 100Hz
    int index;
    int count;
    float sum;
    
public:
    MovingAverage() : index(0), count(0), sum(0) {
        memset(buffer, 0, sizeof(buffer));
    }
    
    float update(float newValue) {
        if (count < 20) {
            buffer[index] = newValue;
            sum += newValue;
            count++;
        } else {
            sum -= buffer[index];
            buffer[index] = newValue;
            sum += newValue;
        }
        
        index = (index + 1) % 20;
        return sum / count;
    }
};

// In readBarometer(), replace direct altitude assignment:
static MovingAverage altitudeAverage;
altitude = altitudeAverage.update(currentAltitude - referenceAltitude);
```

### 3. **Add Accelerometer Bias Removal and Calibration**

Accelerometers have bias that should be removed:

```cpp
class AccelerometerCalibrator {
private:
    float bias[3];           // Bias in each axis
    float scaleFactor[3];    // Scale factor for each axis
    bool calibrated;
    
public:
    AccelerometerCalibrator() : calibrated(false) {
        bias[0] = bias[1] = bias[2] = 0.0f;
        scaleFactor[0] = scaleFactor[1] = scaleFactor[2] = 1.0f;
    }
    
    void calibrate() {
        // Collect data while stationary on level surface
        // Calculate bias and scale factors
        // This should be run during ground initialization
    }
    
    void applyCorrection(float& ax, float& ay, float& az) {
        if (calibrated) {
            ax = (ax - bias[0]) * scaleFactor[0];
            ay = (ay - bias[1]) * scaleFactor[1];
            az = (az - bias[2]) * scaleFactor[2];
        }
    }
};
```

### 4. **Implement Gyroscope Drift Compensation**

Gyroscopes drift over time and need bias compensation:

```cpp
class GyroscopeBiasEstimator {
private:
    float biasEstimate[3];
    float integration[3];
    int stationaryCount;
    bool estimating;
    
public:
    GyroscopeBiasEstimator() {
        memset(biasEstimate, 0, sizeof(biasEstimate));
        memset(integration, 0, sizeof(integration));
        stationaryCount = 0;
        estimating = false;
    }
    
    void update(float gx, float gy, float gz, float totalAccel) {
        // Detect stationary condition (total acceleration near 1g)
        bool stationary = (fabs(totalAccel - 1.0f) < 0.1f);
        
        if (stationary) {
            stationaryCount++;
            integration[0] += gx;
            integration[1] += gy;
            integration[2] += gz;
            
            // After 5 seconds of stationary data, update bias estimate
            if (stationaryCount >= 500) {  // 500 samples at 100Hz = 5 seconds
                biasEstimate[0] = integration[0] / stationaryCount;
                biasEstimate[1] = integration[1] / stationaryCount;
                biasEstimate[2] = integration[2] / stationaryCount;
                
                // Reset for next estimation
                memset(integration, 0, sizeof(integration));
                stationaryCount = 0;
                estimating = true;
            }
        } else {
            // Reset if motion detected
            if (stationaryCount > 0) {
                memset(integration, 0, sizeof(integration));
                stationaryCount = 0;
            }
        }
    }
    
    void applyCorrection(float& gx, float& gy, float& gz) {
        if (estimating) {
            gx -= biasEstimate[0];
            gy -= biasEstimate[1];
            gz -= biasEstimate[2];
        }
    }
};
```

### 5. **Add Temperature Compensation**

IMU sensors are temperature sensitive:

```cpp
class TemperatureCompensator {
private:
    float tempCoeff[3][3];  // Temperature coefficients for each sensor axis
    float refTemp;          // Reference temperature
    
public:
    void compensateAccelerometer(float& ax, float& ay, float& az, float temp) {
        float tempDelta = temp - refTemp;
        ax -= tempCoeff[0][0] * tempDelta;
        ay -= tempCoeff[0][1] * tempDelta;
        az -= tempCoeff[0][2] * tempDelta;
    }
    
    void compensateGyroscope(float& gx, float& gy, float& gz, float temp) {
        float tempDelta = temp - refTemp;
        gx -= tempCoeff[1][0] * tempDelta;
        gy -= tempCoeff[1][1] * tempDelta;
        gz -= tempCoeff[1][2] * tempDelta;
    }
};
```

### 6. **Improve Magnetometer Handling**

Your current magnetometer code has good error handling, but could be enhanced:

```cpp
class MagnetometerProcessor {
private:
    float hardIronOffset[3];
    float softIronMatrix[3][3];
    LowPassFilter magFilters[3];
    float lastValidReading[3];
    int consecutiveFailures;
    
public:
    MagnetometerProcessor() : 
        magFilters{LowPassFilter(10.0f, 100.0f), 
                   LowPassFilter(10.0f, 100.0f), 
                   LowPassFilter(10.0f, 100.0f)},
        consecutiveFailures(0) {
        
        // Initialize identity matrix for soft iron
        memset(softIronMatrix, 0, sizeof(softIronMatrix));
        softIronMatrix[0][0] = softIronMatrix[1][1] = softIronMatrix[2][2] = 1.0f;
        memset(hardIronOffset, 0, sizeof(hardIronOffset));
        memset(lastValidReading, 0, sizeof(lastValidReading));
    }
    
    bool processReading(float& magX, float& magY, float& magZ) {
        // Apply calibration
        magX -= hardIronOffset[0];
        magY -= hardIronOffset[1];
        magZ -= hardIronOffset[2];
        
        // Check for reasonable values
        float magnitude = sqrt(magX*magX + magY*magY + magZ*magZ);
        bool valid = (magnitude > 20.0f && magnitude < 80.0f);  // Earth's field range
        
        if (valid) {
            // Apply filtering
            magX = magFilters[0].update(magX);
            magY = magFilters[1].update(magY);
            magZ = magFilters[2].update(magZ);
            
            // Store as last valid reading
            lastValidReading[0] = magX;
            lastValidReading[1] = magY;
            lastValidReading[2] = magZ;
            consecutiveFailures = 0;
            return true;
        } else {
            consecutiveFailures++;
            // Use last valid reading if failures are recent
            if (consecutiveFailures < 100) {  // 1 second at 100Hz
                magX = lastValidReading[0];
                magY = lastValidReading[1];
                magZ = lastValidReading[2];
                return true;
            }
            return false;
        }
    }
};
```

### 7. **Add Derived Measurements**

Calculate useful rocket-specific parameters:

```cpp
class RocketTelemetryProcessor {
public:
    struct RocketData {
        float totalAcceleration;     // G-force magnitude
        float verticalAcceleration;  // Vertical component
        float lateralAcceleration;   // Lateral component
        float angularVelocity;       // Total rotation rate
        float altitude;              // Filtered altitude
        float verticalVelocity;      // Calculated velocity
        float maxAltitude;           // Peak altitude reached
        float maxAcceleration;       // Peak acceleration
        bool motorBurnDetected;      // Motor firing detection
        bool apogeeDetected;         // Apogee detection
        uint32_t flightTime;         // Time since launch
    };
    
private:
    RocketData rocketData;
    float lastAltitude;
    uint32_t lastAltitudeTime;
    uint32_t launchTime;
    bool launched;
    
public:
    void update(const SensorPacket& packet) {
        // Calculate total acceleration in G's
        float ax = packet.accelX / 1000.0f;  // Convert mg to g
        float ay = packet.accelY / 1000.0f;
        float az = packet.accelZ / 1000.0f;
        
        rocketData.totalAcceleration = sqrt(ax*ax + ay*ay + az*az);
        
        // Detect motor burn (acceleration > 2G sustained)
        static int highAccelCount = 0;
        if (rocketData.totalAcceleration > 2.0f) {
            highAccelCount++;
            if (highAccelCount > 50 && !launched) {  // 0.5 seconds at 100Hz
                launched = true;
                launchTime = packet.timestamp;
                rocketData.motorBurnDetected = true;
            }
        } else {
            highAccelCount = 0;
        }
        
        // Calculate vertical velocity from altitude change
        float currentAltitude = packet.altitude / 100.0f;  // Convert cm to m
        if (lastAltitudeTime > 0) {
            float dt = (packet.timestamp - lastAltitudeTime) / 1000.0f;
            if (dt > 0.001f) {  // Avoid division by zero
                rocketData.verticalVelocity = (currentAltitude - lastAltitude) / dt;
            }
        }
        lastAltitude = currentAltitude;
        lastAltitudeTime = packet.timestamp;
        
        // Track maximums
        if (currentAltitude > rocketData.maxAltitude) {
            rocketData.maxAltitude = currentAltitude;
        }
        if (rocketData.totalAcceleration > rocketData.maxAcceleration) {
            rocketData.maxAcceleration = rocketData.totalAcceleration;
        }
        
        // Detect apogee (velocity changes from positive to negative)
        static float lastVelocity = 0;
        if (lastVelocity > 1.0f && rocketData.verticalVelocity < 0.5f && launched) {
            rocketData.apogeeDetected = true;
        }
        lastVelocity = rocketData.verticalVelocity;
        
        // Calculate flight time
        if (launched) {
            rocketData.flightTime = packet.timestamp - launchTime;
        }
        
        rocketData.altitude = currentAltitude;
    }
    
    const RocketData& getRocketData() const { return rocketData; }
};
```

### 8. **Enhanced Error Detection and Recovery**

Improve sensor fault detection:

```cpp
class SensorHealthMonitor {
private:
    struct SensorHealth {
        int consecutiveFailures;
        uint32_t lastSuccessTime;
        float lastValidValue;
        bool healthy;
    };
    
    SensorHealth accelHealth[3];
    SensorHealth gyroHealth[3];
    SensorHealth baroHealth;
    SensorHealth magHealth[3];
    
public:
    bool validateAccelerometer(float ax, float ay, float az) {
        bool valid = (!isnan(ax) && !isinf(ax) && fabs(ax) < 200.0f &&
                     !isnan(ay) && !isinf(ay) && fabs(ay) < 200.0f &&
                     !isnan(az) && !isinf(az) && fabs(az) < 200.0f);
        
        float totalAccel = sqrt(ax*ax + ay*ay + az*az);
        valid = valid && (totalAccel > 0.1f && totalAccel < 50.0f);
        
        updateSensorHealth(accelHealth[0], valid, ax);
        updateSensorHealth(accelHealth[1], valid, ay);
        updateSensorHealth(accelHealth[2], valid, az);
        
        return valid;
    }
    
    bool validateGyroscope(float gx, float gy, float gz) {
        bool valid = (!isnan(gx) && !isinf(gx) && fabs(gx) < 35.0f &&  // 2000 dps limit
                     !isnan(gy) && !isinf(gy) && fabs(gy) < 35.0f &&
                     !isnan(gz) && !isinf(gz) && fabs(gz) < 35.0f);
        
        updateSensorHealth(gyroHealth[0], valid, gx);
        updateSensorHealth(gyroHealth[1], valid, gy);
        updateSensorHealth(gyroHealth[2], valid, gz);
        
        return valid;
    }
    
private:
    void updateSensorHealth(SensorHealth& health, bool valid, float value) {
        if (valid) {
            health.consecutiveFailures = 0;
            health.lastSuccessTime = millis();
            health.lastValidValue = value;
            health.healthy = true;
        } else {
            health.consecutiveFailures++;
            health.healthy = (health.consecutiveFailures < 10);
        }
    }
};
```

### 9. **Recommended Integration Approach**

Here's how you could gradually integrate these improvements into your current Sensors.cpp:

1. **Phase 1**: Add basic filtering (low-pass filters, moving averages)
2. **Phase 2**: Implement bias compensation for accelerometer and gyroscope
3. **Phase 3**: Add derived measurements and flight phase detection
4. **Phase 4**: Implement full sensor fusion algorithms

### 10. **Performance Optimization**

- Use fixed-point arithmetic for time-critical calculations
- Pre-compute trigonometric values where possible
- Use lookup tables for common mathematical functions
- Implement circular buffers for efficiency

These improvements would significantly enhance the quality and usefulness of your telemetry data, providing better noise rejection, more accurate measurements, and valuable derived parameters for rocket flight analysis.