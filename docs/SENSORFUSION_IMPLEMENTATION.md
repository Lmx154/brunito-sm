## Guide to Implementing Low-Pass Filters for Brunito Sensors

### Introduction

The Brunito project uses a suite of MEMS-based sensors (accelerometer, gyroscope, magnetometer, and barometer) that are prone to noise from mechanical vibrations, electrical interference, and inherent sensor characteristics. Low-pass filters attenuate high-frequency noise, providing cleaner, more accurate data for real-time processing on the microcontroller. This guide outlines a phased implementation, starting with a basic filter for one sensor and expanding to others, with testing and tuning steps to ensure effectiveness.

---

### Phase 1: Understanding the Need for Low-Pass Filters

**Why Use Low-Pass Filters?**
- **Noise Reduction:** Sensors like the BMI088 (accelerometer and gyroscope), BMM150 (magnetometer), and BMP280 (barometer) generate noise that can skew readings.
- **Signal Integrity:** Filtering ensures the data reflects actual physical phenomena, critical for navigation and control in Brunito.
- **Real-Time Accuracy:** Preprocessing on the microcontroller improves data quality before transmission to the desktop application.

**Goal:** Implement lightweight filtering on the STM32F401CCU6 to handle real-time needs, leaving advanced processing (e.g., Kalman filters) for the desktop.

---

### Phase 2: Selecting an Appropriate Low-Pass Filter

**Filter Options:**
- **Simple Moving Average (SMA):** Averages multiple samples; simple but less responsive to rapid changes.
- **Exponential Moving Average (EMA):** Weights recent data more heavily, balancing responsiveness and noise reduction.
- **Butterworth Filter:** Offers a flat passband but is computationally complex.

**Recommendation:** Use an EMA (first-order low-pass filter) due to:
- **Efficiency:** Low computational overhead for the STM32F401CCU6.
- **Real-Time Suitability:** Quick response with effective noise reduction.
- **Simplicity:** Easy to implement and tune.

**EMA Formula:**
```
y[n] = α * x[n] + (1 - α) * y[n-1]
```
- `y[n]`: Current filtered value
- `x[n]`: Current raw sensor value
- `y[n-1]`: Previous filtered value
- `α`: Smoothing factor (0 < α < 1), where smaller α increases smoothing.

---

### Phase 3: Implementing a Basic EMA Filter for the Accelerometer

**Target Sensor:** BMI088 Accelerometer

**Steps:**

1. **Define the Filter Class:**
   Add this to `Sensors.h` under the `SensorManager` class’s private section:
   ```cpp
   class LowPassFilter {
   private:
       float alpha;
       float filteredValue;
       bool initialized;
   public:
       LowPassFilter(float smoothingFactor) : 
           alpha(smoothingFactor), filteredValue(0), initialized(false) {}
       
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
   ```

2. **Add Filter Instances:**
   In `Sensors.h`, under `SensorManager`’s private section, add:
   ```cpp
   LowPassFilter accelFilterX{0.2f}; // α = 0.2
   LowPassFilter accelFilterY{0.2f};
   LowPassFilter accelFilterZ{0.2f};
   ```

3. **Apply the Filter:**
   Modify `readAccelGyro()` in `Sensors.cpp`:
   ```cpp
   void SensorManager::readAccelGyro() {
       accel->readSensor();
       float rawX = accel->getAccelX_mss();
       float rawY = accel->getAccelY_mss();
       float rawZ = accel->getAccelZ_mss();
       
       accelData[0] = accelFilterX.update(rawX);
       accelData[1] = accelFilterY.update(rawY);
       accelData[2] = accelFilterZ.update(rawZ);
   }
   ```

4. **Choose α:**
   - Start with `α = 0.2` (moderate smoothing).
   - Smaller α (e.g., 0.1) increases noise reduction but slows response.
   - Larger α (e.g., 0.5) preserves responsiveness but reduces filtering.

---

### Phase 4: Integrating the Filter into SensorManager

**Implementation:**
- The filter is applied in `readAccelGyro()`, which is called by `update()`.
- Filtered values in `accelData[]` are used in `processSensorData()` to populate `currentPacket`.

**Code Location:**
- Modifications are in `src/navc/Sensors.cpp` and `include/navc/Sensors.h`.
- No changes to the packet structure (`SensorPacket`) are needed, as filtering occurs before scaling.

**Initialization Note:**
- The filter self-initializes with the first raw value, avoiding startup artifacts.

---

### Phase 5: Testing and Tuning the Filter

**Testing Steps:**

1. **Static Test:**
   - Keep the device stationary.
   - Compare raw (unfiltered) and filtered accelerometer values via debug output.
   - Expected Result: Filtered values show less variation (e.g., ±0.01g vs. ±0.1g raw).

   Modify `processSensorData()` temporarily for debugging:
   ```cpp
   #if DEBUG_SENSORS
   char buffer[128];
   snprintf(buffer, sizeof(buffer), "<DEBUG:ACCEL_RAW:%.2f,%.2f,%.2f,FILTERED:%.2f,%.2f,%.2f>",
            accel->getAccelX_mss(), accel->getAccelY_mss(), accel->getAccelZ_mss(),
            accelData[0], accelData[1], accelData[2]);
   Serial.println(buffer);
   #endif
   ```

2. **Dynamic Test:**
   - Perform controlled movements (e.g., slow tilt).
   - Check if filtered values track the motion without excessive delay.
   - Expected Result: Smooth tracking with minimal lag.

3. **Tune α:**
   - If noise persists, decrease α (e.g., to 0.1).
   - If response is too slow, increase α (e.g., to 0.3).
   - Adjust in `Sensors.h` filter declarations.

---

### Phase 6: Expanding to Other Sensors

**Apply EMA to Additional Sensors:**

1. **Gyroscope (BMI088):**
   - Add to `Sensors.h`:
     ```cpp
     LowPassFilter gyroFilterX{0.2f};
     LowPassFilter gyroFilterY{0.2f};
     LowPassFilter gyroFilterZ{0.2f};
     ```
   - Update `readAccelGyro()`:
     ```cpp
     gyro->readSensor();
     float rawGX = gyro->getGyroX_rads();
     float rawGY = gyro->getGyroY_rads();
     float rawGZ = gyro->getGyroZ_rads();
     
     gyroData[0] = gyroFilterX.update(rawGX);
     gyroData[1] = gyroFilterY.update(rawGY);
     gyroData[2] = gyroFilterZ.update(rawGZ);
     ```
   - Tune α based on noise (e.g., 0.1–0.3).

2. **Magnetometer (BMM150):**
   - Add to `Sensors.h`:
     ```cpp
     LowPassFilter magFilterX{0.1f}; // More smoothing due to EMI noise
     LowPassFilter magFilterY{0.1f};
     LowPassFilter magFilterZ{0.1f};
     ```
   - Update `readMagnetometer()` after validation:
     ```cpp
     if (validReading) {
         magData[0] = magFilterX.update(mag.mag_data.x);
         magData[1] = magFilterY.update(mag.mag_data.y);
         magData[2] = magFilterZ.update(mag.mag_data.z);
         
         SensorManager_lastValidX = magData[0];
         SensorManager_lastValidY = magData[1];
         SensorManager_lastValidZ = magData[2];
         SensorManager_hasValidReadingEver = true;
     }
     ```
   - Use smaller α (e.g., 0.1) due to electromagnetic interference sensitivity.

3. **Barometer (BMP280):**
   - Add to `Sensors.h`:
     ```cpp
     LowPassFilter altitudeFilter{0.05f}; // Heavy smoothing for stable altitude
     ```
   - Update `readBarometer()`:
     ```cpp
     if (!isnan(tempReading) && !isnan(pressureReading) && !isnan(currentAltitude)) {
         temperature = tempReading;
         pressure = pressureReading;
         float relativeAltitude = currentAltitude - referenceAltitude;
         altitude = altitudeFilter.update(relativeAltitude);
     }
     ```
   - Use small α (e.g., 0.05) for stable altitude readings.

**Considerations:**
- Adjust α per sensor based on noise characteristics and required responsiveness.
- Test each sensor as in Phase 5.

---

### Phase 7: Desktop Application Integration

**Pre-Filtered Data Handling:**
- The NAVC sends filtered data via UART to the Flight Controller (FC), then via LoRa to the Ground Station (GS), and finally to the desktop via USB.
- In your desktop application (e.g., the Python parser in `TELEMETRY_PROTOCOL.md`), parse the filtered values directly:
  ```python
  accel_x_mps2 = int(fields[3]) * 0.001 * 9.81  # Already filtered mg to m/s²
  ```

**Desktop Role:**
- Use pre-filtered data for visualization or basic analysis.
- Reserve advanced techniques (e.g., Kalman filtering, trend detection) for the desktop, leveraging its computational power.

---

### Conclusion

This phased approach implements EMA low-pass filters on the STM32F401CCU6 for the Brunito project’s sensors, enhancing real-time accuracy without excessive microcontroller load. Start with the accelerometer, test and tune, then expand to the gyroscope, magnetometer, and barometer. The desktop application can then process this cleaner data for higher-level tasks, ensuring an efficient division of labor between hardware and software.
