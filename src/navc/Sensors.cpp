#include "../include/navc/Sensors.h"
#include "../include/utils/FrameCodec.h"

// Define dedicated hardware Serial for GPS with fixed pins
// The pins must be specified in the constructor, not later with setRx/setTx
HardwareSerial SerialGPS(PB7, PB6);  // RX, TX pins for UBLOX MAX-M10S GPS

// Global variables to cache valid magnetometer readings
// These are shared between readMagnetometer() and processSensorData()
bool SensorManager_hasValidReadingEver = false;
int16_t SensorManager_lastValidX = 0;
int16_t SensorManager_lastValidY = 0;
int16_t SensorManager_lastValidZ = 0;

SensorManager::SensorManager() : 
    i2c(&Wire),
    accel(nullptr),
    gyro(nullptr),
    statusLed(1, PC14, NEO_GRB + NEO_KHZ800),
    lastSampleTime(0),
    lastFusionTime(0),
    lastStreamTime(0),
    ledOnTime(0),
    packetCounter(0),
    rtcYear(0),
    rtcMonth(1),
    rtcDay(1),
    rtcHour(0),
    rtcMinute(0),
    rtcSecond(0)
{
    // Initialize arrays to zeros
    memset(accelData, 0, sizeof(accelData));
    memset(gyroData, 0, sizeof(gyroData));
    memset(magData, 0, sizeof(magData)); // Ensure magData is initially zeroed
    memset(&currentPacket, 0, sizeof(currentPacket));
    
    temperature = 0.0f;
    pressure = 0.0f;
    altitude = 0.0f;
}

SensorManager::~SensorManager() {
    // Clean up dynamically allocated sensors
    if (accel) {
        delete accel;
        accel = nullptr;
    }
    
    if (gyro) {
        delete gyro;
        gyro = nullptr;
    }
}

bool SensorManager::begin() {    // Initialize I2C bus with explicit pin configuration
    Wire.setSCL(PB8); // Set SCL pin to PB8
    Wire.setSDA(PB9); // Set SDA pin to PB9
    Wire.begin();
    Wire.setClock(100000); // 100 kHz for better stability with BMM150
    
    // Initialize SerialGPS for GPS communication
    // Basic initialization - TinyGPS++ handles NMEA parsing
    SerialGPS.begin(9600); // UBLOX MAX-M10S default rate
    
    // Clear any existing data in the GPS buffer
    while (SerialGPS.available()) {
        SerialGPS.read();
    }
      delay(100); // Brief stabilization delay
    
    // Initialize RGB LED
    statusLed.begin();
    statusLed.clear();
    statusLed.show();
      // Initialize DS3231 RTC
    if (!rtc.begin()) {
        return false;
    }
    
    // Initialize BMP280 barometer
    if (!baro.begin()) {
        return false;
    }
    
    // Configure BMP280 settings (suggest by datasheet for flight)
    baro.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode
                    Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                    Adafruit_BMP280::FILTER_X4,       // Filtering
                    Adafruit_BMP280::STANDBY_MS_1);   // Standby time
    
    // Initialize BMI088 accelerometer
    accel = new Bmi088Accel(Wire, 0x19);
    if (accel->begin() < 0) {
        return false;
    }
    accel->setRange(Bmi088Accel::RANGE_6G);
    accel->setOdr(Bmi088Accel::ODR_800HZ_BW_80HZ);
    
    // Initialize BMI088 gyroscope
    gyro = new Bmi088Gyro(Wire, 0x69);
    if (gyro->begin() < 0) {
        return false;
    }
    gyro->setRange(Bmi088Gyro::RANGE_500DPS);
    gyro->setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
    
    // Initialize BMM150 magnetometer
    if (mag.initialize() != BMM150_OK) {
        // Try once more with I2C reset
        Wire.endTransmission(true);
        delay(100);
        if (mag.initialize() != BMM150_OK) {
            return false;
        }
    }
    
    // Test read to verify magnetometer functionality
    mag.read_mag_data();
    float testX = mag.raw_mag_data.raw_datax;
    float testY = mag.raw_mag_data.raw_datay;
    float testZ = mag.raw_mag_data.raw_dataz;
      #if DEBUG_SENSORS
    Serial.print("<DEBUG:BMM150_RAW_TEST_DATA:X=");
    Serial.print(testX);
    Serial.print(",Y=");
    Serial.print(testY);
    Serial.print(",Z=");
    Serial.print(testZ);
    Serial.println(">");
    
    // Check for common error patterns
    if (testZ == -32768) {
        Serial.println("<DEBUG:MAG_Z_AXIS_STUCK_DETECTED:CALIBRATION_REQUIRED>");
    } else if (testX == 0 && testY == 0 && testZ == 0) {
        Serial.println("<DEBUG:MAG_ALL_ZEROS_DETECTED:CALIBRATION_REQUIRED>");
    } else {
        Serial.println("<DEBUG:MAG_TEST_VALUES_APPEAR_NORMAL>");
    }
    #endif
    
    // If RTC lost power, set to compile time
    if (rtc.lostPower()) {
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:RTC_SET_TO_COMPILE_TIME>");
        #endif
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    // Set initial LED color to indicate successful init (green)
    setStatusLED(0, 255, 0);
      #if DEBUG_SENSORS
    Serial.println("<DEBUG:ALL_SENSORS_INIT_OK>");
    #endif
    
    return true;
}

int SensorManager::beginWithDiagnostics() {
    // Initialize basic sensor components and use Test module for comprehensive diagnostics
    if (!begin()) {
        return -1; // Basic initialization failed
    }
    
    // Defer to Test module for comprehensive validation
    // This provides separation of concerns and cleaner code organization
    return 0; // Basic initialization succeeded, tests should be run separately
}

void SensorManager::update() {
    unsigned long currentTime = millis();
    
    // Process any available GPS data (non-blocking)
    processGPS();
    
    // Sample sensors at SENSOR_SAMPLE_RATE_HZ (100 Hz)
    if (currentTime - lastSampleTime >= (1000 / SENSOR_SAMPLE_RATE_HZ)) {
        lastSampleTime = currentTime;
        
        // Read all sensor values
        readAccelGyro();
        readMagnetometer();
        readBarometer();
        updateTime();
    }
    
    // Process sensor data at SENSOR_FUSION_RATE_HZ (100 Hz)
    if (currentTime - lastFusionTime >= (1000 / SENSOR_FUSION_RATE_HZ)) {
        lastFusionTime = currentTime;
        processSensorData();
    }
    
    // Turn off LED after 5 seconds if it was turned on
    if (ledOnTime > 0 && (currentTime - ledOnTime >= 5000)) {
        setStatusLED(0, 0, 0); // Turn off the LED
        ledOnTime = 0; // Reset the timer
    }
}

bool SensorManager::updateWithDiagnostics() {
    unsigned long currentTime = millis();
    bool success = true;
    
    // Process any available GPS data (non-blocking)
    processGPS();
    
    // Sample sensors at SENSOR_SAMPLE_RATE_HZ (100 Hz)
    if (currentTime - lastSampleTime >= (1000 / SENSOR_SAMPLE_RATE_HZ)) {
        lastSampleTime = currentTime;
        
        // Read accelerometer data with basic retry
        if (accel != nullptr) {
            bool accelSuccess = false;
            
            for (int attempt = 0; attempt < 3 && !accelSuccess; attempt++) {
                if (attempt > 0) delay(2);
                
                accel->readSensor();
                float x = accel->getAccelX_mss();
                float y = accel->getAccelY_mss();
                float z = accel->getAccelZ_mss();
                
                if (!isnan(x) && !isinf(x) && !isnan(y) && !isinf(y) && !isnan(z) && !isinf(z)) {
                    accelData[0] = x;
                    accelData[1] = y;
                    accelData[2] = z;
                    accelSuccess = true;
                }
            }
            
            if (!accelSuccess) success = false;
        } else {
            success = false;
        }
        
        // Read gyroscope data with basic retry
        if (gyro != nullptr) {
            bool gyroSuccess = false;
            
            for (int attempt = 0; attempt < 3 && !gyroSuccess; attempt++) {
                if (attempt > 0) {
                    delay(2);
                    Wire.endTransmission(true);
                    delay(1);
                }
                
                gyro->readSensor();
                float x = gyro->getGyroX_rads();
                float y = gyro->getGyroY_rads();
                float z = gyro->getGyroZ_rads();
                
                if (!isnan(x) && !isinf(x) && !isnan(y) && !isinf(y) && !isnan(z) && !isinf(z)) {
                    gyroData[0] = x;
                    gyroData[1] = y;
                    gyroData[2] = z;
                    gyroSuccess = true;
                }
            }
            
            if (!gyroSuccess) success = false;
        } else {
            success = false;
        }
        
        // Read magnetometer data with basic retry
        bool magSuccess = false;
        for (int attempt = 0; attempt < 3 && !magSuccess; attempt++) {
            if (attempt > 0) delay(5);
            
            mag.read_mag_data();
              if (mag.mag_data.x != 0 || mag.mag_data.y != 0 || mag.mag_data.z != 0) {
                // Apply coordinate frame alignment: Flip X-axis to match BMI088 coordinate frame
                magData[0] = -mag.mag_data.x;  // Flip X-axis for frame alignment
                magData[1] = mag.mag_data.y;   // Y-axis already aligned
                magData[2] = mag.mag_data.z;   // Z-axis already aligned
                magSuccess = true;
            } else if (attempt == 1) {
                // Try reset on second attempt
                if (mag.initialize() == BMM150_OK) {
                    mag.set_op_mode(BMM150_NORMAL_MODE);
                    mag.set_presetmode(BMM150_PRESETMODE_HIGHACCURACY);
                }
            }
        }
        
        if (!magSuccess) {
            success = false;
            
            // Use last valid values if available
            if (SensorManager_hasValidReadingEver) {
                magData[0] = SensorManager_lastValidX;
                magData[1] = SensorManager_lastValidY;
                magData[2] = SensorManager_lastValidZ;
            } else {
                magData[0] = 0;
                magData[1] = 0;
                magData[2] = 1; // Small non-zero value to avoid division by zero
            }
        }
        
        // Read barometer and update time
        readBarometer();
        updateTime();
    }
    
    // Process sensor data at SENSOR_FUSION_RATE_HZ (100 Hz)
    if (currentTime - lastFusionTime >= (1000 / SENSOR_FUSION_RATE_HZ)) {
        lastFusionTime = currentTime;
        processSensorData();
    }
    
    // Turn off LED after 5 seconds if it was turned on
    if (ledOnTime > 0 && (currentTime - ledOnTime >= 5000)) {
        setStatusLED(0, 0, 0);
        ledOnTime = 0;
    }
    
    return success;
}

void SensorManager::processGPS() {
    // Simple GPS data processing - TinyGPS++ handles NMEA parsing
    // Just read available data and let the library do the work
    while (SerialGPS.available() > 0) {
        char c = SerialGPS.read();
        gps.encode(c);
    }
}

void SensorManager::readAccelGyro() {
    // Read accelerometer data in m/s^2 and apply low-pass filtering
    accel->readSensor();
    float rawX = accel->getAccelX_mss();
    float rawY = accel->getAccelY_mss();
    float rawZ = accel->getAccelZ_mss();
    
    // Apply EMA low-pass filters to reduce noise
    accelData[0] = accelFilterX.update(rawX);
    accelData[1] = accelFilterY.update(rawY);
    accelData[2] = accelFilterZ.update(rawZ);
    
    #if DEBUG_SENSORS
    // Debug output to compare raw vs filtered values (only every 100 samples to reduce spam)
    static int debugCounter = 0;
    if (++debugCounter >= 100) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "<DEBUG:ACCEL_RAW:%.2f,%.2f,%.2f,FILTERED:%.2f,%.2f,%.2f>",
                rawX, rawY, rawZ, accelData[0], accelData[1], accelData[2]);
        Serial.println(buffer);
        debugCounter = 0;
    }
    #endif
      // Read gyroscope data in rad/s and apply low-pass filtering
    gyro->readSensor();
    float rawGX = gyro->getGyroX_rads();
    float rawGY = gyro->getGyroY_rads();
    float rawGZ = gyro->getGyroZ_rads();
    
    // Apply EMA low-pass filters to reduce noise
    gyroData[0] = gyroFilterX.update(rawGX);
    gyroData[1] = gyroFilterY.update(rawGY);
    gyroData[2] = gyroFilterZ.update(rawGZ);
}

void SensorManager::readMagnetometer() {
    // Magnetometer reading with coordinate frame alignment to match BMI088
    // BMM150 X-axis is flipped compared to BMI088, Y+ and Z+ already match
    static int8_t consecutiveFailCount = 0;
    static unsigned long lastResetTime = 0;
    
    // Read magnetometer data
    mag.read_mag_data();
    
    // Check for basic error conditions
    bool validReading = true;
    
    // Check for all zeros or stuck Z-axis (common BMM150 issues)
    if ((mag.mag_data.x == 0 && mag.mag_data.y == 0 && mag.mag_data.z == 0) ||
        mag.mag_data.z == -32768) {
        validReading = false;
    }
    
    // Check for unreasonably large values
    const int16_t MAG_MAX_REASONABLE = 1000;
    if (abs(mag.mag_data.x) > MAG_MAX_REASONABLE || 
        abs(mag.mag_data.y) > MAG_MAX_REASONABLE || 
        abs(mag.mag_data.z) > MAG_MAX_REASONABLE) {
        validReading = false;
    }

    if (!validReading) {
        consecutiveFailCount++;
        
        // Try to reset if we have multiple consecutive failures
        if (consecutiveFailCount >= 10 && (millis() - lastResetTime > 5000)) {
            Wire.endTransmission(true);
            delay(50);
            
            if (mag.initialize() == BMM150_OK) {
                mag.set_op_mode(BMM150_NORMAL_MODE);
                mag.set_presetmode(BMM150_PRESETMODE_HIGHACCURACY);
            }
            
            lastResetTime = millis();
        }
        
        // Use last valid values if available, otherwise set to zero
        if (SensorManager_hasValidReadingEver) {
            magData[0] = SensorManager_lastValidX;
            magData[1] = SensorManager_lastValidY;
            magData[2] = SensorManager_lastValidZ;
        } else {
            magData[0] = 0;
            magData[1] = 0;
            magData[2] = 0;
        }
        return;
    }    // Valid reading - reset fail counter and apply low-pass filtering
    consecutiveFailCount = 0;
    
    // Apply coordinate frame alignment: Flip X-axis to match BMI088 coordinate frame
    // Since Y+ and Z+ already match between BMM150 and BMI088, only X needs inversion
    float alignedX = -mag.mag_data.x;  // Flip X-axis for frame alignment
    float alignedY = mag.mag_data.y;   // Y-axis already aligned
    float alignedZ = mag.mag_data.z;   // Z-axis already aligned
    
    // Apply EMA low-pass filters to reduce EMI noise
    magData[0] = magFilterX.update(alignedX);
    magData[1] = magFilterY.update(alignedY);
    magData[2] = magFilterZ.update(alignedZ);
      #if DEBUG_SENSORS
    // Debug output to compare raw vs coordinate-aligned vs filtered magnetometer values (only every 100 samples to reduce spam)
    static int debugMagCounter = 0;
    if (++debugMagCounter >= 100) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "<DEBUG:MAG_RAW:%.0f,%.0f,%.0f,ALIGNED:%.0f,%.0f,%.0f,FILTERED:%.0f,%.0f,%.0f>",
                (float)mag.mag_data.x, (float)mag.mag_data.y, (float)mag.mag_data.z,
                alignedX, alignedY, alignedZ,
                magData[0], magData[1], magData[2]);
        Serial.println(buffer);
        debugMagCounter = 0;
    }
    #endif
    
    // Cache filtered readings for fallback use
    SensorManager_lastValidX = static_cast<int16_t>(magData[0]);
    SensorManager_lastValidY = static_cast<int16_t>(magData[1]);
    SensorManager_lastValidZ = static_cast<int16_t>(magData[2]);
    SensorManager_hasValidReadingEver = true;
}

void SensorManager::readBarometer() {
    // Read all values once and check validity
    float tempReading = baro.readTemperature();
    float pressureReading = baro.readPressure();
    float currentAltitude = baro.readAltitude(1013.25); // Standard pressure at sea level
    
    if (!isnan(tempReading) && !isnan(pressureReading) && !isnan(currentAltitude)) {
        // Store valid readings
        temperature = tempReading;
        pressure = pressureReading;
        
        // Check if this is the first valid reading to set the reference point
        static bool referenceAltitudeSet = false;
        static float referenceAltitude = 0.0f;
        static int readingCount = 0;
        
        // Wait for a few readings before setting reference to ensure stability
        readingCount++;
        
        if (!referenceAltitudeSet && readingCount > 10) {
            referenceAltitude = currentAltitude;
            referenceAltitudeSet = true;
            
            #if DEBUG_SENSORS
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "<DEBUG:BARO_REF_ALT_SET:%.2fm>", referenceAltitude);
            Serial.println(buffer);
            #endif
        }
          if (referenceAltitudeSet) {
            // Calculate relative altitude and apply low-pass filtering for stability
            float relativeAltitude = currentAltitude - referenceAltitude;
            altitude = altitudeFilter.update(relativeAltitude);
        } else {
            // Before reference is set, use absolute altitude (no filtering during initialization)
            altitude = currentAltitude;
        }
          #if DEBUG_SENSORS
        // Debug altitude values periodically (raw vs filtered)
        static unsigned long lastAltDebugTime = 0;
        if (millis() - lastAltDebugTime > 5000) {
            char buffer[128];
            if (referenceAltitudeSet) {
                float rawRelative = currentAltitude - referenceAltitude;
                snprintf(buffer, sizeof(buffer), "<DEBUG:ALTITUDE:raw_rel=%.2f,filtered=%.2f,ref=%.2f>", 
                         rawRelative, altitude, referenceAltitude);
            } else {
                snprintf(buffer, sizeof(buffer), "<DEBUG:ALTITUDE:current=%.2f,ref=NOT_SET,relative=%.2f>", 
                         currentAltitude, altitude);
            }
            Serial.println(buffer);
            lastAltDebugTime = millis();
        }
        #endif
        
    } else {
        // Note: Don't set success=false here since this function doesn't return bool
        // The calling function (updateWithDiagnostics) should handle this
    }
}

void SensorManager::updateTime() {
    // Get current time from RTC
    DateTime now = rtc.now();
    
    // Store RTC timestamp in global variables for later use in packet
    rtcYear = now.year() % 100;  // Last two digits of year
    rtcMonth = now.month();
    rtcDay = now.day();
    rtcHour = now.hour();
    rtcMinute = now.minute();
    rtcSecond = now.second();
    
    #if DEBUG_SENSORS
    // Periodically log the RTC time
    static unsigned long lastRtcDebugTime = 0;
    if (millis() - lastRtcDebugTime > 10000) { // Every 10 seconds
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "<DEBUG:RTC_TIME:%04d-%02d-%02d %02d:%02d:%02d>", 
                 now.year(), now.month(), now.day(), 
                 now.hour(), now.minute(), now.second());
        Serial.println(buffer);
        lastRtcDebugTime = millis();
    }
    #endif
}

void SensorManager::processSensorData() {
    // Prepare packet data
    currentPacket.packetId = packetCounter++;
    
    // Update timestamp to hold milliseconds since boot 
    // (actual date/time formatting will be done in FC when sending)
    currentPacket.timestamp = millis();
    
    // Scale and convert all values to integers with appropriate scaling
      // Temperature rounded to nearest whole degree (not centi-degrees anymore)
    currentPacket.temperature = static_cast<int16_t>(round(temperature));
      // Altitude in cm (meters * 100) - stored in cm but will be displayed as m with 2 decimal places
  
    
    currentPacket.altitude = static_cast<int32_t>(altitude * 100.0f);
    
    // Acceleration in mg (milli-g) (1g = 9.80665 m/s^2)
    currentPacket.accelX = static_cast<int16_t>(accelData[0] * 1000.0f / 9.80665f);
    currentPacket.accelY = static_cast<int16_t>(accelData[1] * 1000.0f / 9.80665f);
    currentPacket.accelZ = static_cast<int16_t>(accelData[2] * 1000.0f / 9.80665f);
    
    // Gyroscope in 0.01 dps (degrees per second) (rad/s * 180/π * 100)
    currentPacket.gyroX = static_cast<int16_t>(gyroData[0] * 57.2958f * 100.0f);
    currentPacket.gyroY = static_cast<int16_t>(gyroData[1] * 57.2958f * 100.0f);
    currentPacket.gyroZ = static_cast<int16_t>(gyroData[2] * 57.2958f * 100.0f);    // Magnetometer in 0.1 µT
    // Check for invalid readings (all zeros)
    bool magIsAllZeros = (magData[0] == 0 && magData[1] == 0 && magData[2] == 0);

    if (magIsAllZeros) {
        // Access the last valid magnetometer readings which were saved in readMagnetometer()
        static int16_t lastGoodX = 0;
        static int16_t lastGoodY = 0;
        static int16_t lastGoodZ = 0;
        static bool hasSeenGoodData = false;
        
        // Get reference to the cached good values from readMagnetometer() function
        extern bool SensorManager_hasValidReadingEver;
        extern int16_t SensorManager_lastValidX;
        extern int16_t SensorManager_lastValidY;
        extern int16_t SensorManager_lastValidZ;
        
        // Use last good values if we have them
        if (hasSeenGoodData) {
            // Use our own cached values
            currentPacket.magX = lastGoodX;
            currentPacket.magY = lastGoodY;
            currentPacket.magZ = lastGoodZ;
        } else if (SensorManager_hasValidReadingEver) {
            // Use values from the magnetometer reading function
            currentPacket.magX = SensorManager_lastValidX;
            currentPacket.magY = SensorManager_lastValidY;
            currentPacket.magZ = SensorManager_lastValidZ;
            
            // Save these for next time
            lastGoodX = SensorManager_lastValidX;
            lastGoodY = SensorManager_lastValidY; 
            lastGoodZ = SensorManager_lastValidZ;
            hasSeenGoodData = true;
        } else {
            // No good values ever - use neutral values that won't affect heading
            currentPacket.magX = 0;
            currentPacket.magY = 0;
            currentPacket.magZ = 1; // Tiny non-zero value to prevent division by zero
        }
        
        #if DEBUG_SENSORS
        static unsigned long lastPlaceholderTime = 0;
        if (millis() - lastPlaceholderTime > 10000) { // Log only every 10 seconds to reduce spam
            Serial.print("<DEBUG:MAG_USING_LASTVALUES:ALL_ZEROS");
            Serial.print(",X=");
            Serial.print(currentPacket.magX);
            Serial.print(",Y=");
            Serial.print(currentPacket.magY);
            Serial.print(",Z=");
            Serial.print(currentPacket.magZ);
            Serial.println(">");
            lastPlaceholderTime = millis();
        }
        #endif
    } else {
        // Use actual magnetometer values, BMM150 values are already in µT
        // Convert to 0.1µT by multiplying with 10 and cast to int16
        currentPacket.magX = static_cast<int16_t>(magData[0]); 
        currentPacket.magY = static_cast<int16_t>(magData[1]);
        currentPacket.magZ = static_cast<int16_t>(magData[2]);
    }    // GPS data - simplified processing using TinyGPSPlus library
    // Get satellite count
    if (gps.satellites.isValid()) {
        currentPacket.satellites = static_cast<uint8_t>(gps.satellites.value());
    } else {
        currentPacket.satellites = 0;
    }
    
    // Process location data
    if (gps.location.isValid() && currentPacket.satellites > 3) {
        // Valid GPS fix - convert to int32 in 1e-7 degrees format
        currentPacket.latitude = static_cast<int32_t>(gps.location.lat() * 10000000.0);
        currentPacket.longitude = static_cast<int32_t>(gps.location.lng() * 10000000.0);
    } else {
        // No valid fix - use special placeholder values
        if (currentPacket.satellites > 0) {
            currentPacket.latitude = 10;  // Satellites visible but no fix
            currentPacket.longitude = 10;
        } else {
            currentPacket.latitude = 1;   // No satellites
            currentPacket.longitude = 1;
        }
    }
    
    // Include RTC data in packet
    currentPacket.year = rtcYear;
    currentPacket.month = rtcMonth;
    currentPacket.day = rtcDay;
    currentPacket.hour = rtcHour;
    currentPacket.minute = rtcMinute;
    currentPacket.second = rtcSecond;    // Calculate CRC-16 for the packet using FrameCodec utility
    // We need to zero out the CRC field first to ensure consistent calculation
    currentPacket.crc16 = 0;
    
    // Calculate CRC on all fields except the CRC itself
    currentPacket.crc16 = FrameCodec::calculateSensorPacketCRC(
        reinterpret_cast<const uint8_t*>(&currentPacket), 
        sizeof(SensorPacket)
    );
    
    // Note: Packet streaming is now handled by the main loop using isPacketReady()
    // and PacketManager. This ensures both SD logging and UART transmission work together.
}

const SensorPacket& SensorManager::getPacket() const {
    return currentPacket;
}

bool SensorManager::isPacketReady() {
    unsigned long currentTime = millis();
    return (currentTime - lastStreamTime >= (1000 / UART_STREAM_RATE_HZ));
}

void SensorManager::markPacketConsumed() {
    // Update timing for next packet
    lastStreamTime = millis();
}

uint8_t SensorManager::getGpsSatelliteCount() const {
    // The satellite count is populated in currentPacket.satellites by processSensorData(),
    // which correctly handles TinyGPSPlus validity and value extraction in a non-const context.
    // processSensorData sets currentPacket.satellites to 0 if gps.satellites is not valid.
    return currentPacket.satellites;
}

void SensorManager::setStatusLED(uint8_t r, uint8_t g, uint8_t b) {
    statusLed.setPixelColor(0, statusLed.Color(r, g, b));
    statusLed.show();
    
    // Record the time when the LED was turned on (if any color is non-zero)
    if (r > 0 || g > 0 || b > 0) {
        ledOnTime = millis();
    } else {
        // If turning off the LED, reset the ledOnTime
        ledOnTime = 0;
    }
}


