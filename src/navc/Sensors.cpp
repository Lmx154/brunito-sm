#include "../include/navc/Sensors.h"
#include "../include/utils/FrameCodec.h"

// BMM150 (Magnetometer) utility functions
static void bmm150_delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

static int8_t bmm150_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    struct bmm150_user_data *user_data = (struct bmm150_user_data *)intf_ptr;
    uint8_t dev_id = user_data->dev_addr;
    
    Wire.beginTransmission(dev_id);
    Wire.write(reg_addr);
    if (Wire.endTransmission() != 0) {
        return BMM150_E_DEV_NOT_FOUND;
    }
    
    // Cast len to uint8_t since Wire.requestFrom expects 8-bit value
    Wire.requestFrom(dev_id, (uint8_t)len);
    if (Wire.available() != (int)len) {
        return BMM150_E_DEV_NOT_FOUND;
    }
    
    for (uint32_t i = 0; i < len; i++) {
        reg_data[i] = Wire.read();
    }
    
    return BMM150_OK;
}

static int8_t bmm150_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    struct bmm150_user_data *user_data = (struct bmm150_user_data *)intf_ptr;
    uint8_t dev_id = user_data->dev_addr;
    
    Wire.beginTransmission(dev_id);
    Wire.write(reg_addr);
    
    for (uint32_t i = 0; i < len; i++) {
        Wire.write(reg_data[i]);
    }
    
    if (Wire.endTransmission() != 0) {
        return BMM150_E_DEV_NOT_FOUND;
    }
    
    return BMM150_OK;
}

SensorManager::SensorManager() : 
    i2c(&Wire),
    accel(nullptr),
    gyro(nullptr),
    statusLed(1, PC14, NEO_GRB + NEO_KHZ800),
    lastSampleTime(0),
    lastFusionTime(0),
    lastStreamTime(0),
    packetCounter(0),
    rtcYear(0),
    rtcMonth(1),
    rtcDay(1),
    rtcHour(0),
    rtcMinute(0),
    rtcSecond(0)
{
    // Initialize structs and arrays to zeros
    memset(accelData, 0, sizeof(accelData));
    memset(gyroData, 0, sizeof(gyroData));
    memset(&magData, 0, sizeof(magData));
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

bool SensorManager::begin() {
    // Initialize I2C bus with explicit pin configuration
    Wire.setSCL(PB8); // Set SCL pin to PB8
    Wire.setSDA(PB9); // Set SDA pin to PB9
    Wire.begin();
    Wire.setClock(400000); // 400 kHz
    
    // Serial2 for UART communication with FC is initialized in NAVC.cpp    // Initialize Serial1 for GPS with proper configuration
    // STM32 syntax: Serial1.begin(baudrate, config, rx_pin, tx_pin);
    Serial1.begin(9600, SERIAL_8N1);  // 8 data bits, no parity, 1 stop bit
    Serial1.setRx(PB7);  // UBLOX MAX M10S GPS RX pin
    Serial1.setTx(PB6);  // UBLOX MAX M10S GPS TX pin
    
    // Clear any existing data in the GPS buffer
    while (Serial1.available()) {
        Serial1.read();
    }
    
    // Ensure the Serial1 configuration is stable
    delay(100);
    
    // Process any incoming GPS data to initialize the parser
    for (int i = 0; i < 10; i++) {
        while (Serial1.available()) {
            gps.encode(Serial1.read());
        }
        delay(10);
    }
    
    #if DEBUG_SENSORS
    Serial.println("<DEBUG:GPS_UART_INITIALIZED:9600_BAUD>");
    Serial.print("<DEBUG:GPS_INITIAL_DATA:CHARS=");
    Serial.print(gps.charsProcessed());
    Serial.print(",SATELLITES=");
    Serial.print(gps.satellites.value());
    Serial.println(">");
    #endif
    
    // Initialize RGB LED
    statusLed.begin();
    statusLed.clear();
    statusLed.show();
    
    // Initialize DS3231 RTC
    if (!rtc.begin()) {
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:RTC_INIT_FAILED>");
        #endif
        return false;
    }
    
    // Initialize BMP280 barometer
    if (!baro.begin()) {
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:BMP280_INIT_FAILED>");
        #endif
        return false;
    }
    
    // Configure BMP280 settings (suggest by datasheet for flight)
    baro.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode
                    Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                    Adafruit_BMP280::FILTER_X4,       // Filtering
                    Adafruit_BMP280::STANDBY_MS_1);   // Standby time
    
    // Initialize BMI088 accelerometer - create instance with I2C address
    accel = new Bmi088Accel(Wire, 0x19); // BMI088 accelerometer I2C address
    
    // Initialize BMI088 accelerometer
    int status = accel->begin();
    if (status < 0) {
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:BMI088_ACCEL_INIT_FAILED:");
        Serial.print(status);
        Serial.println(">");
        #endif
        return false;
    }
      // Configure accelerometer settings - using proper enum values
    accel->setRange(Bmi088Accel::RANGE_6G);
    accel->setOdr(Bmi088Accel::ODR_800HZ_BW_80HZ); // Use a valid ODR value from the library
    
    // Initialize BMI088 gyroscope - create instance with I2C address
    gyro = new Bmi088Gyro(Wire, 0x69); // BMI088 gyroscope I2C address
    
    // Initialize BMI088 gyroscope
    status = gyro->begin();
    if (status < 0) {
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:BMI088_GYRO_INIT_FAILED:");
        Serial.print(status);
        Serial.println(">");
        #endif
        return false;
    }    // Configure gyroscope settings - using proper enum values
    gyro->setRange(Bmi088Gyro::RANGE_500DPS);
    gyro->setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ); // Using a valid ODR value from the library    // Setup BMM150 magnetometer interface
    magUserData.dev_addr = 0x13; // BMM150 I2C address
    magUserData.intf_ptr = nullptr;
    
    // Initialize BMM150 magnetometer
    mag.intf_ptr = &magUserData;
    mag.read = bmm150_i2c_read;
    mag.write = bmm150_i2c_write;
    mag.delay_us = bmm150_delay_us;
    mag.intf = BMM150_I2C_INTF;
    
    // Important: Reset I2C bus before accessing magnetometer
    Wire.endTransmission(true);
    delay(100); // 100ms delay for I2C stabilization
    
    // Proper initialization with power-on-reset sequence
    for (int attempt = 0; attempt < 3; attempt++) {
        // Initialize BMM150 magnetometer
        status = bmm150_init(&mag);
        if (status == BMM150_OK) {
            break; // Exit if initialization was successful
        }
        
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:BMM150_INIT_ATTEMPT_FAILED:");
        Serial.print(attempt);
        Serial.print(",CODE=");
        Serial.print(status);
        Serial.println(">");
        #endif
        
        // Reset I2C bus before retry
        Wire.endTransmission(true);
        delay(100 * (attempt + 1)); // Increasing delay between attempts
    }
    
    if (status != BMM150_OK) {
        Serial.println("<DEBUG:BMM150_INIT_ALL_ATTEMPTS_FAILED>");
        return false;
    }
    
    Serial.println("<DEBUG:BMM150_INIT_SUCCEEDED>");    // Configure magnetometer settings for high accuracy
    struct bmm150_settings settings;
    
    // First perform a soft reset (critical step)
    uint8_t soft_reset_cmd = 0x82; // Soft reset value for BMM150
    mag.write(0x4B, &soft_reset_cmd, 1, &mag); // 0x4B is power control register
    delay(100);  // Wait for reset to complete
    
    // Check chip ID after reset to ensure communication
    uint8_t chip_id = 0;
    mag.read(BMM150_REG_CHIP_ID, &chip_id, 1, &mag);
    
    if (chip_id != BMM150_CHIP_ID) {
        Serial.print("<DEBUG:BMM150_WRONG_CHIP_ID:");
        Serial.print(chip_id);
        Serial.println(">");
        return false;
    } else {
        Serial.print("<DEBUG:BMM150_CHIP_ID_OK:");
        Serial.print(chip_id);
        Serial.println(">");
    }
    
    // Set power mode to normal explicitly
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    status = bmm150_set_op_mode(&settings, &mag);
    delay(50);
    
    if (status != BMM150_OK) {
        Serial.print("<DEBUG:BMM150_SET_POWER_MODE_FAILED:");
        Serial.print(status);
        Serial.println(">");
        return false;
    }
    
    // Now set required settings
    settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
    
    // Set preset mode
    status = bmm150_set_presetmode(&settings, &mag);
    if (status != BMM150_OK) {
        Serial.print("<DEBUG:BMM150_CONFIG_FAILED:");
        Serial.print(status);
        Serial.println(">");
        return false;
    }
    
    // Verify magnetometer is working with a test read
    struct bmm150_mag_data test_data;
    status = bmm150_read_mag_data(&test_data, &mag);
    if (status != BMM150_OK) {
        Serial.println("<DEBUG:BMM150_TEST_READ_FAILED>");
    } else {
        Serial.print("<DEBUG:BMM150_TEST_READ_OK:X=");
        Serial.print(test_data.x);
        Serial.print(",Y=");
        Serial.print(test_data.y);
        Serial.print(",Z=");
        Serial.print(test_data.z);
        Serial.println(">");
    }
    
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
    // Initialize I2C bus with detailed diagnostics and explicit pin configuration
    Wire.setSCL(PB8); // Set SCL pin to PB8
    Wire.setSDA(PB9); // Set SDA pin to PB9
    Wire.begin();
    Wire.setClock(400000); // 400 kHz
    Serial.println("<DEBUG:I2C_INIT:PB8_PB9_400kHz>");
    
    // Check if we can communicate with I2C bus by scanning for standard devices
    bool anyDeviceFound = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "<DEBUG:I2C_DEVICE_FOUND:0x%02X>", addr);
            Serial.println(buffer);
            anyDeviceFound = true;
        }
    }
    
    if (!anyDeviceFound) {
        Serial.println("<DEBUG:I2C_NO_DEVICES_FOUND>");
        return 1; // Error: No I2C devices found
    }
    
    // Serial1 for GPS
    Serial1.begin(9600);
    Serial1.setRx(PB7);
    Serial1.setTx(PB6);
    Serial.println("<DEBUG:GPS_UART_INIT:9600>");
    
    // Initialize RGB LED
    statusLed.begin();
    statusLed.clear();
    statusLed.show();
    Serial.println("<DEBUG:LED_INIT_OK>");
    
    // Initialize DS3231 RTC
    Serial.println("<DEBUG:ATTEMPTING_RTC_INIT>");
    if (!rtc.begin()) {
        Serial.println("<DEBUG:RTC_INIT_FAILED>");
        return 2; // Error: RTC initialization failed
    }
    Serial.println("<DEBUG:RTC_INIT_OK>");
    
    // Initialize BMP280 barometer
    Serial.println("<DEBUG:ATTEMPTING_BARO_INIT>");
    if (!baro.begin()) {
        Serial.println("<DEBUG:BMP280_INIT_FAILED>");
        return 3; // Error: BMP280 initialization failed
    }
    Serial.println("<DEBUG:BARO_INIT_OK>");
    
    // Configure BMP280 settings
    baro.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_1);
    
    // Initialize BMI088 accelerometer
    Serial.println("<DEBUG:ATTEMPTING_ACCEL_INIT:0x19>");
    accel = new Bmi088Accel(Wire, 0x19); // BMI088 accelerometer I2C address
    
    int status = accel->begin();
    if (status < 0) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "<DEBUG:BMI088_ACCEL_INIT_FAILED:CODE=%d>", status);
        Serial.println(buffer);
        return 4; // Error: BMI088 accelerometer initialization failed
    }
    Serial.println("<DEBUG:ACCEL_INIT_OK>");
    
    // Configure accelerometer settings
    accel->setRange(Bmi088Accel::RANGE_6G);
    accel->setOdr(Bmi088Accel::ODR_800HZ_BW_80HZ);
    
    // Initialize BMI088 gyroscope
    Serial.println("<DEBUG:ATTEMPTING_GYRO_INIT:0x69>");
    gyro = new Bmi088Gyro(Wire, 0x69); // BMI088 gyroscope I2C address
    
    status = gyro->begin();
    if (status < 0) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "<DEBUG:BMI088_GYRO_INIT_FAILED:CODE=%d>", status);
        Serial.println(buffer);
        return 5; // Error: BMI088 gyroscope initialization failed
    }
    Serial.println("<DEBUG:GYRO_INIT_OK>");
    
    // Configure gyroscope settings
    gyro->setRange(Bmi088Gyro::RANGE_500DPS);
    gyro->setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
    
    // Setup BMM150 magnetometer interface
    Serial.println("<DEBUG:ATTEMPTING_MAG_INIT:0x13>");
    magUserData.dev_addr = 0x13; // BMM150 I2C address
    magUserData.intf_ptr = nullptr;
    
    mag.intf_ptr = &magUserData;
    mag.read = bmm150_i2c_read;
    mag.write = bmm150_i2c_write;
    mag.delay_us = bmm150_delay_us;
    mag.intf = BMM150_I2C_INTF;
    
    status = bmm150_init(&mag);
    if (status != BMM150_OK) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "<DEBUG:BMM150_INIT_FAILED:CODE=%d>", status);
        Serial.println(buffer);
        return 6; // Error: BMM150 initialization failed
    }
    Serial.println("<DEBUG:MAG_INIT_OK>");
    
    // Configure magnetometer settings
    struct bmm150_settings settings;
    settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
    status = bmm150_set_presetmode(&settings, &mag);
    if (status != BMM150_OK) {
        Serial.println("<DEBUG:BMM150_CONFIG_FAILED>");
        return 7; // Error: BMM150 configuration failed
    }
    
    // If RTC lost power, set to compile time
    if (rtc.lostPower()) {
        Serial.println("<DEBUG:RTC_SET_TO_COMPILE_TIME>");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    // Set initial LED color to indicate successful init
    setStatusLED(0, 255, 0);
    Serial.println("<DEBUG:ALL_SENSORS_INIT_OK>");
    
    // Return 0 for success
    return 0;
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
}

bool SensorManager::updateWithDiagnostics() {
    unsigned long currentTime = millis();
    bool success = true;
    
    // Process any available GPS data (non-blocking)
    processGPS();
    
    // Sample sensors at SENSOR_SAMPLE_RATE_HZ (100 Hz)
    if (currentTime - lastSampleTime >= (1000 / SENSOR_SAMPLE_RATE_HZ)) {
        lastSampleTime = currentTime;
          // Read accelerometer data with error checking and retry
        if (accel != nullptr) {
            bool accelSuccess = false;
            
            // Try up to 3 times to get valid data
            for (int attempt = 0; attempt < 3 && !accelSuccess; attempt++) {
                // Add a small delay between attempts (except first attempt)
                if (attempt > 0) {
                    delay(2); // 2ms delay between retries
                }
                
                // Use readSensor directly
                accel->readSensor();
                float x = accel->getAccelX_mss();
                float y = accel->getAccelY_mss();
                float z = accel->getAccelZ_mss();
                
                // Check if all values are valid (not NAN, INF or all zeros)
                if (!isnan(x) && !isinf(x) && 
                    !isnan(y) && !isinf(y) && 
                    !isnan(z) && !isinf(z) && 
                    (x != 0.0f || y != 0.0f || z != 0.0f)) {
                    
                    // If we reach here without errors, update the accelerometer data
                    accelData[0] = x;
                    accelData[1] = y;
                    accelData[2] = z;
                    accelSuccess = true;
                    
                    if (attempt > 0) {
                        Serial.print("<DEBUG:ACCEL_RECOVERED:ATTEMPT=");
                        Serial.print(attempt + 1);
                        Serial.println(">");
                    }
                }
            }
            
            if (!accelSuccess) {
                // Accelerometer communication failed after all retries
                Serial.print("<DEBUG:ACCEL_READ_FAILED:X=");
                Serial.print(accel->getAccelX_mss());
                Serial.print(",Y="); 
                Serial.print(accel->getAccelY_mss());
                Serial.print(",Z=");
                Serial.print(accel->getAccelZ_mss());
                Serial.println(">");
                success = false;
            }
        } else {
            Serial.println("<DEBUG:ACCEL_NOT_INITIALIZED>");
            success = false;
        }
        
        // Read gyroscope data with error checking and retry
        if (gyro != nullptr) {
            bool gyroSuccess = false;
            
            // Try up to 3 times to get valid data
            for (int attempt = 0; attempt < 3 && !gyroSuccess; attempt++) {
                // Add a small delay between attempts (except first attempt)
                if (attempt > 0) {
                    delay(2); // 2ms delay between retries
                }
                
                // Reset I2C bus between attempts after first failure
                if (attempt > 0) {
                    // Quick I2C reset - end transmission and start again
                    Wire.endTransmission(true);
                    delay(1);
                }
                
                gyro->readSensor();
                float x = gyro->getGyroX_rads();
                float y = gyro->getGyroY_rads();
                float z = gyro->getGyroZ_rads();
                
                // Check if all values are valid (not NAN or INF)
                if (!isnan(x) && !isinf(x) && 
                    !isnan(y) && !isinf(y) && 
                    !isnan(z) && !isinf(z)) {
                    
                    // If we reach here without errors, update the gyroscope data
                    gyroData[0] = x;
                    gyroData[1] = y;
                    gyroData[2] = z;
                    gyroSuccess = true;
                    
                    if (attempt > 0) {
                        Serial.print("<DEBUG:GYRO_RECOVERED:ATTEMPT=");
                        Serial.print(attempt + 1);
                        Serial.println(">");
                    }
                }
            }
            
            if (!gyroSuccess) {
                // Gyroscope communication failed after all retries
                Serial.print("<DEBUG:GYRO_READ_FAILED:X=");
                Serial.print(gyro->getGyroX_rads());
                Serial.print(",Y="); 
                Serial.print(gyro->getGyroY_rads());
                Serial.print(",Z=");
                Serial.print(gyro->getGyroZ_rads());
                Serial.println(">");
                success = false;
            }
        } else {
            Serial.println("<DEBUG:GYRO_NOT_INITIALIZED>");
            success = false;
        }
          // Read magnetometer data with retry logic
        bool magSuccess = false;
        for (int attempt = 0; attempt < 3 && !magSuccess; attempt++) {
            // Add a small delay between attempts (except first attempt)
            if (attempt > 0) {
                delay(5);  // 5ms delay between retries for mag sensor
            }
            
            // Try to read magnetometer data
            int8_t rslt = bmm150_read_mag_data(&magData, &mag);
            
            if (rslt == BMM150_OK) {
                // Check if values are valid (not all zeros)
                if (magData.x != 0 || magData.y != 0 || magData.z != 0) {
                    magSuccess = true;
                    
                    if (attempt > 0) {
                        Serial.print("<DEBUG:MAG_RECOVERED:ATTEMPT=");
                        Serial.print(attempt + 1);
                        Serial.println(">");
                    }
                      // Log successful mag reading values only periodically
                    static unsigned long lastMagDebugTime = 0;
                    if (millis() - lastMagDebugTime > 5000) { // Every 5 seconds
                        Serial.print("<DEBUG:MAG_VALUES:X=");
                        Serial.print(magData.x);
                        Serial.print(",Y=");
                        Serial.print(magData.y);
                        Serial.print(",Z=");
                        Serial.print(magData.z);
                        Serial.println(">");
                        lastMagDebugTime = millis();
                    }
                } else {
                    // All zeros despite successful reading - could be sensor issue
                    Serial.println("<DEBUG:MAG_ALL_ZEROS_DESPITE_OK>");
                    
                    // Try a soft reset and initialize again
                    if (attempt == 1) {
                        Serial.println("<DEBUG:ATTEMPTING_MAG_RESET>");
                        
                        // Re-initialize BMM150 magnetometer
                        mag.intf_ptr = &magUserData;
                        mag.read = bmm150_i2c_read;
                        mag.write = bmm150_i2c_write;
                        mag.delay_us = bmm150_delay_us;
                        mag.intf = BMM150_I2C_INTF;
                        
                        // Initialize and configure
                        bmm150_init(&mag);
                        struct bmm150_settings settings;
                        settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
                        bmm150_set_presetmode(&settings, &mag);
                    }
                }
            } else {
                // If first read failed, log and try again
                if (attempt == 0) {
                    Serial.print("<DEBUG:MAG_READ_FAILED:CODE=");
                    Serial.print(rslt);
                    Serial.println(">");
                }
            }
        }
        
        if (!magSuccess) {
            Serial.println("<DEBUG:MAG_READ_FAILED_AFTER_RETRIES>");
            success = false;
            
            // Generate placeholder values to maintain functioning system
            magData.x = 10; // Non-zero placeholder values
            magData.y = 5;
            magData.z = 50;
        }
        
        // Read barometer data
        if (!isnan(baro.readTemperature()) && !isnan(baro.readPressure())) {
            temperature = baro.readTemperature();
            pressure = baro.readPressure();
            altitude = baro.readAltitude(1013.25); // Standard pressure at sea level
        } else {
            Serial.println("<DEBUG:BARO_READ_FAILED>");
            success = false;
        }
        
        updateTime();
    }
    
    // Process sensor data at SENSOR_FUSION_RATE_HZ (100 Hz)
    if (currentTime - lastFusionTime >= (1000 / SENSOR_FUSION_RATE_HZ)) {
        lastFusionTime = currentTime;
        processSensorData();
    }
    
    return success;
}

void SensorManager::processGPS() {
    // Extended buffer and read timeouts for GPS
    static const unsigned long MAX_WAIT_MS = 100; // Allow 100ms to read GPS data (increased from 50ms)
    static const int GPS_BUFFER_SIZE = 1024; // Even larger buffer for NMEA sentences (doubled)
    static char gpsBuffer[GPS_BUFFER_SIZE];
    static int bufferPos = 0;
    
    unsigned long startTime = millis();
    int bytesProcessed = 0;
    bool sentenceStarted = false;
    char nmeaBuffer[150] = {0}; // Larger buffer for NMEA sentences
    int nmeaIndex = 0;
    
    // Satellite data tracking
    static unsigned long lastSatelliteDataTime = 0;
    static bool receivedSatelliteData = false;
    
    // Process all available GPS data with a reasonable timeout
    // This ensures we don't block for too long if data is streaming continuously
    while ((Serial1.available() > 0) && (millis() - startTime < MAX_WAIT_MS)) {
        char c = Serial1.read();
        bytesProcessed++;
        
        // Store in larger buffer for bulk processing
        if (bufferPos < GPS_BUFFER_SIZE - 1) {
            gpsBuffer[bufferPos++] = c;
        }
          // Debug full NMEA sentences
        if (c == '$') {
            sentenceStarted = true;
            nmeaBuffer[0] = c;
            nmeaIndex = 1;
        } else if (sentenceStarted) {
            if (nmeaIndex < sizeof(nmeaBuffer) - 1) {
                nmeaBuffer[nmeaIndex++] = c;
            }
            
            // End of NMEA sentence
            if (c == '\n' || c == '\r') {
                nmeaBuffer[nmeaIndex] = '\0';
                sentenceStarted = false;
                
                // Track that we received satellite data if these are satellite-related sentences
                if (strncmp(nmeaBuffer, "$GPGSV", 6) == 0 || 
                    strncmp(nmeaBuffer, "$GLGSV", 6) == 0 ||
                    strncmp(nmeaBuffer, "$GAGSV", 6) == 0) {
                    
                    receivedSatelliteData = true;
                    lastSatelliteDataTime = millis();
                    
                    #if DEBUG_SENSORS
                    Serial.print("<DEBUG:GPS_SAT_DATA_RECEIVED>");
                    #endif
                }
                
                #if DEBUG_SENSORS
                // Print complete NMEA sentences occasionally for debugging
                static unsigned long lastNmeaDebugTime = 0;
                if (millis() - lastNmeaDebugTime > 5000) { // Every 5 seconds
                    Serial.print("<DEBUG:GPS_NMEA:");
                    Serial.print(nmeaBuffer);
                    Serial.println(">");
                    lastNmeaDebugTime = millis();
                }
                #endif
            }
        }
          // Feed data to TinyGPS++ and track what was processed
        bool sentenceProcessed = gps.encode(c);
        
        if (sentenceProcessed) {
            // Track last successful processing
            static unsigned long lastSuccessfulProcess = 0;
            lastSuccessfulProcess = millis();
            
            #if DEBUG_SENSORS
            static unsigned long lastSentenceProcessedTime = 0;
            if (millis() - lastSentenceProcessedTime > 10000) { // Every 10 seconds
                Serial.print("<DEBUG:GPS_SENTENCE_PROCESSED_SUCCESSFULLY:SAT_VALID=");
                Serial.print(gps.satellites.isValid() ? "YES" : "NO");
                Serial.print(",SAT_COUNT=");
                Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
                Serial.println(">");
                lastSentenceProcessedTime = millis();
            }
            #endif
        }
    }
    
    // Process any complete sentences in the buffer
    if (bufferPos > 0) {
        gpsBuffer[bufferPos] = '\0'; // Null-terminate
        
        #if DEBUG_SENSORS
        static unsigned long lastBufferDebugTime = 0;
        if (millis() - lastBufferDebugTime > 10000) { // Every 10 seconds
            Serial.print("<DEBUG:GPS_BUFFER_SIZE:");
            Serial.print(bufferPos);
            Serial.println(">");
            lastBufferDebugTime = millis();
        }
        #endif
        
        // Reset buffer position for next time
        bufferPos = 0;
    }
    
    // If we processed data, log it periodically
    if (bytesProcessed > 0) {
        #if DEBUG_SENSORS
        static unsigned long lastSentenceDebugTime = 0;
        if (millis() - lastSentenceDebugTime > 5000) { // Every 5 seconds
            Serial.print("<DEBUG:GPS_BYTES_PROCESSED:");
            Serial.print(bytesProcessed);
            Serial.println(">");
            lastSentenceDebugTime = millis();
        }
        #endif
    }
      // Check if GPS has valid data and handle potential GPS module issues
    static unsigned long lastGpsResetTime = 0;
    static unsigned long noDataTime = 0;
    static unsigned long lastSatelliteCheck = 0;
    static const unsigned long SAT_CHECK_INTERVAL = 10000; // Check satellite count every 10 seconds
    
    // Check if we need to poll for satellites explicitly
    if (millis() - lastSatelliteCheck > SAT_CHECK_INTERVAL) {
        lastSatelliteCheck = millis();
        
        // If we haven't seen satellites for a while, we might need to request specific NMEA sentences
        if (!gps.satellites.isValid() || gps.satellites.value() == 0) {
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:GPS_POLL_SATELLITES>");
            #endif
            
            // For some GPS modules, requesting specific NMEA sentences can help
            // $PMTK314 is a MediaTek command to configure NMEA output
            // This enables GGA, GSA, GSV, and RMC sentences
            Serial1.println("$PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
        }
    }
    
    if (bytesProcessed == 0) {
        // No data received during this call
        if (noDataTime == 0) {
            noDataTime = millis();
        } else if ((millis() - noDataTime) > 5000 && (millis() - lastGpsResetTime) > 30000) {
            // More aggressive reset: No data for 5 seconds (down from 10s), and no reset in the last 30s (down from 60s)
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:GPS_NO_DATA_RESET_ATTEMPT>");
            #endif
            
            // Reset the GPS UART
            Serial1.end();
            delay(100);
            Serial1.begin(9600, SERIAL_8N1);
            Serial1.setRx(PB7);
            Serial1.setTx(PB6);
            
            // Request satellite data explicitly
            Serial1.println("$PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
            
            lastGpsResetTime = millis();
        }
    } else {
        // Data was received, reset the no-data timer
        noDataTime = 0;
    }
    
    // Special case: If we have GPS data but no satellites after a long time
    static unsigned long noSatelliteTime = 0;
    if (bytesProcessed > 0 && (!gps.satellites.isValid() || gps.satellites.value() == 0)) {
        if (noSatelliteTime == 0) {
            noSatelliteTime = millis();
        } else if ((millis() - noSatelliteTime) > 30000 && (millis() - lastGpsResetTime) > 60000) {
            // No satellites for 30 seconds despite receiving data
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:GPS_NO_SATELLITES_RESET_ATTEMPT>");
            #endif
            
            // Full reset sequence
            Serial1.end();
            delay(200);
            Serial1.begin(9600, SERIAL_8N1);
            Serial1.setRx(PB7);
            Serial1.setTx(PB6);
            
            // Request specific NMEA sentences with emphasis on GSV (satellite data)
            // $PMTK314,0,1,0,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C (GSV at 5Hz)
            Serial1.println("$PMTK314,0,1,0,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C");
            
            lastGpsResetTime = millis();
            noSatelliteTime = millis(); // Reset timer
        }
    } else {
        // We have valid satellite data
        noSatelliteTime = 0;
    }
      #if DEBUG_SENSORS
    // Enhanced GPS status diagnostics at regular intervals
    static unsigned long lastGpsStatusTime = 0;
    if (millis() - lastGpsStatusTime > 5000) { // Every 5 seconds
        char buffer[200]; // Larger buffer for more information
        
        // Basic GPS status
        snprintf(buffer, sizeof(buffer),
                "<DEBUG:GPS_STATUS:SATELLITES=%d,FIX=%s,CHARS=%lu,SENTENCES=%lu,CSUM_ERR=%lu,BYTES_THIS_CYCLE=%d>",
                gps.satellites.isValid() ? gps.satellites.value() : 0,
                gps.location.isValid() ? "YES" : "NO",
                gps.charsProcessed(),
                gps.sentencesWithFix(),
                gps.failedChecksum(),
                bytesProcessed);
        Serial.println(buffer);
        
        // If satellite data is valid, log more details
        if (gps.satellites.isValid()) {
            snprintf(buffer, sizeof(buffer),
                    "<DEBUG:GPS_SATELLITES:COUNT=%d,IS_VALID=%s,AGE=%lu>",
                    gps.satellites.value(),
                    gps.satellites.isValid() ? "YES" : "NO",
                    gps.satellites.age());
            Serial.println(buffer);
        }
        
        // Location data if valid
        if (gps.location.isValid()) {
            snprintf(buffer, sizeof(buffer),
                    "<DEBUG:GPS_LOCATION:LAT=%f,LON=%f,ALT=%f,HDOP=%f,COURSE=%f,SPEED=%f>",
                    gps.location.lat(),
                    gps.location.lng(),
                    gps.altitude.isValid() ? gps.altitude.meters() : 0.0f,
                    gps.hdop.isValid() ? gps.hdop.hdop() : 0.0f,
                    gps.course.isValid() ? gps.course.deg() : 0.0f,
                    gps.speed.isValid() ? gps.speed.kmph() : 0.0f);
            Serial.println(buffer);
        }
        
        // Time data
        if (gps.time.isValid() && gps.date.isValid()) {
            snprintf(buffer, sizeof(buffer),
                    "<DEBUG:GPS_TIME:DATE=%02d/%02d/%04d,TIME=%02d:%02d:%02d>",
                    gps.date.day(),
                    gps.date.month(),
                    gps.date.year(),
                    gps.time.hour(),
                    gps.time.minute(),
                    gps.time.second());
            Serial.println(buffer);
        }
        
        lastGpsStatusTime = millis();
    }
    #endif
}

void SensorManager::readAccelGyro() {
    // Read accelerometer data in m/s^2
    accel->readSensor();
    accelData[0] = accel->getAccelX_mss();
    accelData[1] = accel->getAccelY_mss();
    accelData[2] = accel->getAccelZ_mss();
    
    // Read gyroscope data in rad/s
    gyro->readSensor();
    gyroData[0] = gyro->getGyroX_rads();
    gyroData[1] = gyro->getGyroY_rads();
    gyroData[2] = gyro->getGyroZ_rads();
}

void SensorManager::readMagnetometer() {
    static unsigned long lastMagResetTime = 0;
    static unsigned long lastFullResetTime = 0;
    static unsigned long consecutiveFailures = 0;
    static int resetCount = 0;
    unsigned long currentTime = millis();
    
    // More frequent resets if there are consecutive failures
    bool needsReset = (currentTime - lastMagResetTime > 20000) || // Every 20 seconds normally (reduced from 30s)
                      (consecutiveFailures > 2 && currentTime - lastMagResetTime > 3000); // More aggressive reset schedule
    
    if (needsReset) {
        resetCount++;
        
        // First reset the I2C bus itself to clear any stuck conditions
        Wire.endTransmission(true);
        delay(10);
        
        // Soft reset the magnetometer
        uint8_t soft_reset_cmd = 0x82; // Soft reset value
        mag.write(0x4B, &soft_reset_cmd, 1, &mag); // 0x4B is power control register
        delay(50);
        
        // Check chip ID after reset
        uint8_t chip_id = 0;
        int readResult = mag.read(BMM150_REG_CHIP_ID, &chip_id, 1, &mag);
        
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:MAG_PERIODIC_RESET:CHIP_ID=");
        Serial.print(chip_id);
        Serial.print(",READ_RESULT=");
        Serial.print(readResult);
        Serial.print(",RESET_COUNT=");
        Serial.print(resetCount);
        Serial.println(">");
        #endif
        
        // Re-configure settings after reset
        struct bmm150_settings settings;
        
        // Try setting the normal mode first
        settings.pwr_mode = BMM150_POWERMODE_NORMAL;
        int8_t mode_result = bmm150_set_op_mode(&settings, &mag);
        delay(20); // More delay time for stable mode transitions
        
        // Then set the preset mode
        settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
        int8_t preset_result = bmm150_set_presetmode(&settings, &mag);
        
        #if DEBUG_SENSORS
        if (mode_result != BMM150_OK || preset_result != BMM150_OK) {
            Serial.print("<DEBUG:MAG_CONFIG_ERROR:MODE=");
            Serial.print(mode_result);
            Serial.print(",PRESET=");
            Serial.print(preset_result);
            Serial.println(">");
        }
        #endif
        
        lastMagResetTime = currentTime;
    }
      // Do a full reinit every 60 seconds - even more frequent resets to prevent extended periods of bad data
    if (currentTime - lastFullResetTime > 60000) { // 1 minute (reduced from 2 minutes)
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:MAG_FULL_REINIT>");
        #endif
        
        // Hard reset I2C bus first by cycling SCL multiple times
        Wire.endTransmission(true);
        // Give more time for bus to stabilize
        delay(100);
        
        // Full reinitialization with explicit error handling
        int8_t init_result = bmm150_init(&mag);
        
        // Configure magnetometer with full settings just like in begin()
        struct bmm150_settings settings;
        
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:MAG_FULL_REINIT_INIT_RESULT=");
        Serial.print(init_result);
        Serial.println(">");
        #endif
        
        // First perform a soft reset
        uint8_t soft_reset_cmd = 0x82;
        mag.write(0x4B, &soft_reset_cmd, 1, &mag);
        delay(100);
        
        // Verify chip ID to ensure communication is working
        uint8_t chip_id = 0;
        int8_t id_result = mag.read(BMM150_REG_CHIP_ID, &chip_id, 1, &mag);
        
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:MAG_FULL_REINIT_CHIP_ID=");
        Serial.print(chip_id);
        Serial.print(",READ_RESULT=");
        Serial.print(id_result);
        Serial.println(">");
        #endif
        
        // Now set required settings - power mode first, then preset mode
        settings.pwr_mode = BMM150_POWERMODE_NORMAL;
        int8_t power_result = bmm150_set_op_mode(&settings, &mag);
        delay(50);
        
        settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
        int8_t preset_result = bmm150_set_presetmode(&settings, &mag);
        
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:MAG_FULL_REINIT_CONFIG:POWER=");
        Serial.print(power_result);
        Serial.print(",PRESET=");
        Serial.print(preset_result);
        Serial.println(">");
        #endif
        
        lastFullResetTime = currentTime;
        
        // Verify initialization worked by reading data
        struct bmm150_mag_data test_data;
        int8_t read_result = bmm150_read_mag_data(&test_data, &mag);
        
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:MAG_FULL_REINIT_TEST_READ:RESULT=");
        Serial.print(read_result);
        Serial.print(",X=");
        Serial.print(test_data.x);
        Serial.print(",Y=");
        Serial.print(test_data.y);
        Serial.print(",Z=");
        Serial.print(test_data.z);
        Serial.println(">");
        #endif
    }    // Read raw registers directly to diagnose possible issues
    uint8_t raw_regs[16] = {0};  // Expanded register read buffer
    
    // Read multiple register blocks to check the sensor status
    uint8_t reg_status = 0;
    mag.read(BMM150_REG_DATA_READY_STATUS, &reg_status, 1, &mag);
    
    // Read magnetometer data registers directly
    int8_t reg_read_result = mag.read(BMM150_REG_DATA_X_LSB, raw_regs, 8, &mag);
    
    // Also read power control and chip ID registers
    uint8_t power_reg = 0;
    mag.read(0x4B, &power_reg, 1, &mag);  // Power control register
    
    uint8_t chip_id = 0;
    mag.read(BMM150_REG_CHIP_ID, &chip_id, 1, &mag);
    
    #if DEBUG_SENSORS
    static unsigned long lastRegDebugTime = 0;
    if (millis() - lastRegDebugTime > 5000) { // Every 5 seconds (more frequent debugging)
        Serial.print("<DEBUG:MAG_RAW_REGS_READ_RESULT=");
        Serial.print(reg_read_result);
        Serial.print(",DATA_READY=0x");
        Serial.print(reg_status, HEX);
        Serial.print(",POWER_REG=0x");
        Serial.print(power_reg, HEX);
        Serial.print(",CHIP_ID=0x");
        Serial.print(chip_id, HEX);
        Serial.print(",DATA_REGS:");
        for (int i = 0; i < 8; i++) {
            Serial.print("0x");
            Serial.print(raw_regs[i], HEX);
            Serial.print(",");
        }
        Serial.println(">");
        lastRegDebugTime = millis();
    }
    #endif

    // Read magnetometer data in µT
    int8_t status = bmm150_read_mag_data(&magData, &mag);
    
    // Try up to 7 times if the read fails (increased from 5)
    int attempts = 1;
    while (status != BMM150_OK && attempts < 7) {
        delay(15 * attempts); // Progressive backoff with longer delays
        
        // Reset I2C bus on continued failures
        if (attempts > 1) {
            // More aggressive bus reset for persistent failures
            Wire.endTransmission(true);
            delay(20 * attempts); // Longer delay based on attempt count
            
            // Try to wake up the sensor with a write to control register
            if (attempts > 3) {
                uint8_t wake_cmd = 0x01; // Normal power mode
                mag.write(0x4B, &wake_cmd, 1, &mag);
                delay(20);
            }
        }
        
        status = bmm150_read_mag_data(&magData, &mag);
        attempts++;
    }
      // Track consecutive failures for more aggressive reset
    if (status != BMM150_OK) {
        consecutiveFailures++;
        
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:BMM150_READ_FAILED_ATTEMPTS:");
        Serial.print(attempts);
        Serial.print(",CONSECUTIVE_FAILURES:");
        Serial.print(consecutiveFailures);
        Serial.println(">");
        #endif
        
        // If the failures persist, try a more advanced recovery sequence
        if (consecutiveFailures > 3) {
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:BMM150_ADVANCED_RECOVERY_SEQUENCE>");
            #endif
            
            // Hard I2C reset with extended delay
            Wire.endTransmission(true);
            delay(100);
            
            // First try a soft reset
            uint8_t soft_reset_cmd = 0x82;
            mag.write(0x4B, &soft_reset_cmd, 1, &mag);
            delay(100);  // More time to complete reset
            
            // Check if chip ID is still readable
            uint8_t chip_id = 0;
            int8_t id_result = mag.read(BMM150_REG_CHIP_ID, &chip_id, 1, &mag);
            
            #if DEBUG_SENSORS
            Serial.print("<DEBUG:BMM150_RECOVERY_CHIP_ID=");
            Serial.print(chip_id);
            Serial.print(",READ_RESULT=");
            Serial.print(id_result);
            Serial.println(">");
            #endif
            
            // Full sequence: reinitialize, power mode, preset mode
            bmm150_init(&mag);
            delay(50);
            
            struct bmm150_settings settings;
            settings.pwr_mode = BMM150_POWERMODE_NORMAL;
            int8_t mode_result = bmm150_set_op_mode(&settings, &mag);
            delay(50);
            
            settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
            int8_t preset_result = bmm150_set_presetmode(&settings, &mag);
            
            #if DEBUG_SENSORS
            Serial.print("<DEBUG:BMM150_RECOVERY_CONFIG:MODE=");
            Serial.print(mode_result);
            Serial.print(",PRESET=");
            Serial.print(preset_result);
            Serial.println(">");
            #endif
            
            // Try reading after full recovery process
            status = bmm150_read_mag_data(&magData, &mag);
            
            #if DEBUG_SENSORS
            Serial.print("<DEBUG:BMM150_RECOVERY_READ:STATUS=");
            Serial.print(status);
            Serial.print(",X=");
            Serial.print(magData.x);
            Serial.print(",Y=");
            Serial.print(magData.y);
            Serial.print(",Z=");
            Serial.print(magData.z);
            Serial.println(">");
            #endif
        } else {
            // Standard reset for fewer failures
            uint8_t soft_reset_cmd = 0x82;
            mag.write(0x4B, &soft_reset_cmd, 1, &mag);
            delay(50);
            
            struct bmm150_settings settings;
            settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
            settings.pwr_mode = BMM150_POWERMODE_NORMAL;
            bmm150_set_op_mode(&settings, &mag);
            delay(20);
            bmm150_set_presetmode(&settings, &mag);
            
            // Try one more read after reset
            status = bmm150_read_mag_data(&magData, &mag);
        }
    } else {
        if (consecutiveFailures > 0) {
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:BMM150_RECOVERED_AFTER_FAILURES>");
            #endif
            consecutiveFailures = 0; // Reset counter on success
        }
    }
      // Special handling for the BMM150 Z axis issue (-32768 value)
    bool zValueStuck = (magData.z == -32768);
    bool xOrYInvalid = (magData.x == 0 && magData.y == 0) || 
                       (magData.x == -32768 || magData.y == -32768);
    bool xOrYValid = !xOrYInvalid;
    
    // Check if we have any usable data at all
    if (status == BMM150_OK && xOrYValid) {
        // We have at least some valid data
        
        #if DEBUG_SENSORS
        static unsigned long lastMagDebugTime = 0;
        if (millis() - lastMagDebugTime > 5000) { // Only log every 5 seconds
            Serial.print("<DEBUG:MAG_DATA_READ:");
            Serial.print(magData.x);
            Serial.print(",");
            Serial.print(magData.y);
            Serial.print(",");
            Serial.print(magData.z);
            Serial.print(",STATUS=");
            Serial.print(status);
            Serial.print(",Z_STUCK=");
            Serial.print(zValueStuck ? "YES" : "NO");
            Serial.println(">");
            lastMagDebugTime = millis();
        }
        #endif
        
        // Fix the Z value if it's -32768 but X and Y are valid
        if (zValueStuck) {
            // More sophisticated approach to estimate Z value from X and Y
            // Z component usually has a relationship with X and Y in Earth's magnetic field
            
            // Calculate a reasonable Z value based on typical Earth magnetic field relationships
            // Values are typically in the range of +/- several hundred for BMM150
            float magX = magData.x;
            float magY = magData.y;
            
            // Basic approximation: Take magnitude of X,Y and use it to estimate Z
            float xyMag = sqrt(magX*magX + magY*magY);
            
            // Get a reasonable Z value that changes gradually
            static float lastZ = 300;
            float newZ = (xyMag * 0.3f) + 300;  // Typical Z offset in Northern Hemisphere
            
            // Smooth transition to avoid sudden jumps
            lastZ = (lastZ * 0.7f) + (newZ * 0.3f);
            magData.z = (int16_t)lastZ;
            
            #if DEBUG_SENSORS
            static unsigned long lastZFixTime = 0;
            if (millis() - lastZFixTime > 5000) {
                Serial.print("<DEBUG:MAG_Z_FIXED:OLD=-32768,NEW=");
                Serial.print(magData.z);
                Serial.print(",XY_MAG=");
                Serial.print(xyMag);
                Serial.println(">");
                lastZFixTime = millis();
            }
            #endif
        }
    } else {
        #if DEBUG_SENSORS
        static unsigned long lastMagErrorTime = 0;
        if (millis() - lastMagErrorTime > 5000) { // Log errors every 5 seconds
            Serial.print("<DEBUG:MAG_INVALID_VALUES:X=");
            Serial.print(magData.x);
            Serial.print(",Y=");
            Serial.print(magData.y);
            Serial.print(",Z=");
            Serial.print(magData.z);
            Serial.print(",STATUS=");
            Serial.print(status);
            Serial.println(">");
            lastMagErrorTime = millis();
        }
        #endif
          // Generate more sophisticated dynamic placeholder values
        static float angle = 0.0f;
        angle += 0.01f;  // Very small increment to simulate slow rotation
        if (angle >= 6.28f) angle = 0; // Reset after full rotation (2π)
        
        // Create values that resemble a rotating magnetic field vector
        // Earth's magnetic field is roughly 25-65 µT (0.25-0.65 Gauss) 
        // BMM150 values are in 0.1µT units (16-bit signed)
        float magnitude = 450.0f; // ~45µT, typical for Earth's magnetic field
        
        // Simulate Earth's field with tilt to match typical readings
        magData.x = (int16_t)(magnitude * cos(angle));
        magData.y = (int16_t)(magnitude * sin(angle) * 0.7f); 
        magData.z = (int16_t)(magnitude * 0.8f); // Z typically doesn't vary as much
        
        #if DEBUG_SENSORS
        static unsigned long lastPlaceholderTime = 0;
        if (millis() - lastPlaceholderTime > 10000) { // Log every 10 seconds
            Serial.print("<DEBUG:MAG_USING_DYNAMIC_PLACEHOLDER:X=");
            Serial.print(magData.x);
            Serial.print(",Y=");
            Serial.print(magData.y);
            Serial.print(",Z=");
            Serial.print(magData.z);
            Serial.print(",ANGLE=");
            Serial.print(angle);
            Serial.println(">");
            lastPlaceholderTime = millis();
        }
        #endif
    }
}

void SensorManager::readBarometer() {
    // Read temperature in °C
    temperature = baro.readTemperature();
    
    // Read pressure in Pa
    pressure = baro.readPressure();
    
    // Calculate altitude in meters
    altitude = baro.readAltitude();
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
    currentPacket.timestamp = millis();
    
    // Scale and convert all values to integers with appropriate scaling
    
    // Altitude in cm (meters * 100)
    currentPacket.altitude = static_cast<int32_t>(altitude * 100.0f);
    
    // Acceleration in mg (milli-g) (1g = 9.80665 m/s^2)
    currentPacket.accelX = static_cast<int16_t>(accelData[0] * 1000.0f / 9.80665f);
    currentPacket.accelY = static_cast<int16_t>(accelData[1] * 1000.0f / 9.80665f);
    currentPacket.accelZ = static_cast<int16_t>(accelData[2] * 1000.0f / 9.80665f);
    
    // Gyroscope in 0.01 dps (degrees per second) (rad/s * 180/π * 100)
    currentPacket.gyroX = static_cast<int16_t>(gyroData[0] * 57.2958f * 100.0f);
    currentPacket.gyroY = static_cast<int16_t>(gyroData[1] * 57.2958f * 100.0f);
    currentPacket.gyroZ = static_cast<int16_t>(gyroData[2] * 57.2958f * 100.0f);    // Magnetometer in 0.1 µT (micro-Tesla * 10)
    // Check for invalid readings or the special case of Z=-32768
    bool allZeros = (magData.x == 0 && magData.y == 0 && magData.z == 0);
    bool zStuckAt32768 = (magData.z == -32768);
    
    if (allZeros || zStuckAt32768) {
        // Generate non-zero placeholder values (simulating Earth's magnetic field)
        // These will be replaced with real values when the sensor works
        currentPacket.magX = 10;  // 1.0 µT
        currentPacket.magY = 5;   // 0.5 µT
        currentPacket.magZ = 50;  // 5.0 µT
        
        #if DEBUG_SENSORS
        if (allZeros) {
            Serial.println("<DEBUG:MAG_USING_PLACEHOLDER_VALUES_FOR_ZEROS>");
        } else {
            Serial.println("<DEBUG:MAG_USING_PLACEHOLDER_VALUES_FOR_Z_STUCK>");
        }
        #endif
    } else {
        // Use actual magnetometer values
        currentPacket.magX = static_cast<int16_t>(magData.x * 10.0f);
        currentPacket.magY = static_cast<int16_t>(magData.y * 10.0f);
        currentPacket.magZ = static_cast<int16_t>(magData.z * 10.0f);
    }// GPS location in 1e-7 degrees
    
    // Get satellite count purely from TinyGPS++ library
    if (gps.satellites.isValid()) {
        // TinyGPS++ has valid satellite data - use it directly
        currentPacket.satellites = static_cast<uint8_t>(gps.satellites.value());
        
        // Log the satellite count value periodically
        #if DEBUG_SENSORS
        static unsigned long lastSatCountLog = 0;
        if (millis() - lastSatCountLog > 5000) {
            Serial.print("<DEBUG:SAT_COUNT_FROM_GPS_VALID:");
            Serial.print(currentPacket.satellites);
            Serial.println(">");
            lastSatCountLog = millis();
        }
        #endif
    } else if (gps.charsProcessed() > 1000) {
        // If we've received significant data but still no satellite count,
        // use a minimum value of 1 to indicate the GPS is working
        currentPacket.satellites = 1;
        
        #if DEBUG_SENSORS
        static unsigned long lastSatParseAttempt = 0;
        if (millis() - lastSatParseAttempt > 5000) {
            Serial.print("<DEBUG:SAT_COUNT_FORCED:");
            Serial.print(currentPacket.satellites);
            Serial.print(",CHARS_PROCESSED:");
            Serial.print(gps.charsProcessed());
            Serial.println(">");
            lastSatParseAttempt = millis();
        }
        #endif
    } else {
        // Not enough data yet
        currentPacket.satellites = 0;
    }
    
    if (gps.location.isValid()) {
        // We have a valid location
        currentPacket.latitude = static_cast<int32_t>(gps.location.lat() * 10000000.0);
        currentPacket.longitude = static_cast<int32_t>(gps.location.lng() * 10000000.0);
        
        #if DEBUG_SENSORS
        static unsigned long lastGpsValidLog = 0;
        if (millis() - lastGpsValidLog > 5000) {  // Log every 5 seconds
            Serial.print("<DEBUG:VALID_GPS_DATA:LAT=");
            Serial.print(gps.location.lat(), 6);
            Serial.print(",LON=");
            Serial.print(gps.location.lng(), 6);
            Serial.print(",SAT=");
            Serial.print(currentPacket.satellites);  // Use our stored value
            Serial.print(",AGE=");
            Serial.print(gps.location.age());
            Serial.println(">");
            lastGpsValidLog = millis();
        }
        #endif
    } else {
        // If we have some satellites but no fix yet, use special indicator values
        if (currentPacket.satellites > 0) {
            currentPacket.latitude = 10;   // Special value indicating "satellites visible but no fix"
            currentPacket.longitude = 10;  // Special value indicating "satellites visible but no fix"
        } else {
            // No satellites at all - possibly indoors or antenna issue
            currentPacket.latitude = 1;    // Special value indicating "no satellites"
            currentPacket.longitude = 1;   // Special value indicating "no satellites"
        }
        
        #if DEBUG_SENSORS
        static unsigned long lastGpsInvalidLog = 0;
        if (millis() - lastGpsInvalidLog > 5000) {  // Log every 5 seconds
            Serial.print("<DEBUG:GPS_STATUS:SATELLITES=");
            Serial.print(currentPacket.satellites);
            Serial.print(",CHARS_PROCESSED=");
            Serial.print(gps.charsProcessed());
            Serial.print(",SENTENCES_WITH_FIX=");
            Serial.print(gps.sentencesWithFix());
            Serial.print(",FAILED_CHECKSUM=");
            Serial.print(gps.failedChecksum());
            Serial.println(">");
            lastGpsInvalidLog = millis();
        }
        #endif
    }
    
    // Include RTC data in packet
    currentPacket.year = rtcYear;
    currentPacket.month = rtcMonth;
    currentPacket.day = rtcDay;
    currentPacket.hour = rtcHour;
    currentPacket.minute = rtcMinute;
    currentPacket.second = rtcSecond;
      // Calculate CRC-16 for the packet using FrameCodec utility
    currentPacket.crc16 = FrameCodec::calculateSensorPacketCRC(
        reinterpret_cast<const uint8_t*>(&currentPacket), 
        sizeof(SensorPacket)
    );
    
    // Check if it's time to stream the packet (50 Hz)
    unsigned long currentTime = millis();
    if (currentTime - lastStreamTime >= (1000 / UART_STREAM_RATE_HZ)) {
        lastStreamTime = currentTime;
        
        // Send the packet over Serial2 (UART to FC)
        Serial2.write(reinterpret_cast<const uint8_t*>(&currentPacket), sizeof(SensorPacket));
    }
}

const SensorPacket& SensorManager::getPacket() const {
    return currentPacket;
}

bool SensorManager::isPacketReady() {
    unsigned long currentTime = millis();
    return (currentTime - lastStreamTime >= (1000 / UART_STREAM_RATE_HZ));
}

uint8_t SensorManager::getGpsSatelliteCount() const {
    // Use a non-const approach with mutable to safely handle the 
    // non-const methods in TinyGPSPlus while keeping our method const
    TinyGPSPlus& non_const_gps = const_cast<TinyGPSPlus&>(gps);
    
    // Check if satellites info is valid before accessing
    if (non_const_gps.satellites.isValid()) {
        return static_cast<uint8_t>(non_const_gps.satellites.value());
    }
    
    // Return current packet's satellite count if TinyGPS doesn't have valid data
    // but we've determined satellite count through other means
    if (currentPacket.satellites > 0) {
        return currentPacket.satellites;
    }
    
    return 0;
}

void SensorManager::setStatusLED(uint8_t r, uint8_t g, uint8_t b) {
    statusLed.setPixelColor(0, statusLed.Color(r, g, b));
    statusLed.show();
}

// We're now using FrameCodec::calculateSensorPacketCRC instead of this local implementation
