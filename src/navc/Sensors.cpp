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

bool SensorManager::begin() {
    // Initialize I2C bus with explicit pin configuration
    Wire.setSCL(PB8); // Set SCL pin to PB8
    Wire.setSDA(PB9); // Set SDA pin to PB9
    Wire.begin();
    Wire.setClock(100000); // 100 kHz for better stability with BMM150 (changed from 400kHz)    // Initialize SerialGPS for GPS communication
    // The pins are already configured in the SerialGPS constructor (PB7=RX, PB6=TX)
    // No need to manually set pin modes or use setRx/setTx since they're in the constructor
    
    // Initialize SerialGPS at default baud rate
    SerialGPS.begin(9600);       // UBLOX MAX-M10S default rate is 9600
    
    // Clear any existing data in the GPS buffer
    while (SerialGPS.available()) {
        SerialGPS.read();
    }
    
    delay(300);  // Longer delay for GPS UART to stabilize (increased from 200ms)
    
    // Set GPS to navigation mode with improved satellite acquisition    // UBX-CFG-PMS - Set power mode to full tracking power for faster acquisition
    uint8_t powerMode[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x95, 0x61};
    SerialGPS.write(powerMode, sizeof(powerMode));
    delay(100);
    
    // UBX-CFG-GNSS - Enable GPS+GLONASS for better satellite acquisition
    uint8_t enableSystems[] = {0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2C, 0x4B};
    SerialGPS.write(enableSystems, sizeof(enableSystems));
    delay(100);
    
    // UBX-CFG-NAV5 - Set to airborne<1g dynamic model for better height accuracy
    uint8_t dynamicModel[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
    SerialGPS.write(dynamicModel, sizeof(dynamicModel));
    delay(100);
    
    // UBX-CFG-RATE - Set update rate to 5Hz for better position tracking
    uint8_t updateRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    SerialGPS.write(updateRate, sizeof(updateRate));
    delay(100);
      // Ensure the SerialGPS configuration is stable and prime TinyGPSPlus
    for (int i = 0; i < 10; i++) {
        unsigned long startTime = millis();
        while (millis() - startTime < 100) { // Process for 100ms
            if (SerialGPS.available()) {
                gps.encode(SerialGPS.read());
            }
        }
    }
      #if DEBUG_SENSORS
    Serial.println("\n<DEBUG:GPS_DIAGNOSTIC_START>");
    Serial.print("<DEBUG:GPS_CHARS_PROCESSED:");
    Serial.print(gps.charsProcessed());
    Serial.println(">");
    Serial.print("<DEBUG:GPS_SENTENCES_WITH_FIX:");
    Serial.print(gps.sentencesWithFix());
    Serial.println(">");
    
    // Check if we're getting any data; if not, run the connection test
    if (gps.charsProcessed() == 0) {
        Serial.println("<DEBUG:GPS_NO_DATA_TESTING_CONNECTION>");
        bool connectionOk = testGpsConnection();
        
        if (connectionOk) {
            Serial.println("<DEBUG:GPS_CONNECTION_TEST_PASSED_BUT_NO_DATA_YET>");
        } else {
            Serial.println("<DEBUG:GPS_CONNECTION_TEST_FAILED:CHECK_WIRING>");
        }
    }
    #endif
    
    #if DEBUG_SENSORS
    // Echo raw GPS data to help diagnose connection issues
    Serial.println("<DEBUG:RAW_GPS_DATA_SAMPLE_BEGIN>");
    unsigned long rawStartTime = millis();
    int charsRead = 0;
    while (millis() - rawStartTime < 2000) { // Sample for 2 seconds
        if (SerialGPS.available()) {
            char c = SerialGPS.read();
            Serial.write(c);
            charsRead++;
        }
    }
    Serial.println("\n<DEBUG:RAW_GPS_DATA_SAMPLE_END>");
    Serial.print("<DEBUG:RAW_GPS_CHARS_READ:");
    Serial.print(charsRead);
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
    int accel_status = accel->begin();
    if (accel_status < 0) {
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:BMI088_ACCEL_INIT_FAILED:");
        Serial.print(accel_status);
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
    int gyro_status = gyro->begin();    if (gyro_status < 0) {
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:BMI088_GYRO_INIT_FAILED:");
        Serial.print(gyro_status);
        Serial.println(">");
        #endif
        return false;
    }    gyro->setRange(Bmi088Gyro::RANGE_500DPS);
    gyro->setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
    
    // Initialize BMM150 magnetometer using the Seeed Studio library
    #if DEBUG_SENSORS
    Serial.println("<DEBUG:BMM150_INIT_ATTEMPT>");
    #endif
    
    // Reset I2C bus before magnetometer initialization
    Wire.endTransmission(true);
    delay(100);
    
    // Initialize the BMM150 sensor
    if (mag.initialize() != BMM150_OK) {
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:BMM150_INIT_FAILED>");
        
        // Try a second attempt with reset
        Serial.println("<DEBUG:BMM150_TRYING_RESET_AND_REINIT>");
        
        // Toggle I2C lines to recover potentially stuck bus
        Wire.end();
        pinMode(PB8, OUTPUT); // SCL
        pinMode(PB9, OUTPUT); // SDA
        
        for (int i = 0; i < 10; i++) {
            digitalWrite(PB8, HIGH);
            delay(5);
            digitalWrite(PB8, LOW);
            delay(5);
        }
        
        // Return pins to input mode
        pinMode(PB8, INPUT_PULLUP);
        pinMode(PB9, INPUT_PULLUP);
        delay(100);
        
        // Restart I2C
        Wire.begin();
        Wire.setClock(100000);
        delay(100);
        
        if (mag.initialize() != BMM150_OK) {
            Serial.println("<DEBUG:BMM150_INIT_FAILED_AFTER_RESET>");
        } else {
            Serial.println("<DEBUG:BMM150_INIT_SUCCEEDED_AFTER_RESET>");
        }
        #endif
    } else {
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:BMM150_INIT_OK>");
        #endif
    }
    
    // Test read to verify magnetometer functionality
    mag.read_mag_data();
    float testX = mag.raw_mag_data.raw_datax;
    float testY = mag.raw_mag_data.raw_datay;
    float testZ = mag.raw_mag_data.raw_dataz;
    
    #if DEBUG_SENSORS
    Serial.print("<DEBUG:BMM150_TEST_DATA:X=");
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
    // Initialize I2C bus with detailed diagnostics and explicit pin configuration
    Wire.setSCL(PB8); // Set SCL pin to PB8
    Wire.setSDA(PB9); // Set SDA pin to PB9
    Wire.begin();
    Wire.setClock(100000); // 100 kHz for better stability with BMM150
    Serial.println("<DEBUG:I2C_INIT:PB8_PB9_100kHz>");
    
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
    }    // Initialize SerialGPS with more robust diagnostics
    Serial.println("<DEBUG:GPS_UART_INIT_BEGINNING>");
    
    // Print pin states before UART initialization
    Serial.print("<DEBUG:GPS_PIN_STATES:RX(PB7)=");
    Serial.print(digitalRead(PB7));
    Serial.print(",TX(PB6)=");
    Serial.print(digitalRead(PB6));
    Serial.println(">");
    
    // Initialize SerialGPS - pins are already configured in the constructor
    SerialGPS.begin(9600);
    delay(100); // Wait for UART to initialize    Serial.println("<DEBUG:GPS_UART_INIT:9600>");
    
    // Clear any existing data in buffer
    int clearCount = 0;
    while (SerialGPS.available()) {
        SerialGPS.read();
        clearCount++;
    }
    
    if (clearCount > 0) {
        Serial.print("<DEBUG:GPS_BUFFER_CLEARED:");
        Serial.print(clearCount);
        Serial.println(" BYTES>");
    }
      // Send test commands to GPS module
    Serial.println("<DEBUG:GPS_SENDING_TEST_COMMANDS>");
    
    // Send a simple NMEA test command
    SerialGPS.println("$PMTK000*32");
    SerialGPS.flush();
    delay(100);
    
    // Send a UBX command (ping)
    uint8_t pingCmd[] = {0xB5, 0x62, 0x01, 0x00, 0x00, 0x00, 0x01, 0x21};
    SerialGPS.write(pingCmd, sizeof(pingCmd));
    SerialGPS.flush();
    
    // Process any initial GPS data to help TinyGPSPlus
    Serial.println("<DEBUG:GPS_WAITING_FOR_RESPONSE>");
    int gpsCharsProcessed = 0;
    int bytesReceived = 0;
    unsigned long gpsCheckStart = millis();
      // Sample for 2 seconds (increased from 1) to allow for slower responses
    while (millis() - gpsCheckStart < 2000) {
        if (SerialGPS.available()) {
            char c = SerialGPS.read();
            bytesReceived++;
            
            // Print first 20 bytes as hex for debugging
            if (bytesReceived <= 20) {
                Serial.print(c, HEX);
                Serial.print(" ");
            }
            
            if (gps.encode(c)) {
                gpsCharsProcessed++;
            }
        }
    }
    
    Serial.println(); // New line after hex dump
    Serial.print("<DEBUG:GPS_INITIAL_RESPONSE:BYTES=");
    Serial.print(bytesReceived);
    Serial.print(",PROCESSED=");
    Serial.print(gpsCharsProcessed);
    Serial.println(">");
      // If we didn't get any response, try a more thorough diagnosis
    if (bytesReceived == 0) {
        Serial.println("<DEBUG:GPS_NO_RESPONSE_AT_9600_PERFORMING_DIAGNOSTICS>");
          // Try 115200 (some GPS modules default to this)
        SerialGPS.end();
        delay(100);
        SerialGPS.begin(115200);
        delay(100);
        
        // Send test message
        SerialGPS.println("$PMTK000*32");
        SerialGPS.flush();
        
        // Check for response at 115200
        bytesReceived = 0;
        gpsCheckStart = millis();
        while (millis() - gpsCheckStart < 500) { // Short check
            if (SerialGPS.available()) {
                SerialGPS.read();
                bytesReceived++;
            }
        }
        
        Serial.print("<DEBUG:GPS_AT_115200_BAUD:BYTES=");
        Serial.print(bytesReceived);
        Serial.println(">");
          // Return to 9600 baud as that should be correct for UBLOX MAX-M10S
        SerialGPS.end();
        delay(100);
        SerialGPS.begin(9600);
        
        // Run comprehensive connection test if we're still not getting data
        if (bytesReceived == 0) {
            Serial.println("<DEBUG:GPS_STILL_NO_RESPONSE_TESTING_CONNECTION>");
            bool connectionOk = testGpsConnection();
            
            if (!connectionOk) {
                Serial.println("<DEBUG:GPS_CONNECTION_TEST_FAILED>");
                Serial.println("<DEBUG:GPS_CHECK_HARDWARE:VERIFY_POWER_AND_WIRING>");
                Serial.println("<DEBUG:GPS_PINS:TX=PB6,RX=PB7>");
                Serial.println("<DEBUG:GPS_COMMON_ISSUES:1.LOOSE_WIRING_2.NO_POWER_3.TX/RX_SWAPPED>");
            }
        }
    }
    
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
    // Setup BMM150 magnetometer using Seeed Studio library
    Serial.println("<DEBUG:ATTEMPTING_MAG_INIT:0x13>");
    
    // Reset I2C bus before intensive magnetometer initialization
    Wire.endTransmission(true);
    delay(100);
    
    // Initialize with retry logic
    int8_t bmm_status = BMM150_E_ID_NOT_CONFORM;
    for (int attempt = 0; attempt < 3; attempt++) {
        Serial.print("<DEBUG:BMM150_INIT_ATTEMPT:");
        Serial.print(attempt + 1);
        Serial.println(">");
        
        bmm_status = mag.initialize();
        if (bmm_status == BMM150_OK) {
            Serial.println("<DEBUG:MAG_INIT_OK>");
            break;
        } else {
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "<DEBUG:BMM150_INIT_FAILED:CODE=%d>", bmm_status);
            Serial.println(buffer);
            
            // Reset I2C bus before retry
            Wire.endTransmission(true);
            delay(100 * (attempt + 1));
        }
    }
    
    if (bmm_status != BMM150_OK) {
        Serial.println("<DEBUG:BMM150_ALL_INIT_ATTEMPTS_FAILED>");
        return 6; // Error: BMM150 initialization failed
    }
    
    // Configure magnetometer settings for high accuracy mode
    mag.set_op_mode(BMM150_NORMAL_MODE);
    mag.set_presetmode(BMM150_PRESETMODE_HIGHACCURACY);
    delay(50); // Wait for settings to apply
    
    Serial.println("<DEBUG:BMM150_PRESET_MODE_SET>");
    
    // Test read to verify magnetometer functionality
    mag.read_mag_data();
    Serial.print("<DEBUG:BMM150_TEST_DATA:X=");
    Serial.print(mag.mag_data.x);
    Serial.print(",Y=");
    Serial.print(mag.mag_data.y);
    Serial.print(",Z=");
    Serial.print(mag.mag_data.z);
    Serial.println(">");
    
    // Check for common error patterns
    if (mag.mag_data.z == -32768) {
        Serial.println("<DEBUG:MAG_Z_AXIS_STUCK_DETECTED:CALIBRATION_REQUIRED>");
    } else if (mag.mag_data.x == 0 && mag.mag_data.y == 0 && mag.mag_data.z == 0) {
        Serial.println("<DEBUG:MAG_ALL_ZEROS_DETECTED:CALIBRATION_REQUIRED>");
    } else {
        Serial.println("<DEBUG:MAG_TEST_VALUES_APPEAR_NORMAL>");
    }
    
    // If RTC lost power, set to compile time
    if (rtc.lostPower()) {
        Serial.println("<DEBUG:RTC_SET_TO_COMPILE_TIME>");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    // Set initial LED color to indicate successful init (green)
    setStatusLED(0, 255, 0);
    Serial.println("<DEBUG:ALL_SENSORS_INIT_OK>");
    
    return 0; // Success - return 0 instead of true to be consistent with error codes
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
        }        // Read magnetometer data with retry logic using the Seeed Studio library
        bool magSuccess = false;
        for (int attempt = 0; attempt < 3 && !magSuccess; attempt++) {
            // Add a small delay between attempts (except first attempt)
            if (attempt > 0) {
                delay(5);  // 5ms delay between retries for mag sensor
            }
            
            // Try to read magnetometer data
            mag.read_mag_data();
            
            // Check if values are valid (not all zeros)
            if (mag.mag_data.x != 0 || mag.mag_data.y != 0 || mag.mag_data.z != 0) {
                // Store valid data in our array
                magData[0] = mag.mag_data.x;
                magData[1] = mag.mag_data.y;
                magData[2] = mag.mag_data.z;
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
                    Serial.print(mag.mag_data.x);
                    Serial.print(",Y=");
                    Serial.print(mag.mag_data.y);
                    Serial.print(",Z=");
                    Serial.print(mag.mag_data.z);
                    Serial.println(">");
                    lastMagDebugTime = millis();
                }
            } else {
                // All zeros despite reading - could be sensor issue
                Serial.println("<DEBUG:MAG_ALL_ZEROS_DESPITE_READ>");
                
                // Try a soft reset and initialize again
                if (attempt == 1) {
                    Serial.println("<DEBUG:ATTEMPTING_MAG_RESET>");
                    
                    // Re-initialize BMM150 magnetometer using Seeed library
                    if (mag.initialize() == BMM150_OK) {
                        mag.set_op_mode(BMM150_NORMAL_MODE);
                        mag.set_presetmode(BMM150_PRESETMODE_HIGHACCURACY);
                        Serial.println("<DEBUG:MAG_RESET_SUCCESSFUL>");
                    } else {
                        Serial.println("<DEBUG:MAG_RESET_FAILED>");
                    }
                }
            }
        }
          
        if (!magSuccess) {
            Serial.println("<DEBUG:MAG_READ_FAILED_AFTER_RETRIES>");
            success = false;
            
            // Use last valid values if available
            if (SensorManager_hasValidReadingEver) {
                magData[0] = SensorManager_lastValidX;
                magData[1] = SensorManager_lastValidY;
                magData[2] = SensorManager_lastValidZ;
                Serial.println("<DEBUG:DIAGNOSTICS_USING_LAST_VALID_MAG_VALUES>");
            } else {
                // No valid values ever recorded, use neutral values
                magData[0] = 0;
                magData[1] = 0;
                magData[2] = 1; // Small non-zero value to avoid division by zero
                Serial.println("<DEBUG:DIAGNOSTICS_USING_NEUTRAL_MAG_VALUES>");
            }
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
    // Enhanced GPS data processing with robust error recovery
    unsigned int bytesProcessed = 0;
    static unsigned long lastGpsResetTime = 0;
    static unsigned long lastDataCheck = 0;
    static unsigned long lastCharsProcessed = 0;
    static unsigned int consecutiveNoDataCount = 0;
    const unsigned long GPS_RESET_INTERVAL = 60000; // Reset GPS UART every 1 minute if no fix (reduced from 2 min)
    const unsigned long DATA_CHECK_INTERVAL = 3000; // Check for data flow every 3 seconds
    
    // Check if we're getting any data from the GPS module
    if (millis() - lastDataCheck >= DATA_CHECK_INTERVAL) {
        unsigned long currentCharsProcessed = gps.charsProcessed();
        
        // Compare with previous check to see if we're getting new data
        if (currentCharsProcessed == lastCharsProcessed) {
            // No new data received since last check
            consecutiveNoDataCount++;
            
            #if DEBUG_SENSORS
            Serial.print("<DEBUG:GPS_NO_DATA_FLOW:COUNT=");
            Serial.print(consecutiveNoDataCount);
            Serial.print(",SERIALGPS_AVAILABLE=");
            Serial.print(SerialGPS.available());
            Serial.print(",CHARS_PROCESSED=");
            Serial.print(currentCharsProcessed);
            Serial.println(">");
            #endif
            
            // After 3 consecutive no-data checks (9 seconds), try hardware recovery
            if (consecutiveNoDataCount >= 3) {
                // Reset GPS connection with extended diagnostics
                #if DEBUG_SENSORS
                Serial.println("<DEBUG:GPS_HARDWARE_RECOVERY_ATTEMPT>");
                #endif
                
                // Check if UART TX/RX lines might be swapped or disconnected
                Serial.println("<DEBUG:GPS_TESTING_PINS>");
                
                // End and reinitialize SerialGPS
                SerialGPS.end();
                delay(200);
                
                // No need to explicitly configure pins as they're handled in the SerialGPS constructor
                
                // Restart SerialGPS
                SerialGPS.begin(9600);
                delay(100);
                  // Clear any pending data
                while (SerialGPS.available()) {
                    SerialGPS.read();
                }
                
                // Send a test message and some configuration commands
                Serial.println("<DEBUG:GPS_SENDING_TEST_SEQUENCE>");
                
                // Send a basic NMEA command to test connectivity
                SerialGPS.println("$PMTK000*32"); // Test command (should get a response if GPS is working)
                delay(100);
                
                // Send UBLOX configuration commands with flush after each one
                // UBX-CFG-PMS - Set power mode to full power for better reception
                uint8_t powerMode[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x95, 0x61};
                SerialGPS.write(powerMode, sizeof(powerMode));
                SerialGPS.flush();
                delay(100);
                  // UBX-CFG-GNSS - Enable GPS+GLONASS for better coverage
                uint8_t enableSystems[] = {0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2C, 0x4B};
                SerialGPS.write(enableSystems, sizeof(enableSystems));
                SerialGPS.flush();
                delay(100);
                
                // Also send NMEA command to set update rate to 5Hz (works on most GPS modules)
                SerialGPS.println("$PMTK220,200*2C"); // 5Hz update rate (200ms)
                SerialGPS.flush();
                
                // Reset counters to give the GPS time to respond
                consecutiveNoDataCount = 0;
                lastGpsResetTime = millis();
                
                // Sample for responses for a short period
                Serial.println("<DEBUG:GPS_SAMPLING_FOR_RESPONSE>");
                unsigned long sampleStart = millis();
                int bytesReceived = 0;
                  // Sample for 1 second to see if we get any response
                while (millis() - sampleStart < 1000) {
                    if (SerialGPS.available()) {
                        char c = SerialGPS.read();
                        bytesReceived++;
                        gps.encode(c);
                        
                        #if DEBUG_SENSORS
                        // Print first 20 bytes of raw data to help debug
                        if (bytesReceived <= 20) {
                            Serial.print(c, HEX);
                            Serial.print(" ");
                        }
                        #endif
                    }
                }
                
                #if DEBUG_SENSORS
                Serial.print("<DEBUG:GPS_RECOVERY_RESPONSE_BYTES=");
                Serial.print(bytesReceived);
                Serial.println(">");
                #endif
            }
        } else {
            // We're getting new data, reset the counter
            consecutiveNoDataCount = 0;
            lastCharsProcessed = currentCharsProcessed;
            
            #if DEBUG_SENSORS
            static unsigned long lastDataFlowTime = 0;
            if (millis() - lastDataFlowTime > 30000) { // Don't spam log
                Serial.print("<DEBUG:GPS_DATA_FLOW_OK:CHARS=");
                Serial.print(currentCharsProcessed);
                Serial.println(">");
                lastDataFlowTime = millis();
            }
            #endif
        }
        
        lastDataCheck = millis();
    }
      // Standard fix timeout recovery (checks if we're getting data but no fix)
    if (millis() - lastGpsResetTime > GPS_RESET_INTERVAL && 
        gps.charsProcessed() > 5000 && gps.sentencesWithFix() == 0) {
        
        // If we're getting data but no fix, try sending config commands again
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:GPS_NO_FIX_TIMEOUT_RECONFIGURING>");
        #endif
        
        // Send configuration commands to improve satellite acquisition
        // UBX-CFG-NAV5 - Set to airborne<1g dynamic model for better acquisition
        uint8_t dynamicModel[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
        SerialGPS.write(dynamicModel, sizeof(dynamicModel));
        SerialGPS.flush();
        
        // Update the last reset time
        lastGpsResetTime = millis();
    }
      // Process all available data
    while (SerialGPS.available() > 0) {
        char c = SerialGPS.read();
        if (gps.encode(c)) {
            bytesProcessed++;
        }
    }
    
    // Periodically log GPS statistics to help diagnose issues
    #if DEBUG_SENSORS
    static unsigned long lastGpsStatusTime = 0;
    if (millis() - lastGpsStatusTime > 5000) { // Every 5 seconds
        // Create a comprehensive GPS status report
        Serial.print("<DEBUG:GPS_STATUS:CHARS_PROCESSED=");
        Serial.print(gps.charsProcessed());
        Serial.print(",SENTENCES_WITH_FIX=");
        Serial.print(gps.sentencesWithFix());
        Serial.print(",FAILED_CHECKSUMS=");
        Serial.print(gps.failedChecksum());
        Serial.print(",SATELLITES=");
        Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
        Serial.print(",VALID_LOCATION=");
        Serial.print(gps.location.isValid() ? "YES" : "NO");
        
        // If we have a valid location, print it
        if (gps.location.isValid()) {
            Serial.print(",LAT=");
            Serial.print(gps.location.lat(), 6);
            Serial.print(",LON=");
            Serial.print(gps.location.lng(), 6);
        }
        
        Serial.println(">");
        
        // Add advice if no fix after significant data processed
        if (gps.charsProcessed() > 5000 && gps.sentencesWithFix() == 0) {
            Serial.println("<DEBUG:GPS_NO_FIX_ADVICE:ENSURE_ANTENNA_HAS_CLEAR_SKY_VIEW>");
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
    // Attempt to read magnetometer data with the Seeed Studio BMM150 library
    static int8_t consecutiveFailCount = 0;
    static unsigned long lastResetAttempt = 0;
    static unsigned long lastSoftResetTime = 0;
    static unsigned long lastHardResetTime = 0;
    static bool needsFullReset = false;
    static uint16_t totalReadAttempts = 0;
    static uint16_t totalReadFailures = 0;
    
    // Increment read attempt counter
    totalReadAttempts++;
    
    // Check if we need a deep reset (hardware level)
    if (needsFullReset && millis() - lastHardResetTime > 60000) { // Every minute if needed
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:BMM150_ATTEMPTING_HARD_RESET>");
        #endif
        
        // Completely reset I2C bus
        Wire.end();
        delay(200);
        
        // Reinitialize I2C
        Wire.begin();
        Wire.setClock(100000); // 100 kHz for better stability
        delay(100);
        
        // Reinitialize magnetometer from scratch
        if (mag.initialize() != BMM150_OK) {
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:BMM150_HARD_RESET_FAILED>");
            #endif
        } else {
            // Set to high accuracy mode
            mag.set_op_mode(BMM150_NORMAL_MODE);
            mag.set_presetmode(BMM150_PRESETMODE_HIGHACCURACY);
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:BMM150_HARD_RESET_SUCCESSFUL>");
            #endif
        }
        
        // Set flags back
        lastHardResetTime = millis();
        needsFullReset = false;
        consecutiveFailCount = 0;
    }
    
    // Try soft reset if we have consecutive failures but not yet at hard reset threshold
    if (consecutiveFailCount >= 5 && millis() - lastSoftResetTime > 10000) {
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:BMM150_ATTEMPTING_SOFT_RESET>");
        #endif
        
        // Issue a soft reset by re-initializing the device
        if (mag.initialize() != BMM150_OK) {
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:BMM150_SOFT_RESET_FAILED>");
            #endif
        } else {
            // Set to high accuracy mode
            mag.set_op_mode(BMM150_NORMAL_MODE);
            mag.set_presetmode(BMM150_PRESETMODE_HIGHACCURACY);
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:BMM150_SOFT_RESET_SUCCESSFUL>");
            #endif
        }
        
        lastSoftResetTime = millis();
    }
    
    // Read magnetometer data
    bool readSuccess = true;
    
    // Attempt to read the magnetometer
    mag.read_mag_data(); // This updates mag.mag_data internally
    
    // Check for potential error conditions
    bool magDataInvalid = false;

    // Check if the values are reasonable
    const int16_t MAG_MAX_REASONABLE = 1000;
    if (mag.mag_data.x == 0 && mag.mag_data.y == 0 && mag.mag_data.z == 0) {
        magDataInvalid = true;
        readSuccess = false;
        #if DEBUG_SENSORS
        static unsigned long lastZerosTime = 0;
        if (millis() - lastZerosTime > 5000) {
            Serial.println("<DEBUG:BMM150_ALL_ZEROS_DETECTED>");
            lastZerosTime = millis();
        }
        #endif
    } else if (mag.mag_data.z == -32768) { // Check for the common stuck Z issue
        magDataInvalid = true;
        readSuccess = false;
        #if DEBUG_SENSORS
        static unsigned long lastStuckTime = 0;
        if (millis() - lastStuckTime > 5000) {
            Serial.println("<DEBUG:BMM150_Z_STUCK_AT_-32768>");
            lastStuckTime = millis();
        }
        #endif
    } else if (abs(mag.mag_data.x) > MAG_MAX_REASONABLE || 
              abs(mag.mag_data.y) > MAG_MAX_REASONABLE || 
              abs(mag.mag_data.z) > MAG_MAX_REASONABLE) {
        // Check for unreasonably large values
        magDataInvalid = true;
        readSuccess = false;
        #if DEBUG_SENSORS
        static unsigned long lastUnreasonableTime = 0;
        if (millis() - lastUnreasonableTime > 5000) {
            Serial.println("<DEBUG:BMM150_UNREASONABLE_VALUES_DETECTED>");
            lastUnreasonableTime = millis();
        }
        #endif
    }

    if (!readSuccess) {
        consecutiveFailCount++;
        totalReadFailures++;
        
        #if DEBUG_SENSORS
        // Log failures less frequently to avoid console spam
        static unsigned long lastFailTime = 0;
        if (millis() - lastFailTime > 5000) {
            Serial.print("<DEBUG:BMM150_READ_FAILED:CONSECUTIVE_FAILS=");
            Serial.print(consecutiveFailCount);
            Serial.print(",TOTAL_FAILURES=");
            Serial.print(totalReadFailures);
            Serial.print(",SUCCESS_RATE=");
            float successRate = 100.0f * (totalReadAttempts - totalReadFailures) / totalReadAttempts;
            Serial.print(successRate, 1);
            Serial.println("%>");
            lastFailTime = millis();
        }
        #endif

        // If we've had too many consecutive failures, attempt a hard reset
        if (consecutiveFailCount >= 20) {
            needsFullReset = true;
        }
        
        // If we've had multiple consecutive failures, attempt an immediate reset
        if (consecutiveFailCount >= 10 && (millis() - lastResetAttempt > 5000)) {
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:BMM150_ATTEMPTING_RESET>");
            #endif
            
            // Reset I2C bus
            Wire.endTransmission(true);
            delay(100);
            
            // Re-initialize and configure the sensor
            if (mag.initialize() == BMM150_OK) {
                mag.set_op_mode(BMM150_NORMAL_MODE);
                mag.set_presetmode(BMM150_PRESETMODE_HIGHACCURACY);
                #if DEBUG_SENSORS
                Serial.println("<DEBUG:BMM150_IMMEDIATE_RESET_SUCCESSFUL>");
                #endif
            }
            
            lastResetAttempt = millis();
        }
          
        // Use last valid values if available, otherwise set to zero
        if (SensorManager_hasValidReadingEver) {
            magData[0] = SensorManager_lastValidX;
            magData[1] = SensorManager_lastValidY;
            magData[2] = SensorManager_lastValidZ;
            
            #if DEBUG_SENSORS
            static unsigned long lastFallbackTime = 0;
            if (millis() - lastFallbackTime > 10000) {
                Serial.println("<DEBUG:BMM150_USING_LAST_VALID_VALUES>");
                lastFallbackTime = millis();
            }
            #endif
        } else {
            // Mark data as invalid by setting to all zeros - this will trigger placeholder use
            magData[0] = 0;
            magData[1] = 0;
            magData[2] = 0;
        }
        return;
    }
    
    // We got a valid reading, reset consecutive fail counter
    consecutiveFailCount = 0;
    
    // Store the values in our local array
    magData[0] = mag.mag_data.x;
    magData[1] = mag.mag_data.y;
    magData[2] = mag.mag_data.z;
    
    // Store valid readings and update "received good data ever" flag
    SensorManager_lastValidX = mag.mag_data.x;
    SensorManager_lastValidY = mag.mag_data.y;
    SensorManager_lastValidZ = mag.mag_data.z;
    SensorManager_hasValidReadingEver = true;
    
    // Log valid readings periodically
    #if DEBUG_SENSORS
    static unsigned long lastReadLogTime = 0;
    if (millis() - lastReadLogTime > 10000) {
        Serial.print("<DEBUG:BMM150_READING:X=");
        Serial.print(mag.mag_data.x);
        Serial.print(",Y=");
        Serial.print(mag.mag_data.y);
        Serial.print(",Z=");
        Serial.print(mag.mag_data.z);
        Serial.print(",SUCCESS_RATE=");
        float successRate = 100.0f * (totalReadAttempts - totalReadFailures) / totalReadAttempts;
        Serial.print(successRate, 1);
        Serial.println("%>");
        lastReadLogTime = millis();
        
        // Calibration reminder
        if (mag.mag_data.x < 30 && mag.mag_data.y < 30) { // Low values may indicate poor calibration
            Serial.println("<DEBUG:BMM150_CALIBRATION_REMINDER:MOVE_IN_FIGURE_8_PATTERN>");
        }
    }
    #endif
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
    
    // Temperature in centi-degrees (C * 100)
    currentPacket.temperature = static_cast<int16_t>(temperature * 100.0f);
    
    // Altitude in cm (meters * 100)
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
        // Convert to 0.1µT by multiplying by 10 and cast to int16
        currentPacket.magX = static_cast<int16_t>(magData[0]); 
        currentPacket.magY = static_cast<int16_t>(magData[1]);
        currentPacket.magZ = static_cast<int16_t>(magData[2]);
    }
      // GPS data - Clean approach using TinyGPSPlus library with enhanced error detection
    
    // First, check if we're getting valid GPS messages at all
    static unsigned long lastCharsProcessed = 0;
    static unsigned long lastGpsCheck = 0;
    static bool gpsDataFlowing = false;
    
    if (millis() - lastGpsCheck > 5000) { // Check GPS data flow every 5 seconds
        unsigned long currentCharsProcessed = gps.charsProcessed();
        if (currentCharsProcessed > lastCharsProcessed) {
            // We're receiving some GPS data
            gpsDataFlowing = true;
            lastCharsProcessed = currentCharsProcessed;
        } else {
            // No new data since last check - possible hardware issue
            gpsDataFlowing = false;
            #if DEBUG_SENSORS
            Serial.println("<DEBUG:GPS_NO_DATA_FLOW:POSSIBLE_HARDWARE_ISSUE>");
            #endif
        }
        lastGpsCheck = millis();
    }
    
    // Get satellite count with validity check
    if (gps.satellites.isValid()) {
        currentPacket.satellites = static_cast<uint8_t>(gps.satellites.value());
        
        #if DEBUG_SENSORS
        static uint8_t lastSatCount = 0;
        if (currentPacket.satellites != lastSatCount) {
            Serial.print("<DEBUG:GPS_SATELLITE_COUNT_CHANGED:");
            Serial.print(lastSatCount);
            Serial.print("->");
            Serial.print(currentPacket.satellites);
            Serial.println(">");
            lastSatCount = currentPacket.satellites;
        }
        #endif
    } else {
        if (gpsDataFlowing) {
            // We have data flow but no valid satellite count yet
            currentPacket.satellites = 0;
        } else {
            // No data flow - could be disconnected
            currentPacket.satellites = 0;
            #if DEBUG_SENSORS
            static unsigned long lastDisconnectTime = 0;
            if (millis() - lastDisconnectTime > 30000) { // Log every 30 seconds
                Serial.println("<DEBUG:GPS_POSSIBLE_DISCONNECT:NO_DATA_RECEIVED>");
                lastDisconnectTime = millis();
            }
            #endif
        }
    }
    
    // Check location data validity and quality
    bool hasValidLocation = false;
    
    if (gps.location.isValid() && currentPacket.satellites > 3) { // Require at least 4 satellites for a reliable fix
        // Check for reasonable latitude and longitude values (avoid 0,0 bug)
        if (gps.location.lat() != 0.0 && gps.location.lng() != 0.0) {
            // Check for change in location (avoid stuck GPS bug)
            static double lastLat = 0.0, lastLon = 0.0;
            static unsigned long lastLocationChangeTime = 0;
            static bool locationChanged = false;
            
            if (gps.location.lat() != lastLat || gps.location.lng() != lastLon) {
                lastLat = gps.location.lat();
                lastLon = gps.location.lng();
                lastLocationChangeTime = millis();
                locationChanged = true;
            } else if (millis() - lastLocationChangeTime > 60000) { // No change for 1 minute
                // Possibly stuck GPS values
                #if DEBUG_SENSORS
                static unsigned long lastStuckTime = 0;
                if (millis() - lastStuckTime > 30000) { // Log every 30 seconds
                    Serial.println("<DEBUG:GPS_LOCATION_STUCK:POSITION_NOT_UPDATING>");
                    lastStuckTime = millis();
                }
                #endif
                
                // Don't mark as valid if we've been stuck for too long
                if (millis() - lastLocationChangeTime > 120000) { // 2 minutes stuck
                    locationChanged = false;
                }
            }
            
            hasValidLocation = locationChanged;
        }
    }
    
    // Process location data
    if (hasValidLocation) {
        // We have a valid fix - convert to int32 in 1e-7 degrees format
        currentPacket.latitude = static_cast<int32_t>(gps.location.lat() * 10000000.0);
        currentPacket.longitude = static_cast<int32_t>(gps.location.lng() * 10000000.0);
        
        #if DEBUG_SENSORS
        static bool firstValidFix = true;
        if (firstValidFix) {
            Serial.print("<DEBUG:GPS_FIRST_VALID_FIX:LAT=");
            Serial.print(gps.location.lat(), 6);
            Serial.print(",LON=");
            Serial.print(gps.location.lng(), 6);
            Serial.print(",SATS=");
            Serial.print(currentPacket.satellites);
            Serial.println(">");
            firstValidFix = false;
        }
        
        // Log altitude data if available
        if (gps.altitude.isValid()) {
            static unsigned long lastAltTime = 0;
            if (millis() - lastAltTime > 30000) { // Every 30 seconds
                Serial.print("<DEBUG:GPS_ALTITUDE=");
                Serial.print(gps.altitude.meters());
                Serial.println("m>");
                lastAltTime = millis();
            }
        }
        #endif
    } else {
        // No valid fix - use special placeholder values with additional diagnostics
        if (currentPacket.satellites > 0) {
            // We have satellites but no fix
            currentPacket.latitude = 10;  // Special value: satellites visible but no fix
            currentPacket.longitude = 10;
            
            #if DEBUG_SENSORS
            static unsigned long lastNoFixLog = 0;
            if (millis() - lastNoFixLog > 30000) { // Log every 30 seconds
                Serial.print("<DEBUG:GPS_HAS_SATELLITES_BUT_NO_FIX:SATS=");
                Serial.print(currentPacket.satellites);
                if (gps.location.isValid()) {
                    Serial.print(",POSSIBLE_BAD_DATA:LAT=");
                    Serial.print(gps.location.lat(), 6);
                    Serial.print(",LON=");
                    Serial.print(gps.location.lng(), 6);
                }
                Serial.println(">");
                lastNoFixLog = millis();
            }
            #endif
        } else {
            // No satellites at all
            currentPacket.latitude = 1;  // Special value: no satellites at all
            currentPacket.longitude = 1;
            
            #if DEBUG_SENSORS
            static unsigned long lastNoSatsTime = 0;
            if (millis() - lastNoSatsTime > 30000) { // Log every 30 seconds
                Serial.print("<DEBUG:GPS_NO_SATELLITES:CHARS_PROCESSED=");
                Serial.print(gps.charsProcessed());
                Serial.println(">");
                lastNoSatsTime = millis();
            }
            #endif
        }
    }
    
    // Include RTC data in packet
    currentPacket.year = rtcYear;
    currentPacket.month = rtcMonth;
    currentPacket.day = rtcDay;
    currentPacket.hour = rtcHour;
    currentPacket.minute = rtcMinute;
    currentPacket.second = rtcSecond;
      // Calculate CRC-16 for the packet using FrameCodec utility
    // We need to zero out the CRC field first to ensure consistent calculation
    currentPacket.crc16 = 0;
    
    // Calculate CRC on all fields except the CRC itself
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
    // The satellite count is populated in currentPacket.satellites by processSensorData(),
    // which correctly handles TinyGPSPlus validity and value extraction in a non-const context.
    // processSensorData sets currentPacket.satellites to 0 if gps.satellites is not valid.
    return currentPacket.satellites;
}

void SensorManager::setStatusLED(uint8_t r, uint8_t g, uint8_t b) {
    statusLed.setPixelColor(0, statusLed.Color(r, g, b));
    statusLed.show();
}

// Utility function to test GPS wiring and communications
// This should be called when debugging GPS connection issues
bool SensorManager::testGpsConnection() {
    #if DEBUG_SENSORS
    Serial.println("<DEBUG:GPS_CONNECTION_TEST_STARTED>");
    #endif
    
    // Step 1: Check TX output by toggling it and seeing if we can detect it on RX
    // (Only works if there's a loopback between TX and RX, which would indicate wrong wiring)
    pinMode(PB6, OUTPUT);  // TX pin
    pinMode(PB7, INPUT_PULLUP); // RX pin with pull-up
    
    // Get current state for reference
    int initialRxState = digitalRead(PB7);
    
    #if DEBUG_SENSORS
    Serial.print("<DEBUG:GPS_INITIAL_PIN_STATES:TX=");
    Serial.print(digitalRead(PB6));
    Serial.print(",RX=");
    Serial.print(initialRxState);
    Serial.println(">");
    #endif
    
    // Toggle TX pin rapidly and see if RX pin follows (would indicate a short or loopback)
    bool rxFollowsTx = false;
    for (int i = 0; i < 5; i++) {
        digitalWrite(PB6, HIGH);
        delayMicroseconds(100);
        if (digitalRead(PB7) != initialRxState) {
            rxFollowsTx = true;
            break;
        }
        
        digitalWrite(PB6, LOW);
        delayMicroseconds(100);
        if (digitalRead(PB7) != initialRxState) {
            rxFollowsTx = true;
            break;
        }
    }
    
    // Properly initialize UART - no need to set pins as they're in the constructor
    SerialGPS.end();
    delay(100);
    SerialGPS.begin(9600);
    
    // Step 2: Send test commands and measure response
    SerialGPS.println("$PMTK000*32"); // Test command that most GPS modules respond to
    SerialGPS.flush();
    
    // Wait for response with timeout
    unsigned long startTime = millis();
    int bytesReceived = 0;
    while (millis() - startTime < 1000 && bytesReceived < 50) {
        if (SerialGPS.available()) {
            SerialGPS.read();
            bytesReceived++;
        }
    }
    
    // Results
    #if DEBUG_SENSORS
    Serial.print("<DEBUG:GPS_CONNECTION_TEST:PIN_LOOPBACK=");
    Serial.print(rxFollowsTx ? "YES" : "NO");
    Serial.print(",BYTES_RECEIVED=");
    Serial.print(bytesReceived);
    
    if (rxFollowsTx) {
        Serial.println(",DIAGNOSIS=POSSIBLE_TX_RX_SHORT_OR_SWAP>");
    } else if (bytesReceived > 0) {
        Serial.println(",DIAGNOSIS=CONNECTION_OK>");
    } else {
        Serial.println(",DIAGNOSIS=NO_RESPONSE_CHECK_POWER_AND_WIRING>");
    }
    #endif
    
    return (bytesReceived > 0);
}
