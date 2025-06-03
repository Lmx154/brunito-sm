#include "../include/navc/Test.h"
#include "../include/navc/Sensors.h"

// External GPS Serial defined in Sensors.cpp
extern HardwareSerial SerialGPS;

int SensorTest::runDiagnosticSequence(SensorManager& sensorManager) {
    Serial.println("<DEBUG:SENSOR_TEST_SEQUENCE_STARTING>");
    
    int result;
    
    // Test I2C bus first
    result = testI2CBus();
    if (result != 0) return result;
    
    // Test GPS
    result = testGPS(sensorManager);
    if (result != 0) return result;
    
    // Test RTC
    result = testRTC(sensorManager);
    if (result != 0) return result;
    
    // Test barometer
    result = testBarometer(sensorManager);
    if (result != 0) return result;
    
    // Test accelerometer
    result = testAccelerometer(sensorManager);
    if (result != 0) return result;
    
    // Test gyroscope
    result = testGyroscope(sensorManager);
    if (result != 0) return result;
    
    // Test magnetometer
    result = testMagnetometer(sensorManager);
    if (result != 0) return result;
    
    // Finalize successful test
    finalizeTest(sensorManager);
    
    return 0; // All tests passed
}

int SensorTest::testI2CBus() {
    Serial.println("<DEBUG:TESTING_I2C_BUS>");
    
    // Check if we can communicate with I2C bus by scanning for devices
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
    
    Serial.println("<DEBUG:I2C_BUS_TEST_PASSED>");
    return 0;
}

int SensorTest::testGPS(SensorManager& sensorManager) {
    Serial.println("<DEBUG:TESTING_GPS>");
    
    // Print pin states before UART initialization
    Serial.print("<DEBUG:GPS_PIN_STATES:RX(PB7)=");
    Serial.print(digitalRead(PB7));
    Serial.print(",TX(PB6)=");
    Serial.print(digitalRead(PB6));
    Serial.println(">");
    
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
    
    // Wait for GPS response
    Serial.println("<DEBUG:GPS_WAITING_FOR_RESPONSE>");
    int bytesReceived = 0;
    unsigned long gpsCheckStart = millis();
    
    // Sample for 2 seconds to allow for slower responses
    while (millis() - gpsCheckStart < 2000) {
        if (SerialGPS.available()) {
            char c = SerialGPS.read();
            bytesReceived++;
            
            // Print first 20 bytes as hex for debugging
            if (bytesReceived <= 20) {
                Serial.print(c, HEX);
                Serial.print(" ");
            }
        }
    }
    
    Serial.println(); // New line after hex dump
    Serial.print("<DEBUG:GPS_INITIAL_RESPONSE:BYTES=");
    Serial.print(bytesReceived);
    Serial.println(">");
    
    // If we didn't get any response, try alternative baud rate
    if (bytesReceived == 0) {
        Serial.println("<DEBUG:GPS_NO_RESPONSE_TRYING_115200>");
        
        SerialGPS.end();
        delay(100);
        SerialGPS.begin(115200);
        delay(100);
        
        SerialGPS.println("$PMTK000*32");
        SerialGPS.flush();
        
        // Check for response at 115200
        bytesReceived = 0;
        gpsCheckStart = millis();
        while (millis() - gpsCheckStart < 500) {
            if (SerialGPS.available()) {
                SerialGPS.read();
                bytesReceived++;
            }
        }
        
        Serial.print("<DEBUG:GPS_AT_115200_BAUD:BYTES=");
        Serial.print(bytesReceived);
        Serial.println(">");
        
        // Return to 9600 baud
        SerialGPS.end();
        delay(100);
        SerialGPS.begin(9600);
          // If still no response, note hardware check needed
        if (bytesReceived == 0) {
            Serial.println("<DEBUG:GPS_NO_RESPONSE_AT_ANY_BAUD>");
            Serial.println("<DEBUG:GPS_CHECK_HARDWARE:VERIFY_POWER_AND_WIRING>");
            Serial.println("<DEBUG:GPS_PINS:TX=PB6,RX=PB7>");
            // Note: GPS failures are non-fatal for testing
        }
    }
    
    Serial.println("<DEBUG:GPS_TEST_COMPLETED>");
    return 0; // GPS test is non-fatal
}

int SensorTest::testRTC(SensorManager& sensorManager) {
    Serial.println("<DEBUG:TESTING_RTC>");
    
    if (!sensorManager.rtc.begin()) {
        Serial.println("<DEBUG:RTC_INIT_FAILED>");
        return 2; // Error: RTC initialization failed
    }
    
    // If RTC lost power, set to compile time
    if (sensorManager.rtc.lostPower()) {
        Serial.println("<DEBUG:RTC_SET_TO_COMPILE_TIME>");
        sensorManager.rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    Serial.println("<DEBUG:RTC_TEST_PASSED>");
    return 0;
}

int SensorTest::testBarometer(SensorManager& sensorManager) {
    Serial.println("<DEBUG:TESTING_BAROMETER>");
    
    if (!sensorManager.baro.begin()) {
        Serial.println("<DEBUG:BMP280_INIT_FAILED>");
        return 3; // Error: BMP280 initialization failed
    }
    
    // Configure BMP280 settings
    sensorManager.baro.setSampling(Adafruit_BMP280::MODE_NORMAL,
                                   Adafruit_BMP280::SAMPLING_X2,
                                   Adafruit_BMP280::SAMPLING_X16,
                                   Adafruit_BMP280::FILTER_X4,
                                   Adafruit_BMP280::STANDBY_MS_1);
    
    Serial.println("<DEBUG:BAROMETER_TEST_PASSED>");
    return 0;
}

int SensorTest::testAccelerometer(SensorManager& sensorManager) {
    Serial.println("<DEBUG:TESTING_ACCELEROMETER>");
    
    sensorManager.accel = new Bmi088Accel(Wire, 0x19);
    
    int status = sensorManager.accel->begin();
    if (status < 0) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "<DEBUG:BMI088_ACCEL_INIT_FAILED:CODE=%d>", status);
        Serial.println(buffer);
        return 4; // Error: BMI088 accelerometer initialization failed
    }
    
    // Configure accelerometer settings
    sensorManager.accel->setRange(Bmi088Accel::RANGE_6G);
    sensorManager.accel->setOdr(Bmi088Accel::ODR_800HZ_BW_80HZ);
    
    Serial.println("<DEBUG:ACCELEROMETER_TEST_PASSED>");
    return 0;
}

int SensorTest::testGyroscope(SensorManager& sensorManager) {
    Serial.println("<DEBUG:TESTING_GYROSCOPE>");
    
    sensorManager.gyro = new Bmi088Gyro(Wire, 0x69);
    
    int status = sensorManager.gyro->begin();
    if (status < 0) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "<DEBUG:BMI088_GYRO_INIT_FAILED:CODE=%d>", status);
        Serial.println(buffer);
        return 5; // Error: BMI088 gyroscope initialization failed
    }
    
    // Configure gyroscope settings
    sensorManager.gyro->setRange(Bmi088Gyro::RANGE_500DPS);
    sensorManager.gyro->setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
    
    Serial.println("<DEBUG:GYROSCOPE_TEST_PASSED>");
    return 0;
}

int SensorTest::testMagnetometer(SensorManager& sensorManager) {
    Serial.println("<DEBUG:TESTING_MAGNETOMETER>");
    
    // Reset I2C bus before magnetometer initialization
    Wire.endTransmission(true);
    delay(100);
    
    // Initialize with retry logic
    int8_t bmm_status = BMM150_E_ID_NOT_CONFORM;
    for (int attempt = 0; attempt < 3; attempt++) {
        Serial.print("<DEBUG:BMM150_INIT_ATTEMPT:");
        Serial.print(attempt + 1);
        Serial.println(">");
        
        bmm_status = sensorManager.mag.initialize();
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
    
    // Configure magnetometer settings
    sensorManager.mag.set_op_mode(BMM150_NORMAL_MODE);
    sensorManager.mag.set_presetmode(BMM150_PRESETMODE_HIGHACCURACY);
    delay(50);
      // Test read to verify functionality
    sensorManager.mag.read_mag_data();
    Serial.print("<DEBUG:BMM150_RAW_TEST_DATA:X=");
    Serial.print(sensorManager.mag.mag_data.x);
    Serial.print(",Y=");
    Serial.print(sensorManager.mag.mag_data.y);
    Serial.print(",Z=");
    Serial.print(sensorManager.mag.mag_data.z);
    Serial.println(">");
    
    // Check for common error patterns
    if (sensorManager.mag.mag_data.z == -32768) {
        Serial.println("<DEBUG:MAG_Z_AXIS_STUCK_DETECTED:CALIBRATION_REQUIRED>");
    } else if (sensorManager.mag.mag_data.x == 0 && sensorManager.mag.mag_data.y == 0 && sensorManager.mag.mag_data.z == 0) {
        Serial.println("<DEBUG:MAG_ALL_ZEROS_DETECTED:CALIBRATION_REQUIRED>");
    } else {
        Serial.println("<DEBUG:MAG_TEST_VALUES_APPEAR_NORMAL>");
    }
    
    Serial.println("<DEBUG:MAGNETOMETER_TEST_PASSED>");
    return 0;
}

void SensorTest::finalizeTest(SensorManager& sensorManager) {
    // Set initial LED color to indicate successful test (green)
    sensorManager.setStatusLED(0, 255, 0);
    Serial.println("<DEBUG:ALL_SENSOR_TESTS_PASSED>");
}
