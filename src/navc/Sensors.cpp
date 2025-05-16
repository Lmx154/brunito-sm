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
    packetCounter(0)
{
    // Initialize structs and arrays to zeros    memset(accelData, 0, sizeof(accelData));
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
    // Initialize I2C bus
    Wire.begin();
    Wire.setClock(400000); // 400 kHz
    
    // Serial2 for UART communication with FC is initialized in NAVC.cpp
    
    // Initialize Serial1 for GPS
    // STM32 syntax: Serial1.begin(baudrate, config, rx_pin, tx_pin);
    Serial1.begin(9600);
    Serial1.setRx(PB7);
    Serial1.setTx(PB6);
    
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
    gyro->setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ); // Using a valid ODR value from the library
    
    // Setup BMM150 magnetometer interface
    magUserData.dev_addr = 0x13; // BMM150 I2C address
    magUserData.intf_ptr = nullptr;
    
    // Initialize BMM150 magnetometer
    mag.intf_ptr = &magUserData;
    mag.read = bmm150_i2c_read;
    mag.write = bmm150_i2c_write;
    mag.delay_us = bmm150_delay_us;    mag.intf = BMM150_I2C_INTF;
    
    // Initialize BMM150 magnetometer
    status = bmm150_init(&mag);
    if (status != BMM150_OK) {
        #if DEBUG_SENSORS
        Serial.print("<DEBUG:BMM150_INIT_FAILED:");
        Serial.print(status);
        Serial.println(">");
        #endif
        return false;
    }
    
    // Configure magnetometer settings for high accuracy
    struct bmm150_settings settings;
    settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
    status = bmm150_set_presetmode(&settings, &mag);
    if (status != BMM150_OK) {
        #if DEBUG_SENSORS
        Serial.println("<DEBUG:BMM150_CONFIG_FAILED>");
        #endif
        return false;
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

void SensorManager::processGPS() {
    // Read all available bytes from GPS module
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }
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
    // Read magnetometer data in µT
    int8_t status = bmm150_read_mag_data(&magData, &mag);
    #if DEBUG_SENSORS
    if (status != BMM150_OK) {
        Serial.print("<DEBUG:BMM150_READ_FAILED:");
        Serial.print(status);
        Serial.println(">");
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
    
    // Could use RTC time for packet timestamp if needed
    // We'll use millis() for simplicity in this implementation
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
    currentPacket.gyroZ = static_cast<int16_t>(gyroData[2] * 57.2958f * 100.0f);
    
    // Magnetometer in 0.1 µT (micro-Tesla * 10)
    currentPacket.magX = static_cast<int16_t>(magData.x * 10.0f);
    currentPacket.magY = static_cast<int16_t>(magData.y * 10.0f);
    currentPacket.magZ = static_cast<int16_t>(magData.z * 10.0f);
    
    // GPS location in 1e-7 degrees
    if (gps.location.isValid()) {
        currentPacket.latitude = static_cast<int32_t>(gps.location.lat() * 10000000.0);
        currentPacket.longitude = static_cast<int32_t>(gps.location.lng() * 10000000.0);
    }
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

void SensorManager::setStatusLED(uint8_t r, uint8_t g, uint8_t b) {
    statusLed.setPixelColor(0, statusLed.Color(r, g, b));
    statusLed.show();
}

// We're now using FrameCodec::calculateSensorPacketCRC instead of this local implementation
