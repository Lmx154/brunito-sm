#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <RTClib.h>
#include <Adafruit_BMP280.h>
#include <BMI088.h>
#include <bmm150.h>
#include <Adafruit_NeoPixel.h>

// Set to 1 to enable debug output
#define DEBUG_SENSORS 0

// Sensor reading and fusion rates
#define SENSOR_SAMPLE_RATE_HZ 100
#define SENSOR_FUSION_RATE_HZ 100
#define UART_STREAM_RATE_HZ 50

// Structure to hold sensor readings
typedef struct {
    uint16_t packetId;
    uint32_t timestamp;  // Milliseconds since boot
    int32_t altitude;    // Altitude in cm
    int16_t accelX;      // Acceleration in mg (milli-g)
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;       // Angular rate in 0.01 dps (degrees per second)
    int16_t gyroY;
    int16_t gyroZ;
    int16_t magX;        // Magnetic field in 0.1 µT
    int16_t magY;
    int16_t magZ;
    int32_t latitude;    // Latitude in 1e-7 degrees
    int32_t longitude;   // Longitude in 1e-7 degrees
    uint16_t crc16;      // CRC-16 checksum
} __attribute__((packed)) SensorPacket;

// BMM150 user data structure
struct bmm150_user_data {
    uint8_t dev_addr;
    void *intf_ptr;
};

// Class to manage sensors
class SensorManager {
private:
    // Hardware drivers
    TwoWire *i2c;
    Bmi088Accel *accel;
    Bmi088Gyro *gyro;
    struct bmm150_dev mag;
    bmm150_user_data magUserData;
    Adafruit_BMP280 baro;
    RTC_DS3231 rtc;
    TinyGPSPlus gps;
    Adafruit_NeoPixel statusLed;
    
    // Raw sensor data
    float accelData[3];  // X, Y, Z in m/s^2
    float gyroData[3];   // X, Y, Z in rad/s
    struct bmm150_mag_data magData;  // X, Y, Z in µT
    float temperature;   // in °C
    float pressure;      // in Pa
    float altitude;      // in m
    
    // Timing variables
    unsigned long lastSampleTime;
    unsigned long lastFusionTime;
    unsigned long lastStreamTime;
    
    // Packet variables
    uint16_t packetCounter;
    SensorPacket currentPacket;
    
    // Private methods
    void readAccelGyro();
    void readMagnetometer();
    void readBarometer();    void readGPS();
    void updateTime();
    void processSensorData();
    
public:
    SensorManager();
    ~SensorManager(); // Destructor to clean up dynamic memory
    
    // Initialize all sensors
    bool begin();
    
    // Main update loop - call as frequently as possible
    void update();
    
    // Try to read from Serial GPS (non-blocking)
    void processGPS();
    
    // Get the latest binary packet
    const SensorPacket& getPacket() const;
    
    // Check if a new packet is ready to stream
    bool isPacketReady();
    
    // Set the RGB status LED color
    void setStatusLED(uint8_t r, uint8_t g, uint8_t b);
};

#endif // SENSORS_H
