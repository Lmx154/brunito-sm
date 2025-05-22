#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <RTClib.h>
#include <Adafruit_BMP280.h>
#include <BMI088.h>
#include <BMM150.h> // Seeed Studio BMM150 library
#include <Adafruit_NeoPixel.h>

// Set to 0 to disable sensor debug output, 1 to enable
#define DEBUG_SENSORS 0  // Disabled for production use

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
    uint8_t year;        // Last two digits of year (e.g., 25 for 2025)
    uint8_t month;       // Month (1-12)
    uint8_t day;         // Day (1-31)
    uint8_t hour;        // Hour (0-23)
    uint8_t minute;      // Minute (0-59)
    uint8_t second;      // Second (0-59)
    uint8_t satellites;  // Number of GPS satellites
    int16_t temperature; // Temperature in °C * 100
    uint16_t crc16;      // CRC-16 checksum
} __attribute__((packed)) SensorPacket;

// Using Seeed Studio BMM150 library, no custom structure needed

// Class to manage sensors
class SensorManager {
private:    // Hardware drivers
    TwoWire *i2c;
    Bmi088Accel *accel;
    Bmi088Gyro *gyro;
    BMM150 mag; // Seeed Studio BMM150 library
    Adafruit_BMP280 baro;
    RTC_DS3231 rtc;
    TinyGPSPlus gps;
    Adafruit_NeoPixel statusLed;
      // Raw sensor data
    float accelData[3];  // X, Y, Z in m/s^2
    float gyroData[3];   // X, Y, Z in rad/s
    float magData[3];    // X, Y, Z magnetometer data
    float temperature;   // in °C
    float pressure;      // in Pa
    float altitude;      // in m
    
    // RTC data
    uint8_t rtcYear;     // Last two digits of year
    uint8_t rtcMonth;    // Month (1-12)
    uint8_t rtcDay;      // Day (1-31)
    uint8_t rtcHour;     // Hour (0-23)
    uint8_t rtcMinute;   // Minute (0-59)
    uint8_t rtcSecond;   // Second (0-59)
      // Timing variables
    unsigned long lastSampleTime;
    unsigned long lastFusionTime;
    unsigned long lastStreamTime;
    unsigned long ledOnTime;         // Time when LED was turned on
    
    // Packet variables
    uint16_t packetCounter;
    SensorPacket currentPacket;
    
    // Private methods
    void readAccelGyro();
    void readMagnetometer();
    void readBarometer();
    void updateTime();
    void processSensorData();
    
public:
    SensorManager();
    ~SensorManager(); // Destructor to clean up dynamic memory
      // Initialize all sensors
    bool begin();
    
    // Initialize with detailed diagnostics - returns error code or 0 for success
    int beginWithDiagnostics();
      // Main update loop - call as frequently as possible
    void update();
    
    // Update sensors with diagnostics - returns true if successful, false if failures detected
    bool updateWithDiagnostics();
    
    // Try to read from Serial GPS (non-blocking)
    void processGPS();
    
    // Get the latest binary packet
    const SensorPacket& getPacket() const;
      // Check if a new packet is ready to stream
    bool isPacketReady();
      // Get the number of satellites currently tracked by GPS
    uint8_t getGpsSatelliteCount() const;
    
    // Set the RGB status LED color
    void setStatusLED(uint8_t r, uint8_t g, uint8_t b);
    
    // Utility function to test GPS connection and diagnose wiring issues
    bool testGpsConnection();
};

#endif // SENSORS_H
