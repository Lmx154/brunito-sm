#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <RTClib.h>
#include "Sensors.h"  // For SensorPacket definition

// SD Card configuration
#define SD_CS_PIN PA4  // Choose appropriate CS pin for your hardware
#define PACKET_BUFFER_SIZE 20
#define SD_FILENAME_MAX_LEN 50
#define LED_BLINK_DURATION_MS 50
#define SD_POLL_INTERVAL_MS 1000  // Increased from 100ms to reduce overhead
#define SENSOR_STABILIZATION_DELAY_MS 10000  // 10 second delay after sensor init before logging

// Packet buffer for SD logging
typedef struct {
    char data[250];  // Sized for full telemetry packet string
    uint32_t timestamp_ms;
} PacketEntry;

typedef struct {
    PacketEntry packets[PACKET_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} PacketBuffer;

// Forward declaration
class SensorManager;

class SDLogger {
private:
    PacketBuffer packet_buffer;
    bool sd_card_present;
    bool logging_active;
    bool sensors_ready;  // Track if sensors are ready for logging
    uint32_t sensor_ready_time_ms;  // When sensors became ready
    char current_log_filename[SD_FILENAME_MAX_LEN];
    uint32_t last_sd_poll_ms;
    uint32_t packets_logged;
    File log_file;
    
    // LED feedback variables
    uint32_t led_blink_start_ms;
    bool led_blink_active;
    SensorManager* sensor_manager;  // For LED access
    
    // RTC reference from SensorManager
    RTC_DS3231& rtc;
    
public:
    SDLogger(RTC_DS3231& rtc_ref, SensorManager* sm = nullptr);
    bool begin();
    void update();
    void setSensorsReady();  // Call this when sensors are fully initialized
    bool addPacket(const char* packet_data);
    bool addSensorPacket(const SensorPacket& packet);  // New method for direct sensor packet logging
    bool isLogging() const { return logging_active; }
    uint32_t getPacketsLogged() const { return packets_logged; }
    
private:
    void checkSDCard();
    bool initializeSDCard();
    void generateLogFilename(char* filename, size_t size);
    bool openLogFile();
    void closeLogFile();
    void processWrites();
    void updateLED();
    void formatSensorPacketCSV(const SensorPacket& packet, char* buffer, size_t bufferSize);  // CSV formatter
};

#endif
