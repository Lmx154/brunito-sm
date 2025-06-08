#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <RTClib.h>
#include <STM32FreeRTOS.h>
#include "Sensors.h"  // For SensorPacket definition

// SD Card configuration
#define SD_CS_PIN PA4  // Choose appropriate CS pin for your hardware
#define BUZZER_PIN PA0  // Buzzer pin (shared with FC)
#define PACKET_BUFFER_SIZE 100
#define SD_FILENAME_MAX_LEN 50
#define LED_BLINK_DURATION_MS 50
#define SD_POLL_INTERVAL_MS 2000  // Normal polling interval (reduced from 5000ms to 2000ms)
#define SD_POLL_INTERVAL_FAST_MS 500  // Fast polling when changes detected (reduced from 1000ms to 500ms)
#define SENSOR_STABILIZATION_DELAY_MS 2000  // 2 second delay (reduced from 5s) after sensor init before logging
#define BUZZER_BEEP_DURATION_MS 200  // Duration of buzzer beep when SD card missing

// Packet buffer for SD logging
typedef struct {
    char data[300];  // Increased from 250 to 300 for better buffer capacity
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

// External mutex for thread-safe Serial access
extern SemaphoreHandle_t serialMutex;
// External mutex for thread-safe SD card access
extern SemaphoreHandle_t sdMutex;

class SDLogger {
private:
    PacketBuffer packet_buffer;
    bool sd_card_present;
    bool logging_active;    bool sensors_ready;  // Track if sensors are ready for logging
    uint32_t sensor_ready_time_ms;  // When sensors became ready
    char current_log_filename[SD_FILENAME_MAX_LEN];
    uint32_t last_sd_poll_ms;
    uint32_t last_card_change_ms;  // Track when last change occurred for fast polling
    uint32_t packets_logged;
    File log_file;
    // LED feedback variables
    uint32_t led_blink_start_ms;
    bool led_blink_active;
    uint32_t led_status_update_ms;  // For periodic status LED updates
    SensorManager* sensor_manager;  // For LED access
    
    // Buzzer feedback variables
    uint32_t buzzer_beep_start_ms;
    bool buzzer_active;
    
    // RTC reference from SensorManager
    RTC_DS3231& rtc;
    
public:
    SDLogger(RTC_DS3231& rtc_ref, SensorManager* sm = nullptr);
    bool begin();    void update();
    void setSensorsReady();  // Call this when sensors are fully initialized
    void resetSensorsReady(); // Call this when sensors are reinitialized 
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
    void updateStatusLED();  // New method for status indication
    void updateBuzzer();  // New method for buzzer control
    void formatSensorPacketCSV(const SensorPacket& packet, char* buffer, size_t bufferSize);  // CSV formatter
};

#endif
