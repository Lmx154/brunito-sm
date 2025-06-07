/**
 * FC.cpp - Flight Controller for Brunito Project (FreeRTOS Version)
 * 
 * This file contains the Flight Controller (FC) module implementation
 * using STM32duino FreeRTOS for task management and synchronization.
 */

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "../include/utils/Heartbeat.h"
#include "../include/utils/FrameCodec.h"
#include "../include/utils/LoraManager.h"
#include "../include/config/lora.h"
#include "../include/fc/State.h"
#include "../include/fc/CmdParser.h"
#include "../include/fc/UartManager.h"
#include "../include/fc/BackgroundTask.h"
#include "../include/navc/Sensors.h"

// FreeRTOS Task Handles
TaskHandle_t commandTaskHandle = NULL;
TaskHandle_t statusTaskHandle = NULL;
TaskHandle_t uartTaskHandle = NULL;
TaskHandle_t loraTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
TaskHandle_t heartbeatTaskHandle = NULL;

// FreeRTOS Queues
QueueHandle_t commandQueue = NULL;
QueueHandle_t telemetryQueue = NULL;
QueueHandle_t loraRxQueue = NULL;

// FreeRTOS Semaphores/Mutexes
SemaphoreHandle_t stateMutex = NULL;
SemaphoreHandle_t uartMutex = NULL;
SemaphoreHandle_t loraMutex = NULL;

// Global objects
HeartbeatManager heartbeat(PC13);
StateManager stateManager;
CmdParser cmdParser(stateManager);
LoraManager loraManager;
UartManager uartManager;

// Task timing configurations (in ticks)
const TickType_t COMMAND_TASK_DELAY = pdMS_TO_TICKS(10);      // 10ms (100Hz)
const TickType_t STATUS_TASK_DELAY = pdMS_TO_TICKS(1000);     // 1s
const TickType_t UART_TASK_DELAY = pdMS_TO_TICKS(1);          // 1ms (1kHz)
const TickType_t LORA_TASK_DELAY = pdMS_TO_TICKS(5);          // 5ms (200Hz)
const TickType_t TELEMETRY_TASK_DELAY = pdMS_TO_TICKS(50);    // 50ms (20Hz max)
const TickType_t HEARTBEAT_TASK_DELAY = pdMS_TO_TICKS(10);    // 10ms (100Hz)

// Task stack sizes (in words, not bytes)
const uint16_t COMMAND_TASK_STACK = 2048;
const uint16_t STATUS_TASK_STACK = 1024;
const uint16_t UART_TASK_STACK = 2048;
const uint16_t LORA_TASK_STACK = 2048;
const uint16_t TELEMETRY_TASK_STACK = 1536;
const uint16_t HEARTBEAT_TASK_STACK = 512;

// Task priorities (higher number = higher priority)
const UBaseType_t UART_TASK_PRIORITY = 4;      // Highest - time critical
const UBaseType_t LORA_TASK_PRIORITY = 3;      // High - communication
const UBaseType_t COMMAND_TASK_PRIORITY = 3;   // High - command processing
const UBaseType_t TELEMETRY_TASK_PRIORITY = 2; // Medium - data processing
const UBaseType_t STATUS_TASK_PRIORITY = 1;    // Low - periodic updates
const UBaseType_t HEARTBEAT_TASK_PRIORITY = 1; // Low - LED updates

// Queue sizes
const uint16_t COMMAND_QUEUE_SIZE = 10;
const uint16_t TELEMETRY_QUEUE_SIZE = 20;
const uint16_t LORA_RX_QUEUE_SIZE = 10;

// Telemetry control variables (protected by semaphore)
uint8_t telemRate = 10;
unsigned long telemPeriodMs = 100;

// Command structure for queue
struct CommandMessage {
    char command[MAX_CMD_LENGTH];
    bool fromLoRa;
};

// Telemetry structure for queue
struct TelemetryMessage {
    SensorPacket packet;
    bool gpsOnly;
};

// Forward declarations
void commandTask(void *pvParameters);
void statusTask(void *pvParameters);
void uartTask(void *pvParameters);
void loraTask(void *pvParameters);
void telemetryTask(void *pvParameters);
void heartbeatTask(void *pvParameters);
void handleLoraPacket(LoraPacket* packet);
String formatTelem(const SensorPacket& packet, bool gpsOnly);
void startTelemetry(uint8_t hz = 10);
void adjustTelemRate();
void reinitializeUart();
void reportLoraLinkStatus();
void trySendLoraSettings();

/**
 * Command processing task
 * Handles commands from USB-CDC and command queue
 */
void commandTask(void *pvParameters) {
    static String cdcCmdBuffer;
    static bool cdcCmdInProgress = false;
    CommandMessage cmdMsg;
    
    for (;;) {
        // Process USB-CDC serial commands
        while (Serial.available() > 0) {
            char c = Serial.read();
            
            if (c == '<') {
                cdcCmdBuffer = c;
                cdcCmdInProgress = true;
            } else if (c == '>' && cdcCmdInProgress) {
                cdcCmdBuffer += c;
                cdcCmdInProgress = false;
                
                // Process special commands
                if (cdcCmdBuffer.equals("<CMD:LORA_RESET_STATS>")) {
                    if (xSemaphoreTake(loraMutex, portMAX_DELAY) == pdTRUE) {
                        if (loraManager.isInitialized()) {
                            loraManager.resetStats();
                            char buffer[64];
                            FrameCodec::formatDebug(buffer, sizeof(buffer), "LORA_STATS_RESET");
                            Serial.println(buffer);
                            loraManager.sendPacket(LORA_TYPE_CMD, "<CMD_ACK:OK:LORA_STATS_RESET>", 30);
                        }
                        xSemaphoreGive(loraMutex);
                    }
                } else if (cdcCmdBuffer.equals("<CMD:LORA_STATS>")) {
                    reportLoraLinkStatus();
                } else if (cdcCmdBuffer.startsWith("<CMD:")) {
                    // Add to command queue
                    CommandMessage msg;
                    strncpy(msg.command, cdcCmdBuffer.c_str(), MAX_CMD_LENGTH - 1);
                    msg.command[MAX_CMD_LENGTH - 1] = '\0';
                    msg.fromLoRa = false;
                    
                    if (xQueueSend(commandQueue, &msg, 0) != pdTRUE) {
                        Serial.println("<DEBUG:COMMAND_QUEUE_FULL>");
                    }
                    
                    // Also forward to GS via LoRa
                    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        if (loraManager.isInitialized()) {
                            loraManager.sendPacket(LORA_TYPE_CMD, cdcCmdBuffer.c_str(), cdcCmdBuffer.length());
                        }
                        xSemaphoreGive(loraMutex);
                    }
                } else {
                    // Process locally
                    for (size_t i = 0; i < cdcCmdBuffer.length(); i++) {
                        cmdParser.processChar(cdcCmdBuffer[i]);
                    }
                }
            } else if (cdcCmdInProgress) {
                cdcCmdBuffer += c;
                if (cdcCmdBuffer.length() > LORA_MAX_PACKET_SIZE - 1) {
                    cdcCmdInProgress = false;
                    cdcCmdBuffer = "";
                }
            } else {
                cmdParser.processChar(c);
            }
        }
        
        // Process commands from queue
        if (xQueueReceive(commandQueue, &cmdMsg, 0) == pdTRUE) {
            // Process command through parser
            for (size_t i = 0; i < strlen(cmdMsg.command); i++) {
                cmdParser.processChar(cmdMsg.command[i]);
            }
            
            // Force LoRa queue processing for critical commands
            if (strstr(cmdMsg.command, "DISARM") || strstr(cmdMsg.command, "ENTER_RECOVERY")) {
                if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    loraManager.checkQueue();
                    loraManager.checkQueue();
                    loraManager.checkQueue();
                    xSemaphoreGive(loraMutex);
                }
            }
        }
        
        vTaskDelay(COMMAND_TASK_DELAY);
    }
}

/**
 * Status reporting task
 * Sends periodic status updates
 */
void statusTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t STATUS_PERIOD = pdMS_TO_TICKS(10000); // 10 seconds
    
    for (;;) {
        // Update state manager
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            stateManager.updateState();
            
            // Get current state for status message
            const char* stateStr = stateManager.getStateString();
            
            // Send status message
            char statusBuffer[64];
            FrameCodec::formatStatus(statusBuffer, sizeof(statusBuffer), stateStr, millis());
            Serial.println(statusBuffer);
            
            // Fire-and-forget over LoRa
            if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (loraManager.isInitialized()) {
                    loraManager.sendPacket(LORA_TYPE_STATUS, statusBuffer, strlen(statusBuffer));
                }
                xSemaphoreGive(loraMutex);
            }
            
            xSemaphoreGive(stateMutex);
        }
        
        // Wait for next period
        vTaskDelayUntil(&xLastWakeTime, STATUS_PERIOD);
    }
}

/**
 * UART communication task
 * Handles binary packet reception from NAVC
 */
void uartTask(void *pvParameters) {
    static bool lastTelemetryState = false;
    static unsigned long lastUartDebugTime = 0;
    static int uartFailCount = 0;
    static unsigned long lastNavcStatsReport = 0;
    
    for (;;) {
        // Check current state
        uint8_t currentState = STATE_IDLE;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            currentState = stateManager.getCurrentState();
            xSemaphoreGive(stateMutex);
        }
        
        bool shouldProcessTelemetry = (currentState == STATE_ARMED || 
                                       currentState == STATE_RECOVERY || 
                                       currentState == STATE_TEST);
        
        // Control telemetry streaming
        if (shouldProcessTelemetry != lastTelemetryState) {
            if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE) {
                if (shouldProcessTelemetry) {
                    uartManager.sendCommand("<START_TELEMETRY>");
                    Serial.println("<DEBUG:REQUESTING_TELEMETRY_START>");
                } else {
                    uartManager.sendCommand("<STOP_TELEMETRY>");
                    Serial.println("<DEBUG:REQUESTING_TELEMETRY_STOP>");
                }
                xSemaphoreGive(uartMutex);
            }
            lastTelemetryState = shouldProcessTelemetry;
        }
        
        // Process UART data
        UartPacketStatus status = PACKET_NONE;
        if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            status = shouldProcessTelemetry ? uartManager.processUartData() : PACKET_NONE;
            
            // If packet is ready, add to telemetry queue
            if (status == PACKET_COMPLETE && uartManager.isPacketReady()) {
                const SensorPacket& packet = uartManager.getLatestPacket();
                
                TelemetryMessage telemMsg;
                memcpy(&telemMsg.packet, &packet, sizeof(SensorPacket));
                telemMsg.gpsOnly = (currentState == STATE_RECOVERY);
                
                if (xQueueSend(telemetryQueue, &telemMsg, 0) != pdTRUE) {
                    // Queue full, drop packet
                    uartManager.markPacketProcessed();
                } else {
                    uartManager.markPacketProcessed();
                }
            }
            
            xSemaphoreGive(uartMutex);
        }
        
        // Periodic UART monitoring (every 10 seconds)
        unsigned long currentTime = millis();
        if (currentTime - lastUartDebugTime >= 10000) {
            lastUartDebugTime = currentTime;
            
            int bytesAvailable = Serial2.available();
            unsigned long timeSinceLastPacket = uartManager.getTimeSinceLastPacket();
            
            if ((uartManager.getCrcErrors() > 0 && currentTime % 30000 < 1000) ||
                (bytesAvailable == 0 && timeSinceLastPacket > 5000) ||
                (currentTime % 300000 < 1000)) {
                
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "<UART_MONITOR:RX=%lu,DROP=%lu,ERR=%lu>",
                        uartManager.getPacketsReceived(),
                        uartManager.getPacketsDropped(),
                        uartManager.getCrcErrors());
                Serial.println(buffer);
            }
            
            // Handle UART failures
            if (bytesAvailable == 0) {
                if (uartManager.getPacketsReceived() == 0 || timeSinceLastPacket > 10000) {
                    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        uartManager.sendCommand("<PING>");
                        xSemaphoreGive(uartMutex);
                    }
                    
                    uartFailCount++;
                    if (uartFailCount >= 3) {
                        reinitializeUart();
                        if (uartFailCount >= 10) {
                            uartFailCount = 5;
                        }
                    }
                } else {
                    uartFailCount = 0;
                }
            }
        }
        
        vTaskDelay(UART_TASK_DELAY);
    }
}

/**
 * LoRa communication task
 * Handles LoRa packet transmission and reception
 */
void loraTask(void *pvParameters) {
    static unsigned long lastLoraLinkReport = 0;
    static unsigned long lastBandwidthReport = 0;
    static unsigned long lastSettingsCheck = 0;
    
    for (;;) {
        if (xSemaphoreTake(loraMutex, portMAX_DELAY) == pdTRUE) {
            // Check for received packets
            loraManager.checkReceived();
            
            // Process transmit queue
            loraManager.checkQueue();
            loraManager.checkQueue(); // Process twice for better throughput
            
            // Send ping if enabled
            loraManager.sendPing();
            
            xSemaphoreGive(loraMutex);
        }
        
        // Periodic reports
        unsigned long now = millis();
        
        // LoRa link status report (every 30 seconds)
        if (now - lastLoraLinkReport >= 30000) {
            lastLoraLinkReport = now;
            reportLoraLinkStatus();
        }
        
        // Bandwidth report (every 60 seconds)
        if (now - lastBandwidthReport >= 60000) {
            lastBandwidthReport = now;
            if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                loraManager.reportBandwidthUsage();
                xSemaphoreGive(loraMutex);
            }
        }
        
        // Settings check (every 30 seconds)
        if (now - lastSettingsCheck >= 30000) {
            lastSettingsCheck = now;
            trySendLoraSettings();
        }
        
        vTaskDelay(LORA_TASK_DELAY);
    }
}

/**
 * Telemetry processing task
 * Formats and sends telemetry data
 */
void telemetryTask(void *pvParameters) {
    TelemetryMessage telemMsg;
    static unsigned long lastTelemTime = 0;
    static unsigned long lastRateAdjustTime = 0;
    
    for (;;) {
        // Process telemetry messages from queue
        if (xQueueReceive(telemetryQueue, &telemMsg, TELEMETRY_TASK_DELAY) == pdTRUE) {
            uint8_t currentState = STATE_IDLE;
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                currentState = stateManager.getCurrentState();
                xSemaphoreGive(stateMutex);
            }
            
            unsigned long now = millis();
            
            switch (currentState) {
                case STATE_ARMED:
                    // Check telemetry rate
                    if (now - lastTelemTime >= telemPeriodMs) {
                        lastTelemTime = now;
                        
                        // Check queue congestion
                        uint8_t pendingCount = 0;
                        if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                            pendingCount = loraManager.getPendingPacketCount();
                            xSemaphoreGive(loraMutex);
                        }
                        
                        // Adaptive packet dropping
                        static uint8_t skipCounter = 0;
                        bool shouldSkip = false;
                        
                        if (pendingCount == 0) {
                            skipCounter = 0;
                            shouldSkip = false;
                        } else if (pendingCount == 1) {
                            shouldSkip = (++skipCounter % 2 == 0);
                        } else if (pendingCount == 2) {
                            shouldSkip = (++skipCounter % 3 != 0);
                        } else {
                            shouldSkip = true;
                        }
                        
                        if (!shouldSkip) {
                            String telemStr = formatTelem(telemMsg.packet, false);
                            
                            // Send over LoRa
                            if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                loraManager.sendPacket(LORA_TYPE_TELEM, telemStr.c_str(), telemStr.length());
                                xSemaphoreGive(loraMutex);
                            }
                        }
                        
                        // Adjust telemetry rate periodically
                        if (now - lastRateAdjustTime > 2000) {
                            lastRateAdjustTime = now;
                            adjustTelemRate();
                        }
                    }
                    
                    // Report motion for auto-recovery
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                        float accelMag = sqrt(
                            pow((float)telemMsg.packet.accelX / 1000.0f, 2) +
                            pow((float)telemMsg.packet.accelY / 1000.0f, 2) +
                            pow((float)telemMsg.packet.accelZ / 1000.0f, 2)
                        );
                        stateManager.reportMotion(accelMag);
                        xSemaphoreGive(stateMutex);
                    }
                    break;
                    
                case STATE_RECOVERY:
                    // Recovery mode: 1Hz fixed rate
                    if (now - lastTelemTime >= 1000) {
                        lastTelemTime = now;
                        
                        String telemStr = formatTelem(telemMsg.packet, true);
                        Serial.println(telemStr);
                        
                        if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            loraManager.sendPacket(LORA_TYPE_TELEM, telemStr.c_str(), telemStr.length());
                            xSemaphoreGive(loraMutex);
                        }
                    }
                    break;
                    
                case STATE_TEST:
                    // Test mode: 0.5Hz rate
                    if (now - lastTelemTime >= 2000) {
                        lastTelemTime = now;
                        
                        String telemStr = formatTelem(telemMsg.packet, false);
                        Serial.println(telemStr);
                        
                        if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            loraManager.sendPacket(LORA_TYPE_TELEM, telemStr.c_str(), telemStr.length());
                            xSemaphoreGive(loraMutex);
                        }
                    }
                    
                    // Test mode debug output
                    char testBuffer[128];
                    snprintf(testBuffer, sizeof(testBuffer), 
                            "<TEST:ALT:%.2fm,ACCEL:%.2f,%.2f,%.2f>",
                            (float)telemMsg.packet.altitude / 100.0f,
                            (float)telemMsg.packet.accelX / 1000.0f,
                            (float)telemMsg.packet.accelY / 1000.0f,
                            (float)telemMsg.packet.accelZ / 1000.0f);
                    Serial.println(testBuffer);
                    break;
            }
        }
    }
}

/**
 * Heartbeat LED task
 * Updates LED pattern based on system state
 */
void heartbeatTask(void *pvParameters) {
    for (;;) {
        // Get current state
        uint8_t currentState = STATE_IDLE;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            currentState = stateManager.getCurrentState();
            xSemaphoreGive(stateMutex);
        }
        
        // Set LED pattern based on state
        switch (currentState) {
            case STATE_IDLE:
                heartbeat.setPattern(HB_IDLE);
                break;
            case STATE_TEST:
                heartbeat.setPattern(HB_TEST);
                break;
            case STATE_ARMED:
                heartbeat.setPattern(HB_ARMED);
                break;
            case STATE_RECOVERY:
                heartbeat.setPattern(HB_RECOVERY);
                break;
            default:
                heartbeat.setPattern(HB_ERROR);
        }
        
        // Update heartbeat
        heartbeat.update();
        
        vTaskDelay(HEARTBEAT_TASK_DELAY);
    }
}

/**
 * Helper functions
 */
void handleLoraPacket(LoraPacket* packet) {
    if (packet->type == LORA_TYPE_CMD) {
        // Add command to queue
        CommandMessage msg;
        memcpy(msg.command, packet->data, packet->len);
        msg.command[packet->len] = '\0';
        msg.fromLoRa = true;
        
        if (xQueueSend(commandQueue, &msg, 0) != pdTRUE) {
            Serial.println("<DEBUG:COMMAND_QUEUE_FULL>");
        }
        
        // Check for critical commands
        if (strstr(msg.command, "DISARM") || strstr(msg.command, "ENTER_RECOVERY")) {
            Serial.println("<DEBUG:PRIORITY_COMMAND_RECEIVED>");
        }
    }
}

String formatTelem(const SensorPacket& packet, bool gpsOnly) {
    char buffer[250];
    
    // Format altitude
    float altitudeMeters = packet.altitude / 100.0f;
    char altStr[10];
    int altWhole = (int)altitudeMeters;
    int altFrac = (int)(fabs(altitudeMeters - altWhole) * 100);
    snprintf(altStr, sizeof(altStr), "%s%d.%02d", 
             (altitudeMeters < 0) ? "-" : "", abs(altWhole), altFrac);
    
    // Format date and time
    char datetime[22];
    snprintf(datetime, sizeof(datetime), "%02u/%02u/20%02u,%02u:%02u:%02u", 
             packet.month, packet.day, packet.year, 
             packet.hour, packet.minute, packet.second);
    
    if (gpsOnly) {
        snprintf(buffer, sizeof(buffer), "<%s,%ld,%ld,%s,%u,%d,%u>",
                 datetime,
                 packet.latitude, 
                 packet.longitude, 
                 altStr,
                 packet.satellites,
                 packet.temperature,
                 packet.tirePressure);
    } else {
        snprintf(buffer, sizeof(buffer), 
                 "<%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%u,%d,%u>",
                 datetime,
                 altStr,
                 packet.accelX, packet.accelY, packet.accelZ,
                 packet.gyroX, packet.gyroY, packet.gyroZ,
                 packet.magX, packet.magY, packet.magZ,
                 packet.latitude, packet.longitude,
                 packet.satellites,
                 packet.temperature,
                 packet.tirePressure);
    }
    
    return String(buffer);
}

void startTelemetry(uint8_t hz) {
    // Cap at 10 Hz maximum
    if (hz > 10) {
        hz = 10;
    }
    
    telemRate = hz;
    telemPeriodMs = 1000 / telemRate;
    
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "<DEBUG:TELEM_RATE_SET:%u>", telemRate);
    Serial.println(buffer);
}

void adjustTelemRate() {
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    
    if (!loraManager.isInitialized()) {
        xSemaphoreGive(loraMutex);
        return;
    }
    
    int16_t rssi = loraManager.getLastRssi();
    float lossRate = loraManager.getPacketLossRate();
    uint8_t pendingCount = loraManager.getPendingPacketCount();
    
    xSemaphoreGive(loraMutex);
    
    uint8_t newRate = telemRate;
    
    // Adjust rate based on link quality
    if (rssi < -110 || lossRate > 0.3 || pendingCount > 3) {
        newRate = 1;
    } else if (rssi < -100 || lossRate > 0.2 || pendingCount > 2) {
        newRate = 2;
    } else if (rssi < -90 || lossRate > 0.1 || pendingCount > 1) {
        newRate = 5;
    } else if (rssi < -80) {
        newRate = 8;
    } else {
        newRate = 10;
    }
    
    if (newRate != telemRate) {
        telemRate = newRate;
        telemPeriodMs = 1000 / telemRate;
        
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "<DEBUG:TELEM_RATE_ADJUSTED:%u,RSSI:%d>", telemRate, rssi);
        Serial.println(buffer);
    }
}

void reportLoraLinkStatus() {
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("<FC_LINK:DOWN,MUTEX_TIMEOUT>");
        return;
    }
    
    if (!loraManager.isInitialized()) {
        Serial.println("<FC_LINK:DOWN,NOT_INITIALIZED>");
        xSemaphoreGive(loraMutex);
        return;
    }
    
    // Get statistics
    uint16_t sent = loraManager.getPacketsSent();
    uint16_t received = loraManager.getPacketsReceived();
    uint16_t lost = loraManager.getPacketsLost();
    float lossRate = (received > 0) ? loraManager.getPacketLossRate() * 100.0f : 0.0f;
    int16_t rssi = loraManager.getLastRssi();
    float snr = loraManager.getLastSnr();
    uint32_t timeSinceLastRx = loraManager.getTimeSinceLastRx();
    uint32_t statsDuration = loraManager.getStatsDuration();
    bool isConnected = loraManager.isConnected();
    
    xSemaphoreGive(loraMutex);
    
    // Determine link quality
    String linkQuality;
    if (!isConnected) {
        linkQuality = "DOWN";
    } else if (rssi > -90) {
        linkQuality = "EXCELLENT";
    } else if (rssi > -100) {
        linkQuality = "GOOD";
    } else if (rssi > -110) {
        linkQuality = "FAIR";
    } else {
        linkQuality = "POOR";
    }
    
    // Format time string
    char timeStr[16];
    uint32_t seconds = timeSinceLastRx / 1000;
    uint8_t hours = seconds / 3600;
    seconds %= 3600;
    uint8_t minutes = seconds / 60;
    seconds %= 60;
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hours, minutes, (uint8_t)seconds);
    
    // Format loss rate
    char lossRateStr[16];
    if (sent > 0) {
        snprintf(lossRateStr, sizeof(lossRateStr), "%.1f", lossRate);
    } else {
        strcpy(lossRateStr, "0.0");
    }
    
    // Create report
    char buffer[128];
    if (received > 0 && snr != 0) {
        snprintf(buffer, sizeof(buffer), 
                "<FC_LINK:%s,RSSI:%d,SNR:%.1f,PKT_SENT:%u,PKT_RECV:%u,LOSS:%s%%%%,LAST_RX:%s>", 
                linkQuality.c_str(), rssi, snr, sent, received, lossRateStr, timeStr);
    } else {
        snprintf(buffer, sizeof(buffer), 
                "<FC_LINK:%s,RSSI:%d,PKT_SENT:%u,PKT_RECV:%u,LOSS:%s%%%%,LAST_RX:%s>", 
                linkQuality.c_str(), rssi, sent, received, lossRateStr, timeStr);
    }
    Serial.println(buffer);
    
    // Reset statistics every hour
    if (statsDuration > 3600000) {
        if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            loraManager.resetStats();
            xSemaphoreGive(loraMutex);
        }
        Serial.println("<DEBUG:LORA_STATS_AUTO_RESET>");
    }
}

void trySendLoraSettings() {
    static bool settingsAttempted = false;
    static uint32_t lastAttempt = 0;
    static uint8_t attemptCount = 0;
    const uint32_t RETRY_INTERVAL = 60000;
    const uint8_t MAX_ATTEMPTS = 3;
    
    uint32_t now = millis();
    
    if (settingsAttempted || attemptCount >= MAX_ATTEMPTS) {
        return;
    }
    
    if (lastAttempt > 0 && (now - lastAttempt < RETRY_INTERVAL)) {
        return;
    }
    
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (loraManager.isConnected()) {
            if (loraManager.sendSettings()) {
                Serial.println("<DEBUG:LORA_SETTINGS_SENT>");
                settingsAttempted = true;
            } else {
                Serial.println("<DEBUG:LORA_SETTINGS_SEND_FAILED>");
            }
            
            lastAttempt = now;
            attemptCount++;
        }
        xSemaphoreGive(loraMutex);
    }
}

void reinitializeUart() {
    enum ReinitState {
        UART_REINIT_START,
        UART_REINIT_END_PORT,
        UART_REINIT_BEGIN_PORT,
        UART_REINIT_SEND_PING,
        UART_REINIT_SEND_ECHO,
        UART_REINIT_COMPLETE
    };
    
    static ReinitState reinitState = UART_REINIT_START;
    static unsigned long reinitStateTime = 0;
    static bool reinitializationInProgress = false;
    
    unsigned long currentTime = millis();
    
    if (!reinitializationInProgress) {
        reinitState = UART_REINIT_START;
        reinitializationInProgress = true;
        reinitStateTime = currentTime;
        Serial.println("<DEBUG:REINITIALIZING_UART:INITIATED>");
    }
    
    switch (reinitState) {
        case UART_REINIT_START:
            pinMode(PA2, INPUT_PULLUP);
            pinMode(PA3, OUTPUT);
            Serial.println("<DEBUG:REINITIALIZING_UART:START>");
            reinitState = UART_REINIT_END_PORT;
            reinitStateTime = currentTime;
            break;
            
        case UART_REINIT_END_PORT:
            if (currentTime - reinitStateTime >= 10) {
                Serial.println("<DEBUG:REINITIALIZING_UART:CLOSING_PORT>");
                Serial2.end();
                reinitState = UART_REINIT_BEGIN_PORT;
                reinitStateTime = currentTime;
            }
            break;
            
        case UART_REINIT_BEGIN_PORT:
            if (currentTime - reinitStateTime >= 50) {
                Serial.println("<DEBUG:UART_CONFIG:BAUD=115200,RX=PA2,TX=PA3>");
                Serial2.begin(115200);
                Serial2.setRx(PA2);
                Serial2.setTx(PA3);
                reinitState = UART_REINIT_SEND_PING;
                reinitStateTime = currentTime;
            }
            break;
            
        case UART_REINIT_SEND_PING:
            if (currentTime - reinitStateTime >= 20) {
                Serial.println("<DEBUG:SENDING_UART_PING>");
                Serial2.println("<PING>");
                reinitState = UART_REINIT_SEND_ECHO;
                reinitStateTime = currentTime;
            }
            break;
            
        case UART_REINIT_SEND_ECHO:
            if (currentTime - reinitStateTime >= 20) {
                Serial.println("<DEBUG:SENDING_UART_ECHO>");
                Serial2.println("<ECHO:UART_TEST>");
                reinitState = UART_REINIT_COMPLETE;
                reinitStateTime = currentTime;
            }
            break;
            
        case UART_REINIT_COMPLETE:
            if (currentTime - reinitStateTime >= 20) {
                Serial2.flush();
                if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    uartManager.resetStats();
                    xSemaphoreGive(uartMutex);
                }
                Serial.println("<DEBUG:REINITIALIZING_UART:COMPLETE>");
                reinitializationInProgress = false;
                reinitState = UART_REINIT_START;
            }
            break;
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(921600);
    while (!Serial && millis() < 3000);
    
    // Configure hardware pins
    pinMode(PA0, OUTPUT);
    digitalWrite(PA0, LOW);
    pinMode(PA2, INPUT_PULLUP);
    pinMode(PA3, OUTPUT);
    
    // Initialize UART
    Serial.println("<DEBUG:UART_SETUP:STARTING>");
    Serial2.begin(115200);
    Serial2.setRx(PA2);
    Serial2.setTx(PA3);
    delay(100);
    
    // Clear UART buffer
    while (Serial2.available()) {
        Serial2.read();
    }
    
    // Test UART
    Serial.println("<DEBUG:SENDING_NAVC_PING>");
    Serial2.println("<PING>");
    delay(10);
    Serial2.println("<ECHO:UART_TEST>");
    Serial2.flush();
    
    // Initialize FreeRTOS primitives
    stateMutex = xSemaphoreCreateMutex();
    uartMutex = xSemaphoreCreateMutex();
    loraMutex = xSemaphoreCreateMutex();
    
    commandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(CommandMessage));
    telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_SIZE, sizeof(TelemetryMessage));
    loraRxQueue = xQueueCreate(LORA_RX_QUEUE_SIZE, sizeof(LoraPacket));
    
    if (!stateMutex || !uartMutex || !loraMutex || 
        !commandQueue || !telemetryQueue || !loraRxQueue) {
        Serial.println("<ERROR:FREERTOS_INIT_FAILED>");
        while (1) { delay(1000); }
    }
    
    // Initialize LoRa
    char buffer[64];
    FrameCodec::formatDebug(buffer, sizeof(buffer), "FC_INIT: Brunito Flight Controller v0.4 (FreeRTOS)");
    Serial.println(buffer);
    
    FrameCodec::formatDebug(buffer, sizeof(buffer), "INITIALIZING_LORA");
    Serial.println(buffer);
    
    if (loraManager.begin(LORA_FC_ADDR, LORA_GS_ADDR)) {
        FrameCodec::formatDebug(buffer, sizeof(buffer), "LORA_INIT_SUCCESS");
        Serial.println(buffer);
        delay(1000);
        loraManager.checkQueue();
        loraManager.onPacketReceived = handleLoraPacket;
    } else {
        FrameCodec::formatDebug(buffer, sizeof(buffer), "LORA_INIT_FAILED");
        Serial.println(buffer);
    }
    
    // Set connection parameters
    loraManager.setConnectionTimeout(30000);
    loraManager.enablePing(true);
    
    // Initialize BackgroundTask
    if (BackgroundTask::begin(&stateManager, &cmdParser)) {
        FrameCodec::formatDebug(buffer, sizeof(buffer), "BACKGROUND_TASK_INIT_SUCCESS");
        Serial.println(buffer);
    } else {
        FrameCodec::formatDebug(buffer, sizeof(buffer), "BACKGROUND_TASK_INIT_FAILED");
        Serial.println(buffer);
    }
    
    // Create FreeRTOS tasks
    BaseType_t result;
    
    result = xTaskCreate(commandTask, "Command", COMMAND_TASK_STACK, NULL, 
                        COMMAND_TASK_PRIORITY, &commandTaskHandle);
    if (result != pdPASS) {
        Serial.println("<ERROR:COMMAND_TASK_CREATE_FAILED>");
    }
    
    result = xTaskCreate(statusTask, "Status", STATUS_TASK_STACK, NULL, 
                        STATUS_TASK_PRIORITY, &statusTaskHandle);
    if (result != pdPASS) {
        Serial.println("<ERROR:STATUS_TASK_CREATE_FAILED>");
    }
    
    result = xTaskCreate(uartTask, "UART", UART_TASK_STACK, NULL, 
                        UART_TASK_PRIORITY, &uartTaskHandle);
    if (result != pdPASS) {
        Serial.println("<ERROR:UART_TASK_CREATE_FAILED>");
    }
    
    result = xTaskCreate(loraTask, "LoRa", LORA_TASK_STACK, NULL, 
                        LORA_TASK_PRIORITY, &loraTaskHandle);
    if (result != pdPASS) {
        Serial.println("<ERROR:LORA_TASK_CREATE_FAILED>");
    }
    
    result = xTaskCreate(telemetryTask, "Telemetry", TELEMETRY_TASK_STACK, NULL, 
                        TELEMETRY_TASK_PRIORITY, &telemetryTaskHandle);
    if (result != pdPASS) {
        Serial.println("<ERROR:TELEMETRY_TASK_CREATE_FAILED>");
    }
    
    result = xTaskCreate(heartbeatTask, "Heartbeat", HEARTBEAT_TASK_STACK, NULL, 
                        HEARTBEAT_TASK_PRIORITY, &heartbeatTaskHandle);
    if (result != pdPASS) {
        Serial.println("<ERROR:HEARTBEAT_TASK_CREATE_FAILED>");
    }
    
    FrameCodec::formatStatus(buffer, sizeof(buffer), "IDLE", millis());
    Serial.println(buffer);
    
    // Start FreeRTOS scheduler
    Serial.println("<DEBUG:STARTING_FREERTOS_SCHEDULER>");
    vTaskStartScheduler();
    
    // Should never reach here
    Serial.println("<ERROR:FREERTOS_SCHEDULER_FAILED>");
    while (1) { delay(1000); }
}

void loop() {
    // Empty - all work is done in FreeRTOS tasks
    // This should never be reached as vTaskStartScheduler() doesn't return
}