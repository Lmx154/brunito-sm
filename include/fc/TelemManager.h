#ifndef TELEM_MANAGER_H
#define TELEM_MANAGER_H

#include <Arduino.h>
#include "../navc/Sensors.h" // For SensorPacket structure

// Telemetry states
enum TelemState {
    TELEM_OFF,
    TELEM_STANDARD, // Default rate (20 Hz)
    TELEM_LOW_BW    // Low-bandwidth mode (4 Hz)
};

// Telemetry rates
#define TELEM_RATE_STANDARD 20  // 20 Hz in standard mode
#define TELEM_RATE_LOW_BW   4   // 4 Hz in low-bandwidth mode
#define RSSI_THRESHOLD     -110 // dBm threshold for low-bandwidth mode

class TelemManager {
private:
    // Telemetry state
    TelemState state;
    unsigned long lastTelemetrySent;
    unsigned int telemetryRate; // Current rate in Hz
    
    // Helper functions
    unsigned long calculateTelemetryInterval();
    
public:
    TelemManager();
    
    // Telemetry control
    void startTelemetry();
    void stopTelemetry();
    bool isTelemetryActive() const;
    
    // Rate management
    void adjustTelemRate(int16_t rssi);
    unsigned int getTelemetryRate() const;
    TelemState getTelemState() const;
    
    // Format telemetry from binary packet to ASCII frame
    void formatTelem(char* buffer, size_t bufferSize, const SensorPacket& packet);
    
    // Main update function - should be called in every loop
    // Returns true if telemetry should be sent
    bool update();
};

#endif // TELEM_MANAGER_H
