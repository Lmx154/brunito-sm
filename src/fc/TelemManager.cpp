#include "../include/fc/TelemManager.h"
#include "../include/utils/FrameCodec.h"

TelemManager::TelemManager() : 
    state(TELEM_OFF),
    lastTelemetrySent(0),
    telemetryRate(TELEM_RATE_STANDARD)
{
}

void TelemManager::startTelemetry() {
    // This should only be called when system state is ARMED
    // The calling function should enforce this
    if (state == TELEM_OFF) {
        state = TELEM_STANDARD;
        telemetryRate = TELEM_RATE_STANDARD;
        lastTelemetrySent = millis();
    }
}

void TelemManager::stopTelemetry() {
    // This should be called when system state changes from ARMED to something else
    // The calling function should enforce this
    state = TELEM_OFF;
}

bool TelemManager::isTelemetryActive() const {
    return state != TELEM_OFF;
}

void TelemManager::adjustTelemRate(int16_t rssi) {
    // Only adjust if telemetry is active
    if (state == TELEM_OFF) {
        return;
    }
    
    // Adjust based on RSSI
    if (rssi < RSSI_THRESHOLD && state != TELEM_LOW_BW) {
        state = TELEM_LOW_BW;
        telemetryRate = TELEM_RATE_LOW_BW;
    } else if (rssi >= RSSI_THRESHOLD && state != TELEM_STANDARD) {
        state = TELEM_STANDARD;
        telemetryRate = TELEM_RATE_STANDARD;
    }
}

unsigned int TelemManager::getTelemetryRate() const {
    return telemetryRate;
}

TelemState TelemManager::getTelemState() const {
    return state;
}

void TelemManager::formatTelem(char* buffer, size_t bufferSize, const SensorPacket& packet) {
    // Use FrameCodec to format telemetry
    FrameCodec::formatArmedTelemetry(
        buffer, bufferSize,
        packet.packetId, packet.timestamp,
        packet.altitude, 
        packet.accelX, packet.accelY, packet.accelZ,
        packet.gyroX, packet.gyroY, packet.gyroZ,
        packet.magX, packet.magY, packet.magZ,
        packet.latitude, packet.longitude
    );
}

unsigned long TelemManager::calculateTelemetryInterval() {
    // Convert Hz to milliseconds interval
    return 1000 / telemetryRate;
}

bool TelemManager::update() {
    if (state == TELEM_OFF) {
        return false;
    }
    
    unsigned long now = millis();
    unsigned long interval = calculateTelemetryInterval();
    
    if (now - lastTelemetrySent >= interval) {
        lastTelemetrySent = now;
        return true;
    }
    
    return false;
}
