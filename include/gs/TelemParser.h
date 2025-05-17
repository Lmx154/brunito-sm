#ifndef TELEM_PARSER_H
#define TELEM_PARSER_H

#include <Arduino.h>

class TelemParser {
private:
    // CSV header flag
    bool headerSent;
    
    // Helper method to parse telemetry frame
    bool parseTelemetryFrame(const char* frame, uint16_t& packetId, uint32_t& timestamp,
                          int32_t& alt, int16_t& accelX, int16_t& accelY, int16_t& accelZ,
                          int16_t& gyroX, int16_t& gyroY, int16_t& gyroZ,
                          int16_t& magX, int16_t& magY, int16_t& magZ,
                          int32_t& lat, int32_t& lon);
    
    // Helper method to parse recovery telemetry frame
    bool parseRecoveryFrame(const char* frame, uint32_t& timestamp, 
                         int32_t& lat, int32_t& lon, int32_t& alt);

public:
    TelemParser();
    
    // Process telemetry frame and output CSV
    void processTelemetryFrame(const char* frame);
    
    // Reset parser state
    void reset();
};

#endif // TELEM_PARSER_H
