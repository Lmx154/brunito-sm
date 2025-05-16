#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <Arduino.h>
#include "../include/fc/State.h"
#include "../include/utils/FrameCodec.h"

// Parameter range definitions for validation
struct ParamRange {
    int32_t min;
    int32_t max;
};

class CmdParser {
private:
    StateManager& stateManager;
    
    // Buffer for incoming commands
    static const int MAX_CMD_LENGTH = 128;
    char cmdBuffer[MAX_CMD_LENGTH];
    int cmdIndex;
    
    // Parse command from buffer and execute
    bool parseAndExecute();
      // Extract command type from string
    CommandType getCommandType(const char* cmdStr);
    
    // Validate command parameters
    bool validateParameters(const char* cmdStr, const char* params);
    
    // Verify command checksum
    bool verifyChecksum(const char* cmd, uint16_t providedChecksum);
    
    // Send command acknowledgment
    void sendAck(bool success, const char* message = nullptr);
    
    // Parse parameters into key-value pairs
    bool parseParams(const char* paramStr, char* key, size_t keySize, int32_t* value);
    
public:
    CmdParser(StateManager& stateManager);
    
    // Process a single character from serial
    void processChar(char c);
    
    // Check if command is allowed in current state
    bool isCommandAllowed(const char* cmdStr);
    
    // Calculate command checksum (public for testing)
    uint16_t calculateChecksum(const char* cmd, size_t length);
    
    // For testing: Process a complete command string
    bool processCommand(const char* cmd);
};

#endif // CMD_PARSER_H
