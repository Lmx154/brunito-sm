#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <Arduino.h>
#include "../include/fc/State.h"

class CmdParser {
private:
    StateManager& stateManager;
    
    // Buffer for incoming commands
    static const int MAX_CMD_LENGTH = 64;
    char cmdBuffer[MAX_CMD_LENGTH];
    int cmdIndex;
    
    // Parse command from buffer and execute
    bool parseAndExecute();
    
    // Extract command type from string
    CommandType getCommandType(const char* cmdStr);
    
    // Send command acknowledgment
    void sendAck(bool success, const char* message = nullptr);
    
public:
    CmdParser(StateManager& stateManager);
    
    // Process a single character from serial
    void processChar(char c);
    
    // Check if command is allowed in current state
    bool isCommandAllowed(const char* cmdStr);
};

#endif // CMD_PARSER_H
