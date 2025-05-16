#include "../include/fc/CmdParser.h"
#include "../include/utils/FrameCodec.h"

CmdParser::CmdParser(StateManager& sm) : stateManager(sm), cmdIndex(0) {
    // Initialize command buffer
    memset(cmdBuffer, 0, MAX_CMD_LENGTH);
}

void CmdParser::processChar(char c) {
    // Skip carriage return and newline characters, which might be part of line endings
    if (c == '\r' || c == '\n') {
        return;
    }
    
    // Add debug for first character received
    if (cmdIndex == 0 && c == '<') {
        char buffer[64];
        FrameCodec::formatDebug(buffer, sizeof(buffer), "CMD_START");
        Serial.println(buffer);
    }
    
    // Check for end of command
    if (c == '>') {
        // Process complete command
        if (cmdIndex > 0 && cmdBuffer[0] == '<') {
            cmdBuffer[cmdIndex] = 0; // Null terminate
            char buffer[64];
            FrameCodec::formatDebug(buffer, sizeof(buffer), cmdBuffer);
            Serial.println(buffer);
            parseAndExecute();
        } else {
            char buffer[64];
            FrameCodec::formatDebug(buffer, sizeof(buffer), "INVALID_CMD_FORMAT");
            Serial.println(buffer);
        }
        
        // Reset buffer
        cmdIndex = 0;
        memset(cmdBuffer, 0, MAX_CMD_LENGTH);
    } 
    // Store character if buffer not full
    else if (cmdIndex < MAX_CMD_LENGTH - 1) {
        cmdBuffer[cmdIndex++] = c;
    }
}

bool CmdParser::parseAndExecute() {
    // Check for valid command format: <CMD:payload>
    if (strncmp(cmdBuffer, "<CMD:", 5) != 0) {
        sendAck(false, "INVALID_FORMAT");
        return false;
    }
    
    // Extract command string
    char cmdStr[MAX_CMD_LENGTH];
    if (sscanf(cmdBuffer, "<CMD:%s", cmdStr) != 1) {
        sendAck(false, "PARSE_ERROR");
        return false;
    }
    
    // Remove trailing characters if any
    char* end = strchr(cmdStr, ':');
    if (end) *end = 0;
    end = strchr(cmdStr, '>');
    if (end) *end = 0;
    
    // Get command type
    CommandType cmd = getCommandType(cmdStr);
    
    // Process command
    bool success = stateManager.processCommand(cmd);
    
    // Send acknowledgment
    if (success) {
        sendAck(true, stateManager.getStateString());
    } else {
        sendAck(false, "DENIED");
    }
    
    return success;
}

CommandType CmdParser::getCommandType(const char* cmdStr) {
    if (strcmp(cmdStr, "DISARM") == 0) return CMD_DISARM;
    if (strcmp(cmdStr, "ARM") == 0) return CMD_ARM;
    if (strcmp(cmdStr, "ENTER_TEST") == 0) return CMD_ENTER_TEST;
    if (strcmp(cmdStr, "ENTER_RECOVERY") == 0) return CMD_ENTER_RECOVERY;
    if (strcmp(cmdStr, "TEST") == 0) return CMD_TEST;
    if (strcmp(cmdStr, "QUERY") == 0) return CMD_QUERY;
    if (strcmp(cmdStr, "FIND_ME") == 0) return CMD_FIND_ME;
    if (strncmp(cmdStr, "CONTROL", 7) == 0) return CMD_CONTROL;
    
    // Default
    return CMD_QUERY;
}

bool CmdParser::isCommandAllowed(const char* cmdStr) {
    return stateManager.isCommandAllowed(getCommandType(cmdStr));
}

void CmdParser::sendAck(bool success, const char* message) {
    char ackBuffer[64];
    FrameCodec::formatCmdAck(ackBuffer, sizeof(ackBuffer), success, message);
    Serial.println(ackBuffer);
}
