/**
 * test_command_parser_fuzzing.cpp - Fuzz Testing for CmdParser
 * 
 * Tests the CmdParser's robustness by sending 1,000 randomly generated
 * valid and invalid command frames and verifying proper error detection.
 */

#include <unity.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "../../include/fc/CmdParser.h"
#include "../../include/fc/State.h"

// Mock Arduino functions for testing
#define MOCK_SERIAL_BUFFER_SIZE 256
char mockSerialBuffer[MOCK_SERIAL_BUFFER_SIZE];
int mockSerialIndex = 0;

// Mock Arduino's Serial object
class SerialMock {
public:
    void begin(unsigned long) {}
    int available() { return 0; }
    size_t print(const char*) { return 0; }
    size_t println(const char* str) { 
        size_t len = strlen(str);
        if (mockSerialIndex + len < MOCK_SERIAL_BUFFER_SIZE - 2) {
            strcpy(mockSerialBuffer + mockSerialIndex, str);
            mockSerialIndex += len;
            mockSerialBuffer[mockSerialIndex++] = '\r';
            mockSerialBuffer[mockSerialIndex++] = '\n';
        }
        return len + 2;
    }
    size_t println() { return println(""); }
} Serial;

// Reset the mock serial buffer
void resetMockSerial() {
    memset(mockSerialBuffer, 0, MOCK_SERIAL_BUFFER_SIZE);
    mockSerialIndex = 0;
}

// Mock StateManager for testing
class MockStateManager : public StateManager {
public:
    bool processCalled = false;
    CommandType lastCommand = CMD_QUERY;
    
    MockStateManager() : StateManager() {
        processCalled = false;
    }
    
    bool processCommand(CommandType cmd) override {
        processCalled = true;
        lastCommand = cmd;
        return true;
    }
    
    void reset() {
        processCalled = false;
    }
};

// Test helper functions
static MockStateManager mockStateManager;
static CmdParser cmdParser(mockStateManager);

// Valid commands for testing
const char* VALID_COMMANDS[] = {
    "ARM", "DISARM", "ENTER_TEST", "ENTER_RECOVERY", "TEST", "QUERY", "FIND_ME"
};
const int NUM_VALID_COMMANDS = sizeof(VALID_COMMANDS) / sizeof(VALID_COMMANDS[0]);

// Random string generator
void generateRandomString(char* str, size_t len) {
    const char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:,.<>[]{}|\\/*-+";
    
    for (size_t i = 0; i < len; i++) {
        str[i] = charset[rand() % (sizeof(charset) - 1)];
    }
    str[len] = '\0';
}

// Generate valid command with optional checksums
void generateValidCommand(char* buffer, int bufferSize, bool includeChecksum = false) {
    int cmdIdx = rand() % NUM_VALID_COMMANDS;
    const char* cmd = VALID_COMMANDS[cmdIdx];
    
    if (includeChecksum) {
        // Calculate a proper checksum
        char tempCmd[64];
        sprintf(tempCmd, "<CMD:%s:", cmd);
        uint16_t checksum;
        
        // Access the calculateChecksum method (now public)
        checksum = cmdParser.calculateChecksum(tempCmd, strlen(tempCmd));
        
        snprintf(buffer, bufferSize, "<CMD:%s:%04X>", cmd, checksum);
    } else {
        snprintf(buffer, bufferSize, "<CMD:%s>", cmd);
    }
}

// Generate an invalid command
void generateInvalidCommand(char* buffer, int bufferSize) {
    int type = rand() % 5;
    
    switch (type) {
        case 0: // Missing prefix
            snprintf(buffer, bufferSize, "CMD:%s>", VALID_COMMANDS[rand() % NUM_VALID_COMMANDS]);
            break;
        case 1: // Missing suffix
            snprintf(buffer, bufferSize, "<CMD:%s", VALID_COMMANDS[rand() % NUM_VALID_COMMANDS]);
            break;
        case 2: // Invalid command name
            {
                char randomCmd[12];
                generateRandomString(randomCmd, 8);
                snprintf(buffer, bufferSize, "<CMD:%s>", randomCmd);
            }
            break;
        case 3: // Invalid parameters
            snprintf(buffer, bufferSize, "<CMD:%s:param=%d>", 
                    VALID_COMMANDS[rand() % NUM_VALID_COMMANDS], 
                    rand() % 1000);
            break;
        case 4: // Wrong checksum
            snprintf(buffer, bufferSize, "<CMD:%s:%04X>", 
                    VALID_COMMANDS[rand() % NUM_VALID_COMMANDS], 
                    rand() % 65536);
            break;
    }
}

// Test setup and teardown
void setUp(void) {
    mockStateManager.reset();
    resetMockSerial();
}

void tearDown(void) {
    // Clean up
}

// Test processing valid commands
void test_valid_commands() {
    for (int i = 0; i < NUM_VALID_COMMANDS; i++) {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "<CMD:%s>", VALID_COMMANDS[i]);
        
        mockStateManager.reset();
        bool result = cmdParser.processCommand(cmd);
        
        TEST_ASSERT_TRUE(mockStateManager.processCalled);
        
        // Let's check if the last command was processed correctly
        const char* cmdString = getCommandString(mockStateManager.lastCommand);
        if (strcmp(cmdString, "UNKNOWN") != 0) {
            TEST_ASSERT_EQUAL_STRING(VALID_COMMANDS[i], cmdString);
        }
    }
}

// Test processing invalid commands
void test_invalid_commands() {
    // Invalid format
    TEST_ASSERT_FALSE(cmdParser.processCommand("CMD:ARM>"));
    TEST_ASSERT_FALSE(cmdParser.processCommand("<CMD:ARM"));
    TEST_ASSERT_FALSE(cmdParser.processCommand("<cmd:arm>"));
    TEST_ASSERT_FALSE(cmdParser.processCommand("< CMD:ARM >"));
    TEST_ASSERT_FALSE(cmdParser.processCommand("<CMD:>"));
}

// Fuzz test with 1,000 random commands
void test_fuzz_commands() {
    const int NUM_TESTS = 1000;
    int validTests = 0;
    int invalidTests = 0;
    
    for (int i = 0; i < NUM_TESTS; i++) {
        char cmd[128];
        bool shouldBeValid = (rand() % 4 > 0); // 75% chance of valid command
        
        if (shouldBeValid) {
            generateValidCommand(cmd, sizeof(cmd), true);
            validTests++;
        } else {
            generateInvalidCommand(cmd, sizeof(cmd));
            invalidTests++;
        }
        
        mockStateManager.reset();
        bool result = cmdParser.processCommand(cmd);
        
        // Print progress every 100 tests
        if (i % 100 == 0) {
            char progress[32];
            snprintf(progress, sizeof(progress), "Fuzz test %d/%d", i, NUM_TESTS);
            TEST_MESSAGE(progress);
        }
        
        // For valid commands, verify they were processed
        if (shouldBeValid && strstr(cmd, "CMD:") && strchr(cmd, '>') != NULL) {
            TEST_ASSERT_TRUE_MESSAGE(result, cmd);
        } else {
            // Invalid commands should be rejected
            TEST_ASSERT_FALSE_MESSAGE(result, cmd);
        }
    }
    
    char summary[64];
    snprintf(summary, sizeof(summary), "Completed %d valid and %d invalid tests", validTests, invalidTests);
    TEST_MESSAGE(summary);
}

// Helper function to get command string for verification
const char* getCommandString(CommandType cmd) {
    switch (cmd) {
        case CMD_DISARM: return "DISARM";
        case CMD_ARM: return "ARM";
        case CMD_ENTER_TEST: return "ENTER_TEST";
        case CMD_ENTER_RECOVERY: return "ENTER_RECOVERY";
        case CMD_TEST: return "TEST";
        case CMD_QUERY: return "QUERY";
        case CMD_FIND_ME: return "FIND_ME";
        case CMD_CONTROL: return "CONTROL";
        default: return "UNKNOWN";
    }
}

int main(void) {
    // Seed the random number generator
    srand(time(NULL));
    
    UNITY_BEGIN();
    
    RUN_TEST(test_valid_commands);
    RUN_TEST(test_invalid_commands);
    RUN_TEST(test_fuzz_commands);
    
    return UNITY_END();
}
