#include "../include/fc/IdleLogic.h"
#include "../include/fc/CmdParser.h"
#include "../include/utils/FrameCodec.h"

// Initialize static variables
bool IdleLogic::lastButtonState = false;
unsigned long IdleLogic::lastButtonCheck = 0;

void IdleLogic::init() {
    // Initialize the ARM button pin
    pinMode(ARM_BUTTON_PIN, INPUT);
    lastButtonState = digitalRead(ARM_BUTTON_PIN);
    lastButtonCheck = millis();
}

void IdleLogic::update(CmdParser& cmdParser) {
    unsigned long currentTime = millis();
    
    // Debounce the button reading
    if (currentTime - lastButtonCheck < BUTTON_DEBOUNCE_MS) {
        return;
    }
    
    lastButtonCheck = currentTime;
    
    // Read the current state of the ARM button pin
    bool currentButtonState = digitalRead(ARM_BUTTON_PIN);
    
    // Check for button press (transition from LOW to HIGH)
    if (currentButtonState && !lastButtonState) {
        // Button pressed! Send ARM command
        char buffer[64];
        FrameCodec::formatDebug(buffer, sizeof(buffer), "IDLE_LOGIC:ARM_BUTTON_PRESSED");
        Serial.println(buffer);
        
        // Process the ARM command through the command parser
        // This will properly handle state transitions and acknowledgments
        const char* armCommand = "<CMD:ARM>";
        for (int i = 0; armCommand[i] != '\0'; i++) {
            cmdParser.processChar(armCommand[i]);
        }
    }
    
    // Update the last state for next comparison
    lastButtonState = currentButtonState;
}
