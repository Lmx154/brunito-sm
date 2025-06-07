#ifndef IDLE_LOGIC_H
#define IDLE_LOGIC_H

#include <Arduino.h>

// Forward declarations
class CmdParser;

class IdleLogic {
private:
    static bool lastButtonState;
    static unsigned long lastButtonCheck;
    static const unsigned long BUTTON_DEBOUNCE_MS = 50; // 50ms debounce time
    static const int ARM_BUTTON_PIN = PA1; // Pin A1 for external button
    
public:
    static void init();
    static void update(CmdParser& cmdParser);
};

#endif // IDLE_LOGIC_H
