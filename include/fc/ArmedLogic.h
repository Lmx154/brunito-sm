#ifndef ARMED_LOGIC_H
#define ARMED_LOGIC_H

#include <Arduino.h>

// Forward declarations for buzzer functions and constants
extern void toneMaxVolume(uint8_t pin, unsigned int frequency);
extern void noToneMaxVolume(uint8_t pin);
extern const int BUZZER_PIN;
extern const int TONE_IDLE;

class ArmedLogic {
private:
    static unsigned long lastBeepTime;
    
public:
    static void update();
};

#endif
