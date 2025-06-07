#include "../include/fc/ArmedLogic.h"

// Initialize static variables
unsigned long ArmedLogic::lastBeepTime = 0;

void ArmedLogic::update() {
    unsigned long currentTime = millis();
    
    // Beep every 2 seconds (2000ms)
    if (currentTime - lastBeepTime >= 2000) {
        toneMaxVolume(BUZZER_PIN, TONE_IDLE);
        delay(100); // Short 100ms beep
        noToneMaxVolume(BUZZER_PIN);
        
        lastBeepTime = currentTime;
    }
}
