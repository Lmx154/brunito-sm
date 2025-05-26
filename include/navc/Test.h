#pragma once

// Test module for sensor validation and diagnostics
// Extracted from SensorManager::beginWithDiagnostics to separate concerns

class SensorManager; // Forward declaration

class SensorTest {
public:
    // Run comprehensive sensor testing and validation
    // Returns 0 on success, error code on failure
    // Sets LED to green on success, appropriate error color on failure
    static int runDiagnosticSequence(SensorManager& sensorManager);

private:
    // Individual test functions
    static int testI2CBus();
    static int testGPS(SensorManager& sensorManager);
    static int testRTC(SensorManager& sensorManager);
    static int testBarometer(SensorManager& sensorManager);
    static int testAccelerometer(SensorManager& sensorManager);
    static int testGyroscope(SensorManager& sensorManager);
    static int testMagnetometer(SensorManager& sensorManager);
    static void finalizeTest(SensorManager& sensorManager);
};
