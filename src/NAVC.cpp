#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include "BMI088.h"
#include <bmm150.h>
#include <RTClib.h>
#include <TinyGPSPlus.h>
#include <String.h>

// Define the pin and number of LEDs
#define NEOPIXEL_PIN PC14
#define NUM_LEDS 1

// Define other pins
const int LED_PIN = PC13;   // On-board LED
const int SD_CS_PIN = PA4;  // SD CS

// Define UART2 pins
#define UART2_RX PA3  // UART2 RX on PA3
#define UART2_TX PA2  // UART2 TX on PA2

// Initialize Serial2 as a HardwareSerial on UART2 pins
HardwareSerial Serial2(UART2_TX, UART2_RX);

// Initialize the NeoPixel strip
Adafruit_NeoPixel strip(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Use Serial1 for GPS
TinyGPSPlus gps;

// Initialize sensor objects
Adafruit_BMP280 bmp;           // BMP280 @0x77
Bmi088Accel accel(Wire, 0x19);
Bmi088Gyro gyro(Wire, 0x69);

// Command processing
String receivedCommand = "";
bool commandComplete = false;

// Define commands
const String CMD_RUN_TEST = "TEST";
const String CMD_STATUS = "STATUS";
const String CMD_ARM = "ARMED";
const String CMD_DISARM = "DISARMED";
const String CMD_TEST_MODE_ENTERED = "TEST_MODE_ENTERED";
const String CMD_TEST_MODE_EXITED = "TEST_MODE_EXITED";
const String CMD_FC_INITIALIZED = "FC_INITIALIZED";

// Define system states
enum SystemState {
  STATE_IDLE,
  STATE_TESTING,
  STATE_TEST_SUCCESS,
  STATE_TEST_FAILED_RTC,
  STATE_TEST_FAILED_BMP280,
  STATE_TEST_FAILED_ACCEL,
  STATE_TEST_FAILED_GYRO,
  STATE_TEST_FAILED_MAG,
  STATE_TEST_FAILED_SD
};

SystemState currentState = STATE_IDLE;
unsigned long testStartTime = 0;

// Bosch BMM150 initialization
struct bmm150_dev bmm150Dev;
int8_t bmm150_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
  uint8_t dev_id = *((uint8_t*)intf_ptr);
  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  Wire.endTransmission();
  Wire.requestFrom(dev_id, (uint8_t)length);
  
  uint8_t i = 0;
  while (Wire.available() && i < length) {
    reg_data[i++] = Wire.read();
  }
  
  return (i == length) ? BMM150_OK : BMM150_E_COM_FAIL;
}

int8_t bmm150_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
  uint8_t dev_id = *((uint8_t*)intf_ptr);
  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  
  for (uint32_t i = 0; i < length; i++) {
    Wire.write(reg_data[i]);
  }
  
  uint8_t error = Wire.endTransmission();
  return (error == 0) ? BMM150_OK : BMM150_E_COM_FAIL;
}

void delay_msec(uint32_t ms, void *intf_ptr) {
  delay(ms);
}

RTC_DS3231 rtc;                // DS3231 RTC

#define SEALEVELPRESSURE_HPA 1013.25F

// Function to run specific tests
void runTests() {
  Serial.println("Starting sensor test...");
  Serial2.println("<TESTING_STARTED>");  // Wrap in angle brackets for FC frame parsing
  
  currentState = STATE_TESTING;
  
  // Test RTC
  Serial.print("Testing RTC... ");
  Serial2.println("<TESTING_RTC>");
  if (!rtc.begin()) {
    Serial.println("FAILED!");
    Serial2.println("<RTC_FAILED>");
    currentState = STATE_TEST_FAILED_RTC;
    return;
  }
  DateTime now = rtc.now();
  Serial.print("OK! Time: ");
  Serial.println(String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + 
                 " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));
  Serial2.println("<RTC_OK>");
  delay(100); // Short delay to ensure transmission completes

  // Test BMP280
  Serial.print("Testing BMP280... ");
  Serial2.println("<TESTING_BMP280>");
  if (!bmp.begin(0x77)) {
    Serial.println("FAILED!");
    Serial2.println("<BMP280_FAILED>");
    currentState = STATE_TEST_FAILED_BMP280;
    return;
  }
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("OK! Temp: ");
  Serial.print(temp);
  Serial.print("°C, Pressure: ");
  Serial.print(pressure);
  Serial.print("hPa, Alt: ");
  Serial.println(altitude);
  Serial2.println("<BMP280_OK>");
  delay(100);

  // Test BMI088 Accelerometer
  Serial.print("Testing Accelerometer... ");
  Serial2.println("<TESTING_ACCEL>");
  if (accel.begin() < 0) {
    Serial.println("FAILED!");
    Serial2.println("<ACCEL_FAILED>");
    currentState = STATE_TEST_FAILED_ACCEL;
    return;
  }
  accel.readSensor();
  Serial.print("OK! X:");
  Serial.print(accel.getAccelX_mss());
  Serial.print(" Y:");
  Serial.print(accel.getAccelY_mss());
  Serial.print(" Z:");
  Serial.println(accel.getAccelZ_mss());
  Serial2.println("<ACCEL_OK>");
  delay(100);
  
  // Test BMI088 Gyroscope
  Serial.print("Testing Gyroscope... ");
  Serial2.println("<TESTING_GYRO>");
  if (gyro.begin() < 0) {
    Serial.println("FAILED!");
    Serial2.println("<GYRO_FAILED>");
    currentState = STATE_TEST_FAILED_GYRO;
    return;
  }
  gyro.readSensor();
  Serial.print("OK! X:");
  Serial.print(gyro.getGyroX_rads() * 180.0/PI);
  Serial.print(" Y:");
  Serial.print(gyro.getGyroY_rads() * 180.0/PI);
  Serial.print(" Z:");
  Serial.println(gyro.getGyroZ_rads() * 180.0/PI);
  Serial2.println("<GYRO_OK>");
  delay(100);
  
  // Test BMM150 Magnetometer
  Serial.print("Testing Magnetometer... ");
  Serial2.println("<TESTING_MAG>");
  
  // Initialize BMM150
  bmm150Dev.intf = BMM150_I2C_INTF;
  bmm150Dev.read = bmm150_i2c_read;
  bmm150Dev.write = bmm150_i2c_write;
  bmm150Dev.delay_us = delay_msec;
  bmm150Dev.intf_ptr = (void*)BMM150_DEFAULT_I2C_ADDRESS;
  
  // Initialize the BMM150 sensor
  if (bmm150_init(&bmm150Dev) != BMM150_OK) {
    Serial.println("FAILED!");
    Serial2.println("<MAG_FAILED>");
    currentState = STATE_TEST_FAILED_MAG;
    return;
  }
  
  // Set the power mode
  // Use proper settings - must create a separate settings struct
  struct bmm150_settings settings;
  
  // First read the default settings
  if (bmm150_get_sensor_settings(&settings, &bmm150Dev) != BMM150_OK) {
    Serial.println("Failed to get settings!");
    Serial2.println("<MAG_FAILED>");
    currentState = STATE_TEST_FAILED_MAG;
    return;
  }
  
  // Set power mode
  settings.pwr_mode = BMM150_POWERMODE_NORMAL;
  if (bmm150_set_op_mode(&settings, &bmm150Dev) != BMM150_OK) {
    Serial.println("Failed to set power mode!");
    Serial2.println("<MAG_FAILED>");
    currentState = STATE_TEST_FAILED_MAG;
    return;
  }
  
  // Set preset mode for regular operation
  settings.preset_mode = BMM150_PRESETMODE_REGULAR;
  if (bmm150_set_presetmode(&settings, &bmm150Dev) != BMM150_OK) {
    Serial.println("Failed to set preset mode!");
    Serial2.println("<MAG_FAILED>");
    currentState = STATE_TEST_FAILED_MAG;
    return;
  }
  
  // Allow sensor to settle
  delay(50);
  
  // Read mag data
  struct bmm150_mag_data mag_data;
  if (bmm150_read_mag_data(&mag_data, &bmm150Dev) != BMM150_OK) {
    Serial.println("Failed to read magnetometer data!");
    Serial2.println("<MAG_FAILED>");
    currentState = STATE_TEST_FAILED_MAG;
    return;
  }
  
  Serial.print("OK! X:");
  Serial.print(mag_data.x);
  Serial.print(" Y:");
  Serial.print(mag_data.y);
  Serial.print(" Z:");
  Serial.println(mag_data.z);
  Serial2.println("<MAG_OK>");
  delay(100);
  
  // Test SD card
  Serial.print("Testing SD Card... ");
  Serial2.println("<TESTING_SD>");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("FAILED!");
    Serial2.println("<SD_FAILED>");
    currentState = STATE_TEST_FAILED_SD;
    return;
  }
  Serial.println("OK!");
  Serial2.println("<SD_OK>");
  delay(100);
  
  // All tests passed
  Serial.println("All sensors tested successfully!");
  Serial2.println("<ALL_TESTS_PASSED>");
  currentState = STATE_TEST_SUCCESS;
  testStartTime = millis();
  
  // Send a direct message to confirm tests are complete
  Serial2.println("TESTS_COMPLETE");
}

// Process commands received via UART2
void processSerialCommands() {
  // Read from UART2
  while (Serial2.available() > 0) {
    char inChar = (char)Serial2.read();
    
    // Process the received character
    if (inChar == '\n' || inChar == '\r') {
      if (receivedCommand.length() > 0) {
        commandComplete = true;
      }
    } else {
      receivedCommand += inChar;
    }
  }
  
  // Process completed command
  if (commandComplete) {
    receivedCommand.trim(); // Remove any whitespace
    
    Serial.print("Command received: ");
    Serial.println(receivedCommand);
    
    // Process command
    if (receivedCommand.equalsIgnoreCase(CMD_RUN_TEST) || 
        receivedCommand.equalsIgnoreCase("TEST")) {
      Serial2.println("<STARTING_TESTS>");  // Use angle brackets for FC frame parsing
      runTests();
    } 
    else if (receivedCommand.equalsIgnoreCase(CMD_STATUS)) {
      // Report current status
      switch (currentState) {
        case STATE_IDLE:
          Serial2.println("STATUS: IDLE");
          break;
        case STATE_TESTING:
          Serial2.println("STATUS: TESTING");
          break;
        case STATE_TEST_SUCCESS:
          Serial2.println("STATUS: TEST_SUCCESS");
          break;
        case STATE_TEST_FAILED_RTC:
          Serial2.println("STATUS: TEST_FAILED_RTC");
          break;
        case STATE_TEST_FAILED_BMP280:
          Serial2.println("STATUS: TEST_FAILED_BMP280");
          break;
        case STATE_TEST_FAILED_ACCEL:
          Serial2.println("STATUS: TEST_FAILED_ACCEL");
          break;
        case STATE_TEST_FAILED_GYRO:
          Serial2.println("STATUS: TEST_FAILED_GYRO");
          break;
        case STATE_TEST_FAILED_MAG:
          Serial2.println("STATUS: TEST_FAILED_MAG");
          break;
        case STATE_TEST_FAILED_SD:
          Serial2.println("STATUS: TEST_FAILED_SD");
          break;
      }
    }
    // Handle state change notifications from FC
    else if (receivedCommand.equalsIgnoreCase(CMD_ARM)) {
      Serial.println("FC is now ARMED");
    }
    else if (receivedCommand.equalsIgnoreCase(CMD_DISARM)) {
      Serial.println("FC is now DISARMED");
    }
    else if (receivedCommand.equalsIgnoreCase(CMD_TEST_MODE_ENTERED)) {
      Serial.println("FC entered TEST mode");
    }
    else if (receivedCommand.equalsIgnoreCase(CMD_TEST_MODE_EXITED)) {
      Serial.println("FC exited TEST mode");
    }
    else if (receivedCommand.equalsIgnoreCase(CMD_FC_INITIALIZED)) {
      Serial.println("FC initialized");
    }
    else {
      Serial2.println("UNKNOWN_COMMAND: " + receivedCommand);
    }
    
    // Clear command
    receivedCommand = "";
    commandComplete = false;
  }
}

// Update LED based on current state with additional debug
void updateLED() {
  static SystemState lastState = STATE_IDLE;
  
  // Print state change for debugging
  if (currentState != lastState) {
    Serial.print("LED State changed from ");
    Serial.print(lastState);
    Serial.print(" to ");
    Serial.println(currentState);
    lastState = currentState;
  }
  
  switch (currentState) {
    case STATE_IDLE:
      // Solid blue in idle state
      strip.setPixelColor(0, strip.Color(0, 0, 100)); // Dim blue
      strip.show();
      break;
      
    case STATE_TESTING:
      // Yellow blinking during test
      static unsigned long lastBlink = 0;
      static bool ledOn = false;
      
      if (millis() - lastBlink > 200) {
        ledOn = !ledOn;
        strip.setPixelColor(0, ledOn ? strip.Color(255, 255, 0) : strip.Color(0, 0, 0));
        strip.show();
        lastBlink = millis();
      }
      break;
      
    case STATE_TEST_SUCCESS:
      // Green for 10 seconds after successful test
      if (millis() - testStartTime < 10000) {
        strip.setPixelColor(0, strip.Color(0, 255, 0));
        strip.show();
        
        // Send periodic test success message to ensure FC gets it
        static unsigned long lastSuccessMsg = 0;
        if (millis() - lastSuccessMsg > 1000) {
          Serial2.println("<TEST_SUCCESS_GREEN_LED>");
          lastSuccessMsg = millis();
        }
      } else {
        currentState = STATE_IDLE; // Return to idle state after 10 seconds
        Serial2.println("<RETURNING_TO_IDLE>");
      }
      break;
      
    case STATE_TEST_FAILED_RTC:
      // Purple for RTC failure
      strip.setPixelColor(0, strip.Color(128, 0, 128));
      strip.show();
      break;
      
    case STATE_TEST_FAILED_BMP280:
      // Red for BMP280 failure
      strip.setPixelColor(0, strip.Color(255, 0, 0));
      strip.show();
      break;
      
    case STATE_TEST_FAILED_ACCEL:
      // Orange for Accelerometer failure
      strip.setPixelColor(0, strip.Color(255, 165, 0));
      strip.show();
      break;
      
    case STATE_TEST_FAILED_GYRO:
      // Yellow-green for Gyroscope failure
      strip.setPixelColor(0, strip.Color(128, 255, 0));
      strip.show();
      break;
      
    case STATE_TEST_FAILED_MAG:
      // Cyan for Magnetometer failure
      strip.setPixelColor(0, strip.Color(0, 255, 255));
      strip.show();
      break;
      
    case STATE_TEST_FAILED_SD:
      // Magenta for SD card failure
      strip.setPixelColor(0, strip.Color(255, 0, 255));
      strip.show();
      break;
  }
}

void setup() {  
  // Initialize Serial for debugging
  Serial.begin(115200);
  
  // Initialize UART2 for command communication with FC
  Serial2.begin(115200);  // Changed from 9600 to match FC.cpp baud rate
  
  // Set alternate pins for Serial1 (USART1) - PB7 (RX) and PB6 (TX) for GPS
  Serial1.setRx(PB7);
  Serial1.setTx(PB6);
  Serial1.begin(9600);
  
  // Initialize I2C
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(100000);
  
  // Initialize the NeoPixel strip
  strip.begin();
  strip.setBrightness(50); // Set brightness (0-255, 50 is moderate)
  strip.clear(); // Clear all LEDs
  strip.show(); // Update the strip
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("System ready. Waiting for commands on UART2...");
  Serial2.println("READY"); // Send ready message to the connected microcontroller
}

void loop() {
  // Process any incoming commands on UART2
  processSerialCommands();
  
  // Update LED based on current state
  updateLED();
  
  // Short delay to prevent CPU hogging
  delay(10);
}