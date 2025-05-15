#include <Arduino.h>
#include <RadioLib.h>
#include <HardwareTimer.h>

// ─── LoRa Configuration ────────────────────────────────────────────
#define NSS    PA4
#define DIO0   PA8
#define NRST   PA9
#define DIO1   PA10
SX1276 radio = new Module(NSS, DIO0, NRST, DIO1);

// ─── Serial2 Communication with NAVC ────────────────────────────────
#define UART2_TX PA2  // UART2 TX on PA2
#define UART2_RX PA3  // UART2 RX on PA3
HardwareSerial Serial2(UART2_TX, UART2_RX);

// ─── Servo Configuration ──────────────────────────────────────────
#define SERVO_PIN PB10
const int pwmFreq    = 200;   // Hz
const int closedDuty = 71;    // ~20%
const int openDuty   = 112;   // ~44%
HardwareTimer *servoTimer = nullptr;
bool isServoOpen       = false;

// ─── Altitude Settings ────────────────────────────────────────────
const float TARGET_ALTITUDE = 1828.8;  // meters
float launchAltitude        = 0.0;

// ─── Parser Ring Buffer ──────────────────────────────────────────
#define MAX_FRAMES     16
#define MAX_FRAME_LEN 128

static char frames[MAX_FRAMES][MAX_FRAME_LEN];
static volatile int frameHead = 0, frameTail = 0;

// Frame builder
static char currFrame[MAX_FRAME_LEN];
static int  currIndex = 0;

// ─── State Machine Definition ─────────────────────────────────────
enum SystemState {
  STATE_IDLE,
  STATE_TEST,
  STATE_ARMED
};

SystemState currentState = STATE_IDLE;

// ─── Command Definitions ────────────────────────────────────────
const String CMD_ARM = "ARM";
const String CMD_DISARM = "DISARM";
const String CMD_TEST = "TEST";
const String CMD_EXIT_TEST = "EXIT_TEST";
const String CMD_RUN_TEST = "RUN_TEST";
const String CMD_SET_LORA = "SET_LORA";

// ─── LoRa Configuration ─────────────────────────────────────────
struct LoraSettings {
  float frequency = 915.0;    // Default 915 MHz
  int spreadingFactor = 7;    // Default SF7
  float bandwidth = 250.0;    // Default 250 kHz
  int codingRate = 5;         // Default CR 4/5
  uint8_t syncWord = 0xAB;    // Default sync word
};

LoraSettings loraConfig;

// Called automatically by Arduino when Serial2 has data
void serialEvent2() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '<') {
      currIndex = 0;
      currFrame[currIndex++] = c;
    }
    else if (currIndex > 0) {
      if (currIndex < MAX_FRAME_LEN - 1) {
        currFrame[currIndex++] = c;
      }
      if (c == '>') {
        currFrame[currIndex] = '\0';
        // enqueue
        strncpy(frames[frameHead], currFrame, MAX_FRAME_LEN);
        frameHead = (frameHead + 1) % MAX_FRAMES;
        currIndex = 0;
      }
    }
  }
}

// ─── LoRa Communication ────────────────────────────────────────────
class LoraComm {
  String receivedData;
  const uint8_t fcAddr = 0xA2, gsAddr = 0xA1;

public:
  void setup() {
    applySettings();
  }
  
  void applySettings() {
    radio.begin(loraConfig.frequency);
    radio.setSpreadingFactor(loraConfig.spreadingFactor);
    radio.setBandwidth(loraConfig.bandwidth);
    radio.setCodingRate(loraConfig.codingRate);
    radio.setSyncWord(loraConfig.syncWord);
    radio.setNodeAddress(fcAddr);
    radio.setDio0Action([](){}, RISING);
    radio.startReceive();
    
    Serial.println("LoRa settings applied:");
    Serial.println("Frequency: " + String(loraConfig.frequency) + " MHz");
    Serial.println("Spreading Factor: SF" + String(loraConfig.spreadingFactor));
    Serial.println("Bandwidth: " + String(loraConfig.bandwidth) + " kHz");
    Serial.println("Coding Rate: 4/" + String(loraConfig.codingRate));
    Serial.println("Sync Word: 0x" + String(loraConfig.syncWord, HEX));
  }

  void send(String data) {
    int16_t state = radio.transmit(data, gsAddr);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.println("LoRa TX err: " + String(state));
    }
    radio.startReceive();
  }

  String receive() {
    if (!radio.available()) {
      return "";
    }
    int16_t state = radio.receive(receivedData);
    radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      return receivedData;
    }
    return "";
  }
};

// ─── Flight Computer ──────────────────────────────────────────────
class FC {
  LoraComm lora;
  bool buzzerActive = false;
  unsigned long buzzerStart = 0;
  String commandBuffer = "";
  unsigned long lastIdleMsg = 0; // Timer for idle mode message

  // extract 9th CSV field (BMP280 altitude)
  float getAltitude(const String &d) {
    int commas = 0;
    for (int i = 0; i < d.length(); i++) {
      if (d[i] == ',' && ++commas == 8) {
        int j = d.indexOf(',', i + 1);
        return d.substring(i + 1, j < 0 ? d.length() : j).toFloat();
      }
    }
    return 0.0f;
  }

  void controlServo(float alt) {
    if (launchAltitude == 0.0f && alt > 0.0f) {
      launchAltitude = alt;
      Serial.println("Launch alt: " + String(launchAltitude));
    }
    if (!isServoOpen && alt < TARGET_ALTITUDE) {
      servoTimer->setPWM(3, SERVO_PIN, pwmFreq, closedDuty * 100 / 255);
    }
    else if (!isServoOpen && alt >= TARGET_ALTITUDE) {
      servoTimer->setPWM(3, SERVO_PIN, pwmFreq, openDuty * 100 / 255);
      isServoOpen = true;
      Serial.println("Servo opened at alt: " + String(alt));
    }
  }

  void processLoRaCommand(String cmd) {
    Serial.println("LoRa command: " + cmd);

    // Commands available in ALL states
    if (cmd.startsWith("BUZZER_ON")) {
      digitalWrite(PB13, HIGH);
      buzzerActive = true;
      buzzerStart = millis();
      lora.send("BUZZER_ON_ACK");
      return;
    }
    
    // Process commands based on state
    switch (currentState) {
      case STATE_IDLE:
        if (cmd == CMD_ARM) {
          currentState = STATE_ARMED;
          lora.send("ARMED");
          Serial2.println("ARMED");
          Serial.println("State changed to ARMED");
        }
        else if (cmd == CMD_TEST) {
          currentState = STATE_TEST;
          lora.send("TEST_MODE_ENTERED");
          Serial2.println("TEST_MODE_ENTERED");
          Serial.println("State changed to TEST");
        }
        else if (cmd.startsWith(CMD_SET_LORA)) {
          processLoRaSettings(cmd);
        }
        else {
          lora.send("ERROR: Command not allowed in IDLE state");
        }
        break;
        
      case STATE_TEST:
        Serial.println("Debug - received command in TEST state: '" + cmd + "'");
        Serial.println("Debug - comparing with EXIT_TEST: '" + CMD_EXIT_TEST + "'");
        Serial.println("Debug - Equal?: " + String(cmd == CMD_EXIT_TEST));
        
        if (cmd == CMD_EXIT_TEST) {
          currentState = STATE_IDLE;
          Serial.println("TEST_MODE_EXITED");
          Serial2.println("TEST_MODE_EXITED");
          Serial.println("State changed to IDLE");
        }
        else if (cmd == CMD_RUN_TEST) {
          Serial.println("Running tests on NAVC...");
          Serial2.println("TEST"); // Send test command to NAVC
          lora.send("RUNNING_TESTS");
        }
        else {
          Serial.println("ERROR: Command not allowed in TEST state");
        }
        break;
        
      case STATE_ARMED:
        if (cmd == CMD_DISARM) {
          currentState = STATE_IDLE;
          lora.send("DISARMED");
          Serial2.println("DISARMED");
          Serial.println("State changed to IDLE");
        }
        else if (cmd.startsWith(CMD_SET_LORA)) {
          processLoRaSettings(cmd);
        }
        else {
          // In ARMED state, forward all other commands to NAVC
          Serial2.println(cmd);
          lora.send("FORWARDED: " + cmd);
        }
        break;
    }
  }

  void processSerialCommand(String cmd) {
    Serial.println("Serial command: " + cmd);
    
    // Process the command the same way as LoRa commands
    // We'll reuse the same logic but send responses to Serial
    
    // Commands available in ALL states
    if (cmd.startsWith("BUZZER_ON")) {
      digitalWrite(PB13, HIGH);
      buzzerActive = true;
      buzzerStart = millis();
      Serial.println("BUZZER_ON_ACK");
      return;
    }
    
    // Process commands based on state
    switch (currentState) {
      case STATE_IDLE:
        if (cmd == CMD_ARM) {
          currentState = STATE_ARMED;
          Serial.println("ARMED");
          Serial2.println("ARMED");
          Serial.println("State changed to ARMED");
        }
        else if (cmd == CMD_TEST) {
          currentState = STATE_TEST;
          Serial.println("TEST_MODE_ENTERED");
          Serial2.println("TEST_MODE_ENTERED");
          Serial.println("State changed to TEST");
        }
        else if (cmd.startsWith(CMD_SET_LORA)) {
          processLoRaSettingsFromSerial(cmd);
        }
        else {
          Serial.println("ERROR: Command not allowed in IDLE state");
        }
        break;
        
      case STATE_TEST:
        if (cmd == CMD_EXIT_TEST) {
          currentState = STATE_IDLE;
          Serial.println("TEST_MODE_EXITED");
          Serial2.println("TEST_MODE_EXITED");
          Serial.println("State changed to IDLE");
        }
        else if (cmd == CMD_RUN_TEST) {
          Serial2.println("TEST"); // Send test command to NAVC
          Serial.println("RUNNING_TESTS");
          Serial.println("Running tests on NAVC");
        }
        else {
          Serial.println("ERROR: Command not allowed in TEST state");
        }
        break;
        
      case STATE_ARMED:
        if (cmd == CMD_DISARM) {
          currentState = STATE_IDLE;
          Serial.println("DISARMED");
          Serial2.println("DISARMED");
          Serial.println("State changed to IDLE");
        }
        else if (cmd.startsWith(CMD_SET_LORA)) {
          processLoRaSettingsFromSerial(cmd);
        }
        else {
          // In ARMED state, forward all other commands to NAVC
          Serial2.println(cmd);
          Serial.println("FORWARDED: " + cmd);
        }
        break;
    }
  }

  void processLoRaSettings(String cmd) {
    // Format: SET_LORA,frequency,spreadingFactor,bandwidth,codingRate,syncWord
    // Example: SET_LORA,915.0,7,250.0,5,171
    
    int params[5] = {0}; // To track which parameters were provided
    String parts[6];
    
    int idx = 0;
    int startPos = 0;
    int commaPos = cmd.indexOf(',');
    
    while (commaPos >= 0 && idx < 6) {
      parts[idx] = cmd.substring(startPos, commaPos);
      startPos = commaPos + 1;
      commaPos = cmd.indexOf(',', startPos);
      idx++;
    }
    
    if (startPos < cmd.length() && idx < 6) {
      parts[idx] = cmd.substring(startPos);
      idx++;
    }
    
    // Check if we have at least one parameter
    if (idx < 2) {
      lora.send("ERROR: Invalid LoRa settings format");
      return;
    }
    
    // Process parameters
    for (int i = 1; i < idx; i++) {
      if (parts[i].length() > 0) {
        switch (i) {
          case 1: // frequency
            loraConfig.frequency = parts[i].toFloat();
            params[0] = 1;
            break;
          case 2: // spreading factor
            loraConfig.spreadingFactor = parts[i].toInt();
            params[1] = 1;
            break;
          case 3: // bandwidth
            loraConfig.bandwidth = parts[i].toFloat();
            params[2] = 1;
            break;
          case 4: // coding rate
            loraConfig.codingRate = parts[i].toInt();
            params[3] = 1;
            break;
          case 5: // sync word
            loraConfig.syncWord = (uint8_t)strtol(parts[i].c_str(), NULL, 10);
            params[4] = 1;
            break;
        }
      }
    }
    
    // Apply settings
    lora.applySettings();
    
    // Send acknowledgment
    String response = "LoRa settings updated: ";
    if (params[0]) response += "Freq=" + String(loraConfig.frequency) + " ";
    if (params[1]) response += "SF=" + String(loraConfig.spreadingFactor) + " ";
    if (params[2]) response += "BW=" + String(loraConfig.bandwidth) + " ";
    if (params[3]) response += "CR=4/" + String(loraConfig.codingRate) + " ";
    if (params[4]) response += "SW=0x" + String(loraConfig.syncWord, HEX);
    
    lora.send(response);
  }
  
  void processLoRaSettingsFromSerial(String cmd) {
    // Process LoRa settings from Serial with same logic
    // but respond to Serial instead of LoRa
    int params[5] = {0}; // To track which parameters were provided
    String parts[6];
    
    int idx = 0;
    int startPos = 0;
    int commaPos = cmd.indexOf(',');
    
    while (commaPos >= 0 && idx < 6) {
      parts[idx] = cmd.substring(startPos, commaPos);
      startPos = commaPos + 1;
      commaPos = cmd.indexOf(',', startPos);
      idx++;
    }
    
    if (startPos < cmd.length() && idx < 6) {
      parts[idx] = cmd.substring(startPos);
      idx++;
    }
    
    // Check if we have at least one parameter
    if (idx < 2) {
      Serial.println("ERROR: Invalid LoRa settings format");
      return;
    }
    
    // Process parameters
    for (int i = 1; i < idx; i++) {
      if (parts[i].length() > 0) {
        switch (i) {
          case 1: // frequency
            loraConfig.frequency = parts[i].toFloat();
            params[0] = 1;
            break;
          case 2: // spreading factor
            loraConfig.spreadingFactor = parts[i].toInt();
            params[1] = 1;
            break;
          case 3: // bandwidth
            loraConfig.bandwidth = parts[i].toFloat();
            params[2] = 1;
            break;
          case 4: // coding rate
            loraConfig.codingRate = parts[i].toInt();
            params[3] = 1;
            break;
          case 5: // sync word
            loraConfig.syncWord = (uint8_t)strtol(parts[i].c_str(), NULL, 10);
            params[4] = 1;
            break;
        }
      }
    }
    
    // Apply settings
    lora.applySettings();
    
    // Send acknowledgment to Serial
    String response = "LoRa settings updated: ";
    if (params[0]) response += "Freq=" + String(loraConfig.frequency) + " ";
    if (params[1]) response += "SF=" + String(loraConfig.spreadingFactor) + " ";
    if (params[2]) response += "BW=" + String(loraConfig.bandwidth) + " ";
    if (params[3]) response += "CR=4/" + String(loraConfig.codingRate) + " ";
    if (params[4]) response += "SW=0x" + String(loraConfig.syncWord, HEX);
    
    Serial.println(response);
  }

  void processNavcResponse() {
    // Process responses from NAVC and forward to LoRa
    if (frameTail != frameHead) {
      char *pkt = frames[frameTail++];
      if (frameTail >= MAX_FRAMES) frameTail = 0;

      // strip framing and forward
      String payload = String(pkt).substring(1, strlen(pkt) - 1);
      lora.send(payload);

      // Print to Serial for debugging
      Serial.println("NAVC: " + payload);

      // Only control servo in ARMED state
      if (currentState == STATE_ARMED) {
        controlServo(getAltitude(payload));
      }
    }
  }

  // New method to handle test command response forwarding
  void handleTestResponses() {
    // Only process additional responses during TEST state
    if (currentState == STATE_TEST) {
      // Don't read directly here if serialEvent2() is also reading
      // Instead, process existing frames or direct output
      
      // Check for regular NAVC frames but also forward as test responses
      if (frameTail != frameHead) {
        char *pkt = frames[frameTail++];
        if (frameTail >= MAX_FRAMES) frameTail = 0;
        
        // strip framing and forward
        String payload = String(pkt).substring(1, strlen(pkt) - 1);
        Serial.println("NAVC Test Frame: " + payload);
        lora.send("TEST_OUTPUT: " + payload);
      }
      
      // Try to read any direct (non-framed) output as well
      if (Serial2.available() > 0) {
        String directOutput = Serial2.readStringUntil('\n');
        if (directOutput.length() > 0) {
          directOutput.trim();
          Serial.println("NAVC Test Direct: " + directOutput);
          lora.send("TEST_DIRECT: " + directOutput);
        }
      }
    }
  }

public:
  void setup() {
    // LoRa & buzzer & servo
    lora.setup();
    pinMode(PB13, OUTPUT); digitalWrite(PB13, LOW);
    pinMode(SERVO_PIN, OUTPUT);
    servoTimer = new HardwareTimer(TIM2);
    servoTimer->setPWM(3, SERVO_PIN, pwmFreq, closedDuty * 100 / 255);

    // Serial2 for NAVC
    Serial2.begin(115200);
    
    // Initial state messages
    lora.send("FC_INITIALIZED_IN_IDLE_STATE");
    Serial2.println("FC_INITIALIZED");
    Serial.println("Flight Controller Initialized in IDLE state");
  }

  void loop() {
    // Check and process NAVC responses
    processNavcResponse();
    
    // Handle additional test responses
    handleTestResponses();

    // Process LoRa commands
    String cmd = lora.receive();
    if (cmd.length() > 0) {
      processLoRaCommand(cmd);
    }

    // Process Serial commands
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (commandBuffer.length() > 0) {
          processSerialCommand(commandBuffer);
          commandBuffer = "";
        }
      } else {
        commandBuffer += c;
      }
    }

    // Send IDLE mode message periodically
    if (currentState == STATE_IDLE && millis() - lastIdleMsg >= 5000) {  // Every 5 seconds
      Serial.println("FC IN IDLE MODE");
      lastIdleMsg = millis();
    }

    // Manage buzzer timer
    if (buzzerActive && millis() - buzzerStart >= 2000) {
      digitalWrite(PB13, LOW);
      buzzerActive = false;
      Serial.println("Buzzer off");
    }
  }
};

FC fc;

void setup() {
  // USB‑CDC
  Serial.begin(115200);  // Changed from 921600 to 115200
  // start the FC
  fc.setup();
}

void loop() {
  fc.loop();
  // let serialEvent2 run
  delay(1);
}
