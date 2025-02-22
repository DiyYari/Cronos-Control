#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <HID-Project.h>

// ------------------------------------------------------------------------------------
// TWO MPUs (accelerometers)
// ------------------------------------------------------------------------------------
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

// ------------------------------------------------------------------------------------
// CONSTANTS AND PARAMETERS
// ------------------------------------------------------------------------------------
#define MAX_SPEED              300
#define THRESHOLD              10
#define DEAD_ZONE              350
#define SCROLL_DELAY           200
#define SCROLL_DEADZONE        7
#define INVERT_SCROLL          1
#define PRECISION_FACTOR       0.5

// Updated angle parameters:
// The dead zone is increased so that no scrolling occurs until the absolute angle exceeds 3°.
// Full scroll control is achieved at 15°.
#define MIN_ANGLE              3     // Minimal angle for scrolling to start (in degrees)
#define MAX_ANGLE              25    // Angle (in degrees) at which full scroll speed is achieved

#define MIN_SPEED              1
#define EXPONENTIAL_FACTOR     1
#define SMOOTHING_SAMPLES      10
#define SCROLL_SMOOTHING_SAMPLES 10

// ------------------------------------------------------------------------------------
// EEPROM Addresses
// ------------------------------------------------------------------------------------
#define EEPROM_INITIALIZED_FLAG  0
#define EEPROM_CURSOR_SPEED      1
#define EEPROM_SCROLL_SPEED      2
#define EEPROM_BUTTON_FUNCTIONS  3
#define EEPROM_DEAD_ZONE         20
#define EEPROM_SENSITIVITY       21

int cursorSpeed = 1;
int scrollSpeed = 3;

// Encoder modes
enum EncoderMode {
  VOLUME_CONTROL,
  HORIZONTAL_SCROLL,
  ZOOM_CONTROL
};

EncoderMode currentEncoderMode = VOLUME_CONTROL;

float scrollAccumulator = 0.0;
unsigned long lastScrollUpdate = 0;

int SENSITIVITY = 100;
int invertX = -1;
int invertY = 1;

// Calibration offsets for two MPUs
int16_t offset_ax1 = 0, offset_ay1 = 0, offset_az1 = 0;
int16_t offset_ax2 = 0, offset_ay2 = 0, offset_az2 = 0;

// ------------------------------------------------------------------------------------
// BUTTON PINS
// ------------------------------------------------------------------------------------
const int pinLeftButton    = 9;
const int pinRightButton   = 8;
const int pinMiddleButton  = 18;
const int pinForwardButton = 7;
const int pinBackButton    = 6;
const int pinCopyButton    = 4;
const int pinPasteButton   = 5;

// Encoder pins
const int pinVolumeEncoderA = 10;
const int pinVolumeEncoderB = 16;

int lastVolumeStateA = HIGH;
int lastVolumeStateB = HIGH;
int volumePosition   = 0;

unsigned long lastScrollTime = 0;
unsigned long lastDebugTime  = 0;
const unsigned long DEBUG_INTERVAL = 1000;

// ------------------------------------------------------------------------------------
// Button states structure
// ------------------------------------------------------------------------------------
bool buttonStates[7]     = {false, false, false, false, false, false, false};
bool buttonPressed[7]    = {false, false, false, false, false, false, false};
unsigned long lastDebounceTime[7] = {0, 0, 0, 0, 0, 0, 0};
const unsigned long debounceDelay = 50;

// ------------------------------------------------------------------------------------
// ARRAY OF FUNCTIONS FOR EACH BUTTON
// ------------------------------------------------------------------------------------
String buttonFunctions[7] = {
  "LeftClick",
  "RightClick",
  "MiddleClick",
  "Forward",
  "Back",
  "Copy",
  "Paste"
};

// ------------------------------------------------------------------------------------
// Structure for storing string function in EEPROM
// ------------------------------------------------------------------------------------
struct ButtonFunction {
  char name[15];
};

// ------------------------------------------------------------------------------------
// SAVE/LOAD FUNCTIONS (EEPROM)
// ------------------------------------------------------------------------------------
void saveButtonFunction(int buttonIndex, String function) {
  int address = EEPROM_BUTTON_FUNCTIONS + (buttonIndex * sizeof(ButtonFunction));
  ButtonFunction bf;
  function.toCharArray(bf.name, sizeof(bf.name));
  EEPROM.put(address, bf);
}

String loadButtonFunction(int buttonIndex) {
  int address = EEPROM_BUTTON_FUNCTIONS + (buttonIndex * sizeof(ButtonFunction));
  ButtonFunction bf;
  EEPROM.get(address, bf);
  return String(bf.name);
}

void saveAllSettings() {
  EEPROM.write(EEPROM_INITIALIZED_FLAG, 0xAA);
  EEPROM.write(EEPROM_CURSOR_SPEED, cursorSpeed);
  EEPROM.write(EEPROM_SCROLL_SPEED, scrollSpeed);
  EEPROM.write(EEPROM_DEAD_ZONE, DEAD_ZONE);
  EEPROM.write(EEPROM_SENSITIVITY, SENSITIVITY);

  for (int i = 0; i < 7; i++) {
    saveButtonFunction(i, buttonFunctions[i]);
  }
}

void setDefaultSettings() {
  cursorSpeed = 5;
  scrollSpeed = 5;
  buttonFunctions[0] = "LeftClick";
  buttonFunctions[1] = "RightClick";
  buttonFunctions[2] = "MiddleClick";
  buttonFunctions[3] = "Forward";
  buttonFunctions[4] = "Back";
  buttonFunctions[5] = "Copy";
  buttonFunctions[6] = "Paste";
}

void loadAllSettings() {
  if (EEPROM.read(EEPROM_INITIALIZED_FLAG) != 0xAA) {
    setDefaultSettings();
    saveAllSettings();
    return;
  }

  cursorSpeed = EEPROM.read(EEPROM_CURSOR_SPEED);
  scrollSpeed = EEPROM.read(EEPROM_SCROLL_SPEED);

  for (int i = 0; i < 7; i++) {
    buttonFunctions[i] = loadButtonFunction(i);
  }
}

// ------------------------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  Mouse.begin();
  Keyboard.begin();
  Consumer.begin();
  Serial.begin(115200);

  mpu1.initialize();
  mpu2.initialize();

  // Uncomment below to test MPU connection:
  // if (!mpu1.testConnection() || !mpu2.testConnection()) { while(1); }

  delay(1000);

  loadAllSettings();

  pinMode(pinLeftButton,    INPUT_PULLUP);
  pinMode(pinRightButton,   INPUT_PULLUP);
  pinMode(pinMiddleButton,  INPUT_PULLUP);
  pinMode(pinForwardButton, INPUT_PULLUP);
  pinMode(pinBackButton,    INPUT_PULLUP);
  pinMode(pinCopyButton,    INPUT_PULLUP);
  pinMode(pinPasteButton,   INPUT_PULLUP);

  pinMode(pinVolumeEncoderA, INPUT_PULLUP);
  pinMode(pinVolumeEncoderB, INPUT_PULLUP);

  calibrateSensors();
}

void loop() {
  processMouseMovement();
  handleButtons();
  handleScroll();  // Updated scroll function with new dead zone and angle limits
  handleEncoder();

  if (Serial.available() > 0) {
    handleSerialInput();
  }

  // Example periodic debug (if needed)
  if (millis() - lastDebugTime > DEBUG_INTERVAL) {
    lastDebugTime = millis();
  }

  delay(1);
}

// ------------------------------------------------------------------------------------
// SENSOR CALIBRATION
// ------------------------------------------------------------------------------------
void calibrateSensors() {
  int16_t ax, ay, az;
  const int numReadings = 100;
  long sum_ax1 = 0, sum_ay1 = 0, sum_az1 = 0;
  long sum_ax2 = 0, sum_ay2 = 0, sum_az2 = 0;

  // Calibration for MPU1
  for (int i = 0; i < numReadings; i++) {
    mpu1.getAcceleration(&ax, &ay, &az);
    sum_ax1 += ax;
    sum_ay1 += ay;
    sum_az1 += az;
    delay(10);
  }
  offset_ax1 = sum_ax1 / numReadings;
  offset_ay1 = sum_ay1 / numReadings;
  offset_az1 = sum_az1 / numReadings;

  // Calibration for MPU2
  for (int i = 0; i < numReadings; i++) {
    mpu2.getAcceleration(&ax, &ay, &az);
    sum_ax2 += ax;
    sum_ay2 += ay;
    sum_az2 += az;
    delay(10);
  }
  offset_ax2 = sum_ax2 / numReadings;
  offset_ay2 = sum_ay2 / numReadings;
  offset_az2 = sum_az2 / numReadings;
}

// ------------------------------------------------------------------------------------
// MOUSE MOVEMENT PROCESSING
// ------------------------------------------------------------------------------------
void processMouseMovement() {
  int16_t ax1, ay1, az1;
  mpu1.getAcceleration(&ax1, &ay1, &az1);

  ax1 -= offset_ax1;
  ay1 -= offset_ay1;

  static float emaAx = 0.0;
  static float emaAy = 0.0;

  float alpha = 0.2;
  emaAx = alpha * ay1  + (1 - alpha) * emaAx;
  emaAy = alpha * (-ax1) + (1 - alpha) * emaAy;

  float filteredAx = (abs(emaAx) < DEAD_ZONE) ? 0 : emaAx;
  float filteredAy = (abs(emaAy) < DEAD_ZONE) ? 0 : emaAy;

  int mouseX = calculateAdjustedSpeed(filteredAx) * invertX;
  int mouseY = calculateAdjustedSpeed(filteredAy) * invertY;

  if (mouseX != 0 || mouseY != 0) {
    Mouse.move(mouseX, mouseY, 0);
  }
}

int calculateAdjustedSpeed(float accelValue) {
  if (abs(accelValue) < THRESHOLD) {
    return 0;
  }

  float normalizedValue = (abs(accelValue) - THRESHOLD) / (16384.0 - THRESHOLD);
  normalizedValue = constrain(normalizedValue, 0.0, 1.0);

  float scaling = (cursorSpeed / 10.0);
  float factor  = scaling * pow(normalizedValue, 1.5);

  int speed = 1 + (int)(factor * (MAX_SPEED - MIN_SPEED));
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);

  return (accelValue < 0) ? -speed : speed;
}

// ------------------------------------------------------------------------------------
// BUTTON HANDLING
// ------------------------------------------------------------------------------------
void handleButtons() {
  int buttons[7] = {
    pinLeftButton,
    pinRightButton,
    pinMiddleButton,
    pinForwardButton,
    pinBackButton,
    pinCopyButton,
    pinPasteButton
  };

  for (int i = 0; i < 7; i++) {
    bool reading = !digitalRead(buttons[i]);

    if (reading != buttonStates[i]) {
      lastDebounceTime[i] = millis();
    }

    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading && !buttonPressed[i]) {
        performButtonFunction(i);
        buttonPressed[i] = true;
      }
      else if (!reading && buttonPressed[i]) {
        releaseButtonFunction(i);
        buttonPressed[i] = false;
      }
    }
    buttonStates[i] = reading;
  }
}

void performButtonFunction(int buttonIndex) {
  String function = buttonFunctions[buttonIndex];

  if (function == "LeftClick") {
    Mouse.press(MOUSE_LEFT);
  }
  else if (function == "RightClick") {
    Mouse.press(MOUSE_RIGHT);
  }
  else if (function == "MiddleClick") {
    Mouse.press(MOUSE_MIDDLE);
  }
  else if (function == "Forward") {
    Keyboard.press(KEY_LEFT_ALT);
    Keyboard.press(KEY_RIGHT_ARROW);
    Keyboard.releaseAll();
  }
  else if (function == "Back") {
    Keyboard.press(KEY_LEFT_ALT);
    Keyboard.press(KEY_LEFT_ARROW);
    Keyboard.releaseAll();
  }
  else if (function == "Copy") {
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press('c');
    Keyboard.releaseAll();
  }
  else if (function == "Paste") {
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press('v');
    Keyboard.releaseAll();
  }
  else if (function == "Ctrl") {
    Keyboard.press(KEY_LEFT_CTRL);
  }
  else if (function == "Alt") {
    Keyboard.press(KEY_LEFT_ALT);
  }
  else if (function == "Shift") {
    Keyboard.press(KEY_LEFT_SHIFT);
  }
  else {
    pressCustomCombination(function);
  }
}

void releaseButtonFunction(int buttonIndex) {
  String function = buttonFunctions[buttonIndex];

  if (function == "LeftClick") {
    Mouse.release(MOUSE_LEFT);
  }
  else if (function == "RightClick") {
    Mouse.release(MOUSE_RIGHT);
  }
  else if (function == "MiddleClick") {
    Mouse.release(MOUSE_MIDDLE);
  }
  else if (function == "Ctrl") {
    Keyboard.release(KEY_LEFT_CTRL);
  }
  else if (function == "Alt") {
    Keyboard.release(KEY_LEFT_ALT);
  }
  else if (function == "Shift") {
    Keyboard.release(KEY_LEFT_SHIFT);
  }
}

void pressCustomCombination(String combo) {
  String parts[8];
  int partCount = 0;
  int start = 0;

  while (true) {
    int plusIndex = combo.indexOf('+', start);
    if (plusIndex == -1) {
      parts[partCount++] = combo.substring(start);
      break;
    } else {
      parts[partCount++] = combo.substring(start, plusIndex);
      start = plusIndex + 1;
    }
    if (partCount >= 8) break;
  }

  bool shiftSpecified = false;
  for (int i = 0; i < partCount; i++) {
    String token = parts[i];
    token.trim();
    if (token.equalsIgnoreCase("SHIFT")) {
      shiftSpecified = true;
      break;
    }
  }

  for (int i = 0; i < partCount; i++) {
    String token = parts[i];
    token.trim();

    if (token.equalsIgnoreCase("CTRL")) {
      Keyboard.press(KEY_LEFT_CTRL);
    }
    else if (token.equalsIgnoreCase("SHIFT")) {
      Keyboard.press(KEY_LEFT_SHIFT);
    }
    else if (token.equalsIgnoreCase("ALT")) {
      Keyboard.press(KEY_LEFT_ALT);
    }
    else if (token.equalsIgnoreCase("GUI") || token.equalsIgnoreCase("WIN")) {
      Keyboard.press(KEY_LEFT_GUI);
    }
    else {
      if (token.length() == 1) {
        char c = token.charAt(0);
        if (!shiftSpecified && ((c >= 'A' && c <= 'Z') || (c >= 'А' && c <= 'Я'))) {
          c = tolower(c);
        }
        Keyboard.press(c);
      }
      else {
        if (token.equalsIgnoreCase("ENTER")) {
          Keyboard.press(KEY_ENTER);
        }
        else if (token.equalsIgnoreCase("ESC")) {
          Keyboard.press(KEY_ESC);
        }
        else if (token.equalsIgnoreCase("F1")) {
          Keyboard.press(KEY_F1);
        }
        else if (token.equalsIgnoreCase("F2")) {
          Keyboard.press(KEY_F2);
        }
      }
    }
  }

  delay(50);
  Keyboard.releaseAll();
}

// ------------------------------------------------------------------------------------
// SCROLL HANDLING WITH DEBUG OUTPUT AND FREEZE ON DIRECTION REVERSAL
// ------------------------------------------------------------------------------------

// Set DEBUG_SCROLL to true to output debugging info to the Serial Monitor.
#define DEBUG_SCROLL true
unsigned long lastScrollDebugTime = 0;

void handleScroll() {
  // Read data from MPU2 and apply calibration offsets
  int16_t ax2, ay2, az2;
  mpu2.getAcceleration(&ax2, &ay2, &az2);
  ax2 -= offset_ax2;
  ay2 -= offset_ay2;
  az2 -= offset_az2;
  
  // Convert accelerometer reading to an approximate angle in degrees
  float rawAngle = (float)ay2 / 16384.0 * 90.0;
  
  // Use an exponential moving average (EMA) to smooth the angle
  static float emaScrollAngle = 0.0;
  const float alphaScroll = 0.2;
  
  unsigned long now = millis();
  
  // Variables for freezing the scroll update when a reversal is detected
  static bool freezeActive = false;       // True when in freeze mode
  static unsigned long freezeStartTime = 0; // Time when freeze started
  static int lastSign = 0;                  // Last stable direction (+1 or -1)
  const unsigned long freezeDuration = 200; // Freeze period in ms
  
  // If freeze is active, ignore new accumulation and let the accumulator decay
  if (freezeActive) {
    if (now - freezeStartTime >= freezeDuration) {
      freezeActive = false;
      emaScrollAngle = rawAngle; // Reinitialize EMA to avoid a jump
      if (DEBUG_SCROLL) {
        Serial.println("Freeze period ended.");
      }
    } else {
      scrollAccumulator *= 0.95; // Decay the accumulator during freeze
      int steps = (int)floor(scrollAccumulator);
      if (steps != 0) {
        Mouse.move(0, 0, steps);
        scrollAccumulator -= steps;
      }
      if (DEBUG_SCROLL && now - lastScrollDebugTime > 200) {
        Serial.print("Freeze active | rawAngle: ");
        Serial.print(rawAngle);
        Serial.print(" | emaScrollAngle: ");
        Serial.print(emaScrollAngle);
        Serial.print(" | scrollAccumulator: ");
        Serial.println(scrollAccumulator);
        lastScrollDebugTime = now;
      }
      return; // Skip further processing during freeze
    }
  }
  
  // Update the EMA with the new rawAngle value
  emaScrollAngle = alphaScroll * rawAngle + (1 - alphaScroll) * emaScrollAngle;
  float angleAbs = fabs(emaScrollAngle);
  int currentSign = (emaScrollAngle > 0) ? 1 : -1;
  
  // If the absolute angle is below MIN_ANGLE, reset the stable direction and accumulator
  if (angleAbs < MIN_ANGLE) {
    lastSign = 0;
    scrollAccumulator = 0.0;
  } else {
    // If a stable direction exists and the current sign is different, a reversal is detected
    if (lastSign != 0 && currentSign != lastSign) {
      freezeActive = true;
      freezeStartTime = now;
      scrollAccumulator = 0.0;
      lastSign = currentSign;
      if (DEBUG_SCROLL) {
        Serial.print("Direction reversal detected. Entering freeze. currentSign: ");
        Serial.println(currentSign);
      }
      return;
    }
    // No reversal detected: update the stable direction and accumulate scroll.
    lastSign = currentSign;
    // Normalize the angle: map MIN_ANGLE to 0 and MAX_ANGLE to 1.
    float normalizedAngle = (angleAbs - MIN_ANGLE) / (float)(MAX_ANGLE - MIN_ANGLE);
    normalizedAngle = constrain(normalizedAngle, 0.0, 1.0);
    // Use a logistic function for a smooth transition of scroll speed.
    float logistic = 1.0 / (1.0 + exp(-12 * (normalizedAngle - 0.5)));
    float baseSpeed = 0.03 + (scrollSpeed - 0.03) * logistic;
    scrollAccumulator += baseSpeed * currentSign * INVERT_SCROLL * 0.2;
  }
  
int steps = (int)floor(scrollAccumulator);
if (steps != 0) {
  Mouse.move(0, 0, steps);
  scrollAccumulator -= steps;
}
  
  if (DEBUG_SCROLL && now - lastScrollDebugTime > 200) {
    Serial.print("rawAngle: ");
    Serial.print(rawAngle);
    Serial.print(" | emaScrollAngle: ");
    Serial.print(emaScrollAngle);
    Serial.print(" | freezeActive: ");
    Serial.print(freezeActive);
    Serial.print(" | currentSign: ");
    Serial.print(currentSign);
    Serial.print(" | scrollAccumulator: ");
    Serial.print(scrollAccumulator);
    Serial.print(" | steps: ");
    Serial.println(steps);
    lastScrollDebugTime = now;
  }
}

// ------------------------------------------------------------------------------------
// HANDLE ENCODER INPUT
// ------------------------------------------------------------------------------------
void handleEncoder() {
  int aState = digitalRead(pinVolumeEncoderA);
  int bState = digitalRead(pinVolumeEncoderB);

  if (aState != lastVolumeStateA) {
    if (bState != aState) {
      volumePosition++;
      switch(currentEncoderMode) {
        case VOLUME_CONTROL:
          Consumer.write(MEDIA_VOLUME_UP);
          break;
        case HORIZONTAL_SCROLL:
          Mouse.move(-1, 0, 0);
          break;
        case ZOOM_CONTROL:
          Keyboard.press(KEY_LEFT_CTRL);
          Keyboard.press(KEY_LEFT_SHIFT);
          Keyboard.press('=');
          Keyboard.releaseAll();
          break;
      }
    } else {
      volumePosition--;
      switch(currentEncoderMode) {
        case VOLUME_CONTROL:
          Consumer.write(MEDIA_VOLUME_DOWN);
          break;
        case HORIZONTAL_SCROLL:
          Mouse.move(1, 0, 0);
          break;
        case ZOOM_CONTROL:
          Keyboard.press(KEY_LEFT_CTRL);
          Keyboard.press('-');
          Keyboard.releaseAll();
          break;
      }
    }
  }
  lastVolumeStateA = aState;
  lastVolumeStateB = bState;
}

// ------------------------------------------------------------------------------------
// HANDLE SERIAL INPUT FOR SETTINGS
// ------------------------------------------------------------------------------------
void handleSerialInput() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    bool settingsChanged = false;
    if (command.startsWith("CURSOR_SPEED:")) {
      cursorSpeed = command.substring(13).toInt();
      cursorSpeed = constrain(cursorSpeed, 1, 10);
      settingsChanged = true;
    }
    else if (command.startsWith("SCROLL_SPEED:")) {
      scrollSpeed = command.substring(13).toInt();
      scrollSpeed = constrain(scrollSpeed, 1, 10);
      settingsChanged = true;
    }
    else if (command.startsWith("BTN:")) {
      command = command.substring(4);
      int firstColon = command.indexOf(':');
      int secondColon = command.indexOf(':', firstColon + 1);
      if (firstColon != -1 && secondColon != -1) {
        int buttonPin = command.substring(0, firstColon).toInt();
        String function = command.substring(firstColon + 1, secondColon);
        int buttonIndex = -1;
        int pins[] = { pinLeftButton, pinRightButton, pinMiddleButton, pinForwardButton, pinBackButton, pinCopyButton, pinPasteButton };
        for (int i = 0; i < 7; i++) {
          if (pins[i] == buttonPin) {
            buttonIndex = i;
            break;
          }
        }
        if (buttonIndex != -1) {
          buttonFunctions[buttonIndex] = function;
          settingsChanged = true;
          Serial.print("BTN_SET:");
          Serial.print(buttonPin);
          Serial.print(":");
          Serial.println(function);
        }
      }
    }
    else if (command == "RESET:ALL") {
      setDefaultSettings();
      saveAllSettings();
      Serial.println("OK:RESET");
    }
    if (settingsChanged) {
      saveAllSettings();
      Serial.println("Settings saved to EEPROM");
    }
  }
}

// ------------------------------------------------------------------------------------
// OPTIONAL DEBUG OUTPUT (currently empty)
// ------------------------------------------------------------------------------------
void debugOutput() {
  // Additional debug output can be added here if needed.
}
