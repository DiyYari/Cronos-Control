#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <HID-Project.h>
#include <math.h> 

MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

#define MAX_SPEED 1600 
#define BASE_THRESHOLD 10
#define BASE_DEAD_ZONE 550
#define SCROLL_DELAY 200
#define SCROLL_DEADZONE 15
#define INVERT_SCROLL 1
#define MAX_SCROLL_SPEED 10
#define MIN_ANGLE 4
#define MAX_ANGLE 15
#define MIN_SPEED 1
#define EXPONENTIAL_FACTOR 1
#define SMOOTHING_SAMPLES 10
#define SCROLL_SMOOTHING_SAMPLES 10
bool precisionMode = false; 
float precisionFactor = 0.8; 

int deadZoneHigh = BASE_DEAD_ZONE + 50;
int deadZoneLow = BASE_DEAD_ZONE - 50;

bool isCursorIdle = false;
float idleThreshold = 100; 
int idleFadeOutTime = 150; 
unsigned long lastMovementTime = 0;
float currentMouseX = 0;
float currentMouseY = 0;

#define EEPROM_INITIALIZED_FLAG 0
#define EEPROM_CURSOR_SPEED 1
#define EEPROM_SCROLL_SPEED 2
#define EEPROM_BUTTON_FUNCTIONS 3
#define EEPROM_DEAD_ZONE 20 
#define EEPROM_SENSITIVITY 21
#define EEPROM_PRECISION_FACTOR 22

int cursorSpeed = 5;
int scrollSpeed = 3;
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

float emaAx = 0.0;
float emaAy = 0.0;

int16_t offset_ax1 = 0, offset_ay1 = 0, offset_az1 = 0;
int16_t offset_ax2 = 0, offset_ay2 = 0, offset_az2 = 0;

const int pinLeftButton = 9;
const int pinRightButton = 8;
const int pinMiddleButton = 18;
const int pinForwardButton = 7;
const int pinBackButton = 6;
const int pinCopyButton = 4;
const int pinPasteButton = 5;
// Пины для энкодера
const int pinVolumeEncoderA = 10;
const int pinVolumeEncoderB = 16;
int lastVolumeStateA = HIGH;
int lastVolumeStateB = HIGH;
int volumePosition = 0;
unsigned long lastScrollTime = 0;
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 100; 

bool buttonStates[7] = {false, false, false, false, false, false, false};
bool buttonPressed[7] = {false, false, false, false, false, false, false};
unsigned long lastDebounceTime[7] = {0, 0, 0, 0, 0, 0, 0};
const unsigned long debounceDelay = 50;

String buttonFunctions[7] = {
"LeftClick",
"RightClick",
"MiddleClick",
"Forward",
"Back",
"Copy",
"Paste"
};

struct ButtonFunction {
char name[15]; 
};

void saveButtonFunction(int buttonIndex, String function) {
int address = EEPROM_BUTTON_FUNCTIONS + (buttonIndex * sizeof(ButtonFunction));
ButtonFunction bf;
function.toCharArray(bf.name, sizeof(bf.name) - 1); // Ensure null termination
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
EEPROM.put(EEPROM_PRECISION_FACTOR, precisionFactor);
for (int i = 0; i < 7; i++) {
saveButtonFunction(i, buttonFunctions[i]);
}
}
void setDefaultSettings() {
cursorSpeed = 5;
scrollSpeed = 5;
precisionFactor = 0.8;
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
EEPROM.get(EEPROM_PRECISION_FACTOR, precisionFactor);
for (int i = 0; i < 7; i++) {
buttonFunctions[i] = loadButtonFunction(i);
}
}

float calculateDynamicAlpha(float accelValue) {
float baseAlpha = 0.15; 
if (abs(accelValue) < BASE_DEAD_ZONE * 0.5) {
return 0.05; 
} else {
return baseAlpha;
}
}

void calibrateSensors() {
int16_t ax, ay, az;
const int numReadings = 100;
long sum_ax1 = 0, sum_ay1 = 0, sum_az1 = 0;
long sum_ax2 = 0, sum_ay2 = 0, sum_az2 = 0;
Serial.println("Calibrating MPU1...");
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
Serial.print("MPU1 Offsets: ");
Serial.print(offset_ax1); Serial.print(" ");
Serial.print(offset_ay1); Serial.print(" ");
Serial.println(offset_az1);
Serial.println("Calibrating MPU2...");
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
Serial.print("MPU2 Offsets: ");
Serial.print(offset_ax2); Serial.print(" ");
Serial.print(offset_ay2); Serial.print(" ");
Serial.println(offset_az2);
Serial.println("Calibration complete.");
delay(1000);
}

int calculateAdjustedSpeed(float accelValue) {
    int threshold = BASE_THRESHOLD; 
    if (abs(accelValue) < threshold) {
        return 0;
    }

    float effectiveAccel = abs(accelValue) - threshold;
    float normalizedValue = effectiveAccel / (16384.0 - threshold);
    normalizedValue = constrain(normalizedValue, 0.0, 1.0);

    float scaling = (cursorSpeed / 10.0); 

    float power = 1.8; 
    float factor = scaling * pow(normalizedValue, power);

    const int INTERNAL_MAX_SPEED = 500; 

    int speed = MIN_SPEED + (int)(factor * (INTERNAL_MAX_SPEED - MIN_SPEED));
    speed = constrain(speed, MIN_SPEED, INTERNAL_MAX_SPEED); 

    return (accelValue < 0) ? -speed : speed;
}

void processMouseMovement() {
int16_t ax1, ay1, az1;
mpu1.getAcceleration(&ax1, &ay1, &az1);
ax1 -= offset_ax1;
ay1 -= offset_ay1;

float alphaX = calculateDynamicAlpha(ay1);
float alphaY = calculateDynamicAlpha(-ax1);
emaAx = alphaX * ay1 + (1 - alphaX) * emaAx;
emaAy = alphaY * (-ax1) + (1 - alphaY) * emaAy;
float filteredAx = emaAx;
float filteredAy = emaAy;

static bool inDeadZoneX = true;
static bool inDeadZoneY = true;
if (inDeadZoneX) {
if (abs(filteredAx) > deadZoneHigh) {
inDeadZoneX = false;
} else {
filteredAx = 0;
}
} else {
if (abs(filteredAx) < deadZoneLow) {
inDeadZoneX = true;
filteredAx = 0;
}
}
if (inDeadZoneY) {
if (abs(filteredAy) > deadZoneHigh) {
inDeadZoneY = false;
} else {
filteredAy = 0;
}
} else {
if (abs(filteredAy) < deadZoneLow) {
inDeadZoneY = true;
filteredAy = 0;
}
}
int mouseX = calculateAdjustedSpeed(filteredAx) * invertX;
int mouseY = calculateAdjustedSpeed(filteredAy) * invertY;

float currentAccel = abs(ay1) + abs(-ax1);
if (currentAccel < idleThreshold) {
if (!isCursorIdle) {
isCursorIdle = true;
lastMovementTime = millis();
currentMouseX = 0;
currentMouseY = 0;
} else {

if (millis() - lastMovementTime < idleFadeOutTime) {
float fadeFactor = 1.0 - (float)(millis() - lastMovementTime) / idleFadeOutTime;
currentMouseX *= fadeFactor;
currentMouseY *= fadeFactor;
} else {
currentMouseX = 0;
currentMouseY = 0;
}
}
} else {
isCursorIdle = false;
currentMouseX = mouseX;
currentMouseY = mouseY;
lastMovementTime = millis();
}

    if (precisionMode) {
        currentMouseX = (int)(currentMouseX * precisionFactor);
        currentMouseY = (int)(currentMouseY * precisionFactor);
    }

    int finalMouseX = constrain(round(currentMouseX), -127, 127);
    int finalMouseY = constrain(round(currentMouseY), -127, 127);

    if (finalMouseX != 0 || finalMouseY != 0) { 
        Mouse.move(finalMouseX, finalMouseY, 0);
    }
}

void handleButtons() {
int buttons[] = {
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
  } else if (!reading && buttonPressed[i]) {
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
} else if (function == "RightClick") {
Mouse.press(MOUSE_RIGHT);
} else if (function == "MiddleClick") {
Mouse.press(MOUSE_MIDDLE);
} else if (function == "Forward") {
Keyboard.press(KEY_LEFT_ALT);
Keyboard.press(KEY_RIGHT_ARROW);
Keyboard.releaseAll();
} else if (function == "Back") {
Keyboard.press(KEY_LEFT_ALT);
Keyboard.press(KEY_LEFT_ARROW);
Keyboard.releaseAll();
} else if (function == "Copy") {
Keyboard.press(KEY_LEFT_CTRL);
Keyboard.press('c');
Keyboard.releaseAll();
} else if (function == "Paste") {
Keyboard.press(KEY_LEFT_CTRL);
Keyboard.press('v');
Keyboard.releaseAll();
} else if (function == "Ctrl") {
Keyboard.press(KEY_LEFT_CTRL);
} else if (function == "Alt") {
Keyboard.press(KEY_LEFT_ALT);
} else if (function == "Shift") {
Keyboard.press(KEY_LEFT_SHIFT);
} else {
pressCustomCombination(function);
}
}

void releaseButtonFunction(int buttonIndex) {
String function = buttonFunctions[buttonIndex];
if (function == "LeftClick") {
Mouse.release(MOUSE_LEFT);
} else if (function == "RightClick") {
Mouse.release(MOUSE_RIGHT);
} else if (function == "MiddleClick") {
Mouse.release(MOUSE_MIDDLE);
} else if (function == "Ctrl") {
Keyboard.release(KEY_LEFT_CTRL);
} else if (function == "Alt") {
Keyboard.release(KEY_LEFT_ALT);
} else if (function == "Shift") {
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
} else if (token.equalsIgnoreCase("SHIFT")) {
Keyboard.press(KEY_LEFT_SHIFT);
} else if (token.equalsIgnoreCase("ALT")) {
Keyboard.press(KEY_LEFT_ALT);
} else if (token.equalsIgnoreCase("GUI") || token.equalsIgnoreCase("WIN")) {
Keyboard.press(KEY_LEFT_GUI);
} else {
if (token.length() == 1) {
char c = token.charAt(0);
if (!shiftSpecified && ((c >= 'A' && c <= 'Z') || (c >= 'А' && c <= 'Я'))) {
c = tolower(c);
}
Keyboard.press(c);
} else {
if (token.equalsIgnoreCase("ENTER")) {
Keyboard.press(KEY_ENTER);
} else if (token.equalsIgnoreCase("ESC")) {
Keyboard.press(KEY_ESC);
} else if (token.equalsIgnoreCase("F1")) {
Keyboard.press(KEY_F1);
} else if (token.equalsIgnoreCase("F2")) {
Keyboard.press(KEY_F2);
}
}
}
}
delay(50);
Keyboard.releaseAll();
}

void handleScroll() {
int16_t ax2, ay2, az2;
mpu2.getAcceleration(&ax2, &ay2, &az2);
ax2 -= offset_ax2;
ay2 -= offset_ay2;
az2 -= offset_az2;
float rawAngle = (float)ay2 / 16384.0 * 90.0;
static float emaScrollAngle = 0.0;
float alphaScroll = 0.15; 
emaScrollAngle = alphaScroll * rawAngle + (1 - alphaScroll) * emaScrollAngle;
float angleAbs = fabs(emaScrollAngle);
if (angleAbs > MIN_ANGLE) {
int direction = (emaScrollAngle > 0) ? 1 : -1;
float normalizedAngle = (angleAbs - MIN_ANGLE) / (float)(MAX_ANGLE - MIN_ANGLE);
normalizedAngle = constrain(normalizedAngle, 0.0, 1.0);
float logistic = 1.0 / (1.0 + exp(-12 * (normalizedAngle - 0.5)));
float baseSpeed = 0.03 + (scrollSpeed - 0.03) * logistic;

scrollAccumulator += baseSpeed * direction * INVERT_SCROLL * 0.2;

} else {
scrollAccumulator *= 0.9; 
if (fabs(scrollAccumulator) < 0.001) {
scrollAccumulator = 0.0;
}
}
int steps = (int)floor(scrollAccumulator);
if (steps != 0) {
Mouse.move(0, 0, steps);
scrollAccumulator -= steps;
}
}

void handleEncoder() {
int aState = digitalRead(pinVolumeEncoderA);
int bState = digitalRead(pinVolumeEncoderB);
if (aState != lastVolumeStateA) {
if (bState != aState) {
volumePosition++;
switch (currentEncoderMode) {
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
switch (currentEncoderMode) {
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

void handleSerialInput() {
if (Serial.available()) {
String command = Serial.readStringUntil('\n');
command.trim();
bool settingsChanged = false;

if (command.startsWith("CURSOR_SPEED:")) {
  cursorSpeed = command.substring(13).toInt();
  cursorSpeed = constrain(cursorSpeed, 1, 10);
  settingsChanged = true;
} else if (command.startsWith("SCROLL_SPEED:")) {
  scrollSpeed = command.substring(13).toInt();
  scrollSpeed = constrain(scrollSpeed, 1, 10);
  settingsChanged = true;
} else if (command.startsWith("PRECISION_FACTOR:")) {
  precisionFactor = command.substring(17).toFloat();
  precisionFactor = constrain(precisionFactor, 0.1, 0.9);
  EEPROM.put(EEPROM_PRECISION_FACTOR, precisionFactor);
  settingsChanged = true;
  Serial.print("PRECISION_FACTOR_SET:");
  Serial.println(precisionFactor);
} else if (command.startsWith("PRECISION:")) {
  String mode = command.substring(10);
  mode.trim();
  if (mode.equalsIgnoreCase("ON")) {
    precisionMode = true;
    Serial.println("Precision mode enabled");
  } else if (mode.equalsIgnoreCase("OFF")) {
    precisionMode = false;
    Serial.println("Precision mode disabled");
  } else if (mode.equalsIgnoreCase("TOGGLE")) {
    precisionMode = !precisionMode;
    Serial.print("Precision mode toggled to: ");
    Serial.println(precisionMode ? "ON" : "OFF");
  }
  settingsChanged = true;
} else if (command.startsWith("BTN:")) {
  command = command.substring(4);
  int firstColon = command.indexOf(':');
  int secondColon = command.indexOf(':', firstColon + 1);

  if (firstColon != -1 && secondColon != -1) {
    int buttonPin = command.substring(0, firstColon).toInt();
    String function = command.substring(firstColon + 1, secondColon);

    int buttonIndex = -1;
    int pins[] = {
      pinLeftButton,
      pinRightButton,
      pinMiddleButton,
      pinForwardButton,
      pinBackButton,
      pinCopyButton,
      pinPasteButton
    };

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
} else if (command == "RESET:ALL") {
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

void debugOutput() {
Serial.print("CursorSpeed: ");
Serial.print(cursorSpeed);
Serial.print(" | ScrollSpeed: ");
Serial.print(scrollSpeed);
Serial.print(" | PrecisionFactor: ");
Serial.print(precisionFactor);
Serial.print(" | PrecisionMode: ");
Serial.print(precisionMode ? "ON" : "OFF");
Serial.print(" | Idle: ");
Serial.print(isCursorIdle);
Serial.print(" | EMA_AX: ");
Serial.print(emaAx);
Serial.print(" | EMA_AY: ");
Serial.println(emaAy);
}

void setup() {
Wire.begin();
Mouse.begin();
Keyboard.begin();
Consumer.begin();
Serial.begin(115200);
Serial.println("Initializing MPU sensors...");
mpu1.initialize();
mpu2.initialize();
Serial.println("MPU initialization complete.");
delay(1000);
loadAllSettings();
pinMode(pinLeftButton, INPUT_PULLUP);
pinMode(pinRightButton, INPUT_PULLUP);
pinMode(pinMiddleButton, INPUT_PULLUP);
pinMode(pinForwardButton, INPUT_PULLUP);
pinMode(pinBackButton, INPUT_PULLUP);
pinMode(pinCopyButton, INPUT_PULLUP);
pinMode(pinPasteButton, INPUT_PULLUP);
pinMode(pinVolumeEncoderA, INPUT_PULLUP);
pinMode(pinVolumeEncoderB, INPUT_PULLUP);
calibrateSensors();
}

void loop() {
processMouseMovement();
handleButtons();
handleScroll();
handleEncoder();
if (Serial.available() > 0) {
handleSerialInput();
}
if (millis() - lastDebugTime > DEBUG_INTERVAL) {
debugOutput();
lastDebugTime = millis();
}
delay(1);
}
