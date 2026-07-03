#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Update.h>

// MARK: - Stable 33.33 BLE Protocol

static const char *SERVICE_UUID =
  "4fafc201-1fb5-459e-8fcc-c5c9c331914b";

static const char *CHAR_COMMAND_UUID =
  "beb5483e-36e1-4688-b7f5-ea07361b26a8";

static const char *CHAR_POSITION_UUID =
  "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e";

static const char *CHAR_DEVICE_INFO_UUID =
  "7a7c4b3a-6d2c-4e1f-9f3b-333300000001";

static const char *CHAR_TIMER_UUID =
  "7a7c4b3a-6d2c-4e1f-9f3b-333300000002";

static const char *CHAR_SETTINGS_UUID =
  "7a7c4b3a-6d2c-4e1f-9f3b-333300000003";

static const char *CHAR_OTA_CONTROL_UUID =
  "7a7c4b3a-6d2c-4e1f-9f3b-333300000010";

static const char *CHAR_OTA_DATA_UUID =
  "7a7c4b3a-6d2c-4e1f-9f3b-333300000011";

static const char *FIRMWARE_VERSION = "1.0.0-ble";

// MARK: - Servo Settings

int upPosition = 180;
int downPosition = 95;
int pos = upPosition;

int stepDelayMs = 21;
int servoMinUs = 500;
int servoMaxUs = 2500;

unsigned long inputDelayMs = 500;
unsigned long centerDebounceMs = 180;
unsigned long holdStartMs = 550;
unsigned long holdRepeatMs = 85;

// MARK: - ESP32-C3 Supermini Pins

int servoPin = 6;
int upPin = 1;
int downPin = 3;
int sleepPin = 2;
int sensorPin = 5;

bool sensorActiveLow = true;

// MARK: - OLED

int oledSdaPin = 8;
int oledSclPin = 9;
uint8_t oledAddress = 0x3C;

Servo myservo;
Preferences memory;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

BLEServer *bleServer = nullptr;
BLECharacteristic *commandCharacteristic = nullptr;
BLECharacteristic *positionCharacteristic = nullptr;
BLECharacteristic *deviceInfoCharacteristic = nullptr;
BLECharacteristic *timerCharacteristic = nullptr;
BLECharacteristic *settingsCharacteristic = nullptr;
BLECharacteristic *otaControlCharacteristic = nullptr;
BLECharacteristic *otaDataCharacteristic = nullptr;

bool bleClientConnected = false;
String deviceID;
String deviceName;

bool otaInProgress = false;
size_t otaExpectedBytes = 0;
size_t otaWrittenBytes = 0;
int otaLastProgressPercent = -1;

bool servoAttached = false;
int lastSavedPos = -999;

bool lastUp = false;
bool lastDown = false;
bool lastSleep = false;
bool lastSensor = false;

unsigned long lastAcceptedInputAt = 0;
unsigned long lastCenterEdgeAt = 0;
unsigned long centerPressedAt = 0;
bool centerLongHandled = false;
unsigned long centerSettingsHoldMs = 1200;

bool timedDownActive = false;
unsigned long timedDownEndsAt = 0;
bool timerEditMode = false;
unsigned long timerSetSeconds = 0;

unsigned long upHoldStartedAt = 0;
unsigned long downHoldStartedAt = 0;
unsigned long lastHoldRepeatAt = 0;

bool displayReady = false;
unsigned long lastDisplayAt = 0;
unsigned long lastActivityAt = 0;
unsigned long displaySleepTimeoutSeconds = 600;
bool displaySleeping = false;
uint8_t displayContrast = 255;
bool showSkuOnMain = true;

enum DisplayPage {
  DISPLAY_PAGE_MAIN = 0,
  DISPLAY_PAGE_SETTINGS_STATUS = 1,
  DISPLAY_PAGE_SETTINGS_PRODUCT = 2,
  DISPLAY_PAGE_SETTINGS_DISPLAY = 3
};

enum DisplayLanguage {
  LANGUAGE_EN = 0,
  LANGUAGE_FR = 1
};

int displayPage = DISPLAY_PAGE_MAIN;
int displayLanguage = LANGUAGE_EN;
String displayCountry = "US";
int settingsFieldIndex = 0;
bool settingsEditMode = false;

int motionMode = 0; // 0 none, 1 UP, -1 DOWN
int animationFrame = 0;

String currentPositionState = "UNKNOWN";

// MARK: - Helpers

void logLine(const String &message) {
  Serial.print('[');
  Serial.print(millis());
  Serial.print(F(" ms] "));
  Serial.println(message);
}

String buildDeviceID() {
  uint64_t mac = ESP.getEfuseMac();
  char buffer[17];
  snprintf(
    buffer,
    sizeof(buffer),
    "%04X%08X",
    (uint16_t)(mac >> 32),
    (uint32_t)mac
  );
  return String(buffer);
}

String shortDeviceSuffix() {
  if (deviceID.length() <= 6) {
    return deviceID;
  }

  return deviceID.substring(deviceID.length() - 6);
}

bool isPressed(int pin) {
  return digitalRead(pin) == HIGH;
}

bool sensorActive() {
  bool rawHigh = digitalRead(sensorPin) == HIGH;
  return sensorActiveLow ? !rawHigh : rawHigh;
}

bool inputAllowed(const char *source) {
  const unsigned long now = millis();

  if (lastAcceptedInputAt != 0 && (now - lastAcceptedInputAt) < inputDelayMs) {
    logLine(String(source) + " ignore: delai anti-doubles");
    return false;
  }

  lastAcceptedInputAt = now;
  return true;
}

void notifyCharacteristic(BLECharacteristic *characteristic, const String &value) {
  if (characteristic == nullptr) {
    return;
  }

  characteristic->setValue(value.c_str());

  if (bleClientConnected) {
    characteristic->notify();
  }
}

void notifyPosition(const String &state) {
  currentPositionState = state;
  notifyCharacteristic(positionCharacteristic, state);
  logLine("BLE position -> " + state);
}

unsigned long remainingTimerSeconds() {
  if (!timedDownActive) {
    return timerSetSeconds;
  }

  long remainingMs = (long)(timedDownEndsAt - millis());
  if (remainingMs <= 0) {
    return 0;
  }

  return (remainingMs + 999) / 1000;
}

String timerPayload() {
  String payload = "{";
  payload += "\"setSeconds\":";
  payload += String(timerSetSeconds);
  payload += ",\"remainingSeconds\":";
  payload += String(remainingTimerSeconds());
  payload += ",\"active\":";
  payload += timedDownActive ? "true" : "false";
  payload += ",\"editMode\":";
  payload += timerEditMode ? "true" : "false";
  payload += "}";
  return payload;
}

String deviceInfoPayload() {
  String payload = "{";
  payload += "\"id\":\"";
  payload += deviceID;
  payload += "\",\"name\":\"";
  payload += deviceName;
  payload += "\",\"firmware\":\"";
  payload += FIRMWARE_VERSION;
  payload += "\",";
  payload += "\"service\":\"";
  payload += SERVICE_UUID;
  payload += "\"}";
  return payload;
}

void notifyTimer() {
  notifyCharacteristic(timerCharacteristic, timerPayload());
}

String displayPageName() {
  if (displayPage == DISPLAY_PAGE_SETTINGS_STATUS) {
    return "status";
  }
  if (displayPage == DISPLAY_PAGE_SETTINGS_PRODUCT) {
    return "product";
  }
  if (displayPage == DISPLAY_PAGE_SETTINGS_DISPLAY) {
    return "display";
  }
  return "main";
}

String languageName() {
  return displayLanguage == LANGUAGE_EN ? "en" : "fr";
}

String localeName() {
  return languageName() + "-" + displayCountry;
}

bool isFrenchLocale() {
  return displayLanguage == LANGUAGE_FR;
}

bool isEnglishLocale() {
  return displayLanguage == LANGUAGE_EN;
}

bool isCanadianLocale() {
  return displayCountry == "CA";
}

bool isBritishLocale() {
  return displayCountry == "GB";
}

bool isAustralianLocale() {
  return displayCountry == "AU";
}

const __FlashStringHelper *textPlayingTime() {
  if (isEnglishLocale()) {
    return F("Playing Time");
  }

  return F("Temps lecture");
}

const __FlashStringHelper *textSettings() {
  return isEnglishLocale() ? F("Settings") : F("Parametres");
}

const __FlashStringHelper *textStatus() {
  return isEnglishLocale() ? F("Status") : F("Statut");
}

const __FlashStringHelper *textInfo() {
  return isEnglishLocale() ? F("Info") : F("Infos");
}

const __FlashStringHelper *textDisplay() {
  if (isEnglishLocale() && (isBritishLocale() || isAustralianLocale())) {
    return F("Screen");
  }

  return isEnglishLocale() ? F("Display") : F("Ecran");
}

const __FlashStringHelper *textConnected() {
  return isEnglishLocale() ? F("connected") : F("connecte");
}

const __FlashStringHelper *textWaiting() {
  return isEnglishLocale() ? F("waiting") : F("attente");
}

const __FlashStringHelper *textAdjust() {
  return isEnglishLocale() ? F("UP/DOWN adjust") : F("UP/DOWN regle");
}

const __FlashStringHelper *textTimerActive() {
  return isEnglishLocale() ? F("Timer active") : F("Minuterie ON");
}

String textLanguageLabel() {
  return isFrenchLocale() ? "Langue" : "Language";
}

String textCountryLabel() {
  return isFrenchLocale() ? "Pays" : "Country";
}

String textSleepDelayLabel() {
  if (isFrenchLocale() && isCanadianLocale()) {
    return "Delai veille";
  }

  return isFrenchLocale() ? "Veille ecran" : "Sleep delay";
}

String textContrastLabel() {
  return isFrenchLocale() ? "Contraste" : "Contrast";
}

String textMainSkuLabel() {
  return isFrenchLocale() ? "SKU accueil" : "Home SKU";
}

String textSettingsProductFooter() {
  return isFrenchLocale() ? "OK suiv APP long" : "OK next HOLD page";
}

String textSettingsEditFooter() {
  return isFrenchLocale() ? "UP/DN reg OK" : "UP/DN set OK";
}

String textSettingsNavFooter() {
  return isFrenchLocale() ? "UP/DN nav OK reg" : "UP/DN nav OK set";
}

String settingsPayload() {
  String payload = "{";
  payload += "\"language\":\"";
  payload += languageName();
  payload += "\",\"country\":\"";
  payload += displayCountry;
  payload += "\",\"locale\":\"";
  payload += localeName();
  payload += "\",\"displayPage\":\"";
  payload += displayPageName();
  payload += "\",\"settingsFieldIndex\":";
  payload += String(settingsFieldIndex);
  payload += ",\"settingsEditMode\":";
  payload += settingsEditMode ? "true" : "false";
  payload += ",\"displaySleepTimeoutSeconds\":";
  payload += String(displaySleepTimeoutSeconds);
  payload += ",\"displayContrast\":";
  payload += String(displayContrast);
  payload += ",\"showSkuOnMain\":";
  payload += showSkuOnMain ? "true" : "false";
  payload += ",\"displaySleeping\":";
  payload += displaySleeping ? "true" : "false";
  payload += "}";
  return payload;
}

void notifySettings() {
  notifyCharacteristic(settingsCharacteristic, settingsPayload());
}

void notifyAllStatus() {
  notifyPosition(currentPositionState);
  notifyTimer();
  notifyCharacteristic(deviceInfoCharacteristic, deviceInfoPayload());
  notifySettings();
}

// MARK: - OLED

void applyDisplayContrast() {
  if (displayReady) {
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(displayContrast);
  }
}

void formatTime(unsigned long seconds, char *buffer, size_t size) {
  unsigned long minutes = seconds / 60;
  unsigned long secs = seconds % 60;
  snprintf(buffer, size, "%02lu:%02lu", minutes, secs);
}

void wakeDisplay(const char *source) {
  lastActivityAt = millis();

  if (!displaySleeping) {
    return;
  }

  displaySleeping = false;

  if (displayReady) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    updateDisplay(true);
  }

  notifySettings();
  logLine(String(source) + " -> reveil OLED");
}

void registerActivity(const char *source) {
  lastActivityAt = millis();

  if (displaySleeping) {
    wakeDisplay(source);
  }
}

void updateDisplaySleep() {
  if (!displayReady || displaySleeping || displaySleepTimeoutSeconds == 0) {
    return;
  }

  if ((millis() - lastActivityAt) >= (displaySleepTimeoutSeconds * 1000UL)) {
    displaySleeping = true;
    showSleepingDisplay();
    notifySettings();
    logLine(F("OLED en veille"));
  }
}

void drawBluetoothIcon(int x, int y) {
  display.drawLine(x, y - 6, x, y + 6, SSD1306_WHITE);
  display.drawLine(x, y - 6, x + 5, y - 2, SSD1306_WHITE);
  display.drawLine(x + 5, y - 2, x - 3, y + 4, SSD1306_WHITE);
  display.drawLine(x - 3, y - 4, x + 5, y + 2, SSD1306_WHITE);
  display.drawLine(x + 5, y + 2, x, y + 6, SSD1306_WHITE);
}

void drawBluetoothIconSmall(int x, int y) {
  display.drawLine(x, y - 4, x, y + 4, SSD1306_WHITE);
  display.drawLine(x, y - 4, x + 3, y - 1, SSD1306_WHITE);
  display.drawLine(x + 3, y - 1, x - 2, y + 3, SSD1306_WHITE);
  display.drawLine(x - 2, y - 3, x + 3, y + 1, SSD1306_WHITE);
  display.drawLine(x + 3, y + 1, x, y + 4, SSD1306_WHITE);
}

void showSleepingDisplay() {
  if (!displayReady) {
    return;
  }

  if (bleClientConnected) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    drawBluetoothIcon(119, 8);
    display.display();
  } else {
    display.clearDisplay();
    display.display();
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

void printCentered(const String &text, int y, int textSize = 1) {
  int width = text.length() * 6 * textSize;
  int x = max(0, (128 - width) / 2);
  display.setTextSize(textSize);
  display.setCursor(x, y);
  display.print(text);
}

int settingsFieldCount() {
  if (displayPage == DISPLAY_PAGE_SETTINGS_STATUS) {
    return 3;
  }
  if (displayPage == DISPLAY_PAGE_SETTINGS_DISPLAY) {
    return 3;
  }
  return 2;
}

void clampSettingsField() {
  int count = settingsFieldCount();
  if (settingsFieldIndex < 0) {
    settingsFieldIndex = count - 1;
  } else if (settingsFieldIndex >= count) {
    settingsFieldIndex = 0;
  }
}

void drawSettingRow(int row, const String &label, const String &value) {
  int y = 19 + (row * 10);
  bool selected = row == settingsFieldIndex;

  if (selected) {
    display.fillRect(0, y - 1, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }

  display.setTextSize(1);
  display.setCursor(2, y);
  display.print(label);
  display.setCursor(74, y);
  display.print(value);

  if (selected && settingsEditMode) {
    display.drawRect(72, y - 2, 54, 11, SSD1306_BLACK);
  }

  display.setTextColor(SSD1306_WHITE);
}

void drawSlider(int row, long value, long minValue, long maxValue) {
  int y = 29 + (row * 10);
  int filled = map(constrain(value, minValue, maxValue), minValue, maxValue, 0, 24);
  uint16_t color = row == settingsFieldIndex ? SSD1306_BLACK : SSD1306_WHITE;
  display.drawRect(100, y, 26, 4, color);
  display.fillRect(101, y + 1, filled, 2, color);
}

void drawLargeSlider(int y, long value, long minValue, long maxValue) {
  int filled = map(constrain(value, minValue, maxValue), minValue, maxValue, 0, 96);
  display.drawRect(16, y, 98, 7, SSD1306_WHITE);
  display.fillRect(17, y + 1, filled, 5, SSD1306_WHITE);
}

void drawSettingsFooter() {
  display.fillRect(0, 54, 128, 10, settingsEditMode ? SSD1306_WHITE : SSD1306_BLACK);
  display.setTextColor(settingsEditMode ? SSD1306_BLACK : SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 55);

  if (displayPage == DISPLAY_PAGE_SETTINGS_PRODUCT) {
    display.print(textSettingsProductFooter());
  } else if (settingsEditMode) {
    display.print(textSettingsEditFooter());
  } else {
    display.print(textSettingsNavFooter());
  }

  display.setTextColor(SSD1306_WHITE);
}

void drawMovementAnimation() {
  if (!displayReady) {
    return;
  }

  if (motionMode == 1) {
    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print(F("UP"));

    for (int i = 0; i < 4; i++) {
      int x = 42 + (i * 20);
      int y = 58 - ((animationFrame + (i * 2)) % 10);
      display.fillTriangle(x, y - 6, x - 5, y + 3, x + 5, y + 3, SSD1306_WHITE);
    }
  } else if (motionMode == -1) {
    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print(F("DOWN"));

    for (int i = 0; i < 4; i++) {
      int x = 48 + (i * 19);
      int y = 48 + ((animationFrame + (i * 2)) % 10);
      display.fillTriangle(x, y + 6, x - 5, y - 3, x + 5, y - 3, SSD1306_WHITE);
    }
  }
}

void drawSettingsPage() {
  clampSettingsField();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(textSettings());

  display.setCursor(86, 0);
  if (displayPage == DISPLAY_PAGE_SETTINGS_STATUS) {
    display.print(textStatus());
  } else if (displayPage == DISPLAY_PAGE_SETTINGS_PRODUCT) {
    display.print(textInfo());
  } else {
    display.print(textDisplay());
  }

  display.drawLine(0, 14, 127, 14, SSD1306_WHITE);

  if (displayPage == DISPLAY_PAGE_SETTINGS_STATUS) {
    drawSettingRow(0, textLanguageLabel(), languageName());
    drawSettingRow(1, textCountryLabel(), displayCountry);
    drawSettingRow(2, "BLE", bleClientConnected ? String(textConnected()) : String(textWaiting()));
  } else if (displayPage == DISPLAY_PAGE_SETTINGS_PRODUCT) {
    drawSettingRow(0, "SKU", "SPJP-5018");
    drawSettingRow(1, "ID", deviceID.substring(0, 8));
    display.setCursor(2, 41);
    display.print(F("FW "));
    display.print(FIRMWARE_VERSION);
  } else {
    display.setTextSize(1);
    display.setCursor(0, 20);

    if (settingsFieldIndex == 0) {
      display.print(textSleepDelayLabel());
      printCentered(String(displaySleepTimeoutSeconds) + " s", 31);
      drawLargeSlider(43, displaySleepTimeoutSeconds, 0, 3600);
    } else if (settingsFieldIndex == 1) {
      display.print(textContrastLabel());
      printCentered(String(displayContrast), 31);
      drawLargeSlider(43, displayContrast, 0, 255);
    } else {
      display.print(textMainSkuLabel());
      printCentered(showSkuOnMain ? "ON" : "OFF", 34, 2);
    }
  }

  drawSettingsFooter();
}

void drawMainPage(const char *timeText) {
  display.setTextSize(1);
  display.setCursor(0, 2);
  display.print(textPlayingTime());

  display.setCursor(82, 2);
  if (timerEditMode) {
    display.print(F("EDIT"));
  } else if (timedDownActive) {
    display.print(F("RUN"));
  } else if (bleClientConnected) {
    display.print(F("BLE"));
  } else {
    display.print(F("OK"));
  }

  if (bleClientConnected) {
    drawBluetoothIconSmall(113, 6);
  }

  display.drawCircle(124, 6, 3, SSD1306_WHITE);

  if (timerEditMode || timedDownActive || bleClientConnected) {
    display.fillCircle(124, 6, 3, SSD1306_WHITE);
  }

  display.drawLine(0, 16, 127, 16, SSD1306_WHITE);

  display.setTextSize(3);
  display.setCursor(18, 24);
  display.print(timeText);

  if (motionMode != 0) {
    drawMovementAnimation();
  } else if (timerEditMode) {
    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print(textAdjust());
  } else if (timedDownActive) {
    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print(textTimerActive());
  } else {
    if (showSkuOnMain) {
      printCentered("SPJP-5018 TEST2", 54);
    }
  }
}

void updateDisplay(bool force) {
  if (!displayReady) {
    return;
  }

  if (displaySleeping) {
    return;
  }

  const unsigned long now = millis();

  if (!force && (now - lastDisplayAt) < 180) {
    return;
  }

  lastDisplayAt = now;
  animationFrame++;

  char timeText[8];
  unsigned long shownSeconds = timedDownActive ? remainingTimerSeconds() : timerSetSeconds;
  formatTime(shownSeconds, timeText, sizeof(timeText));

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (displayPage == DISPLAY_PAGE_MAIN) {
    drawMainPage(timeText);
  } else {
    drawSettingsPage();
  }

  display.display();
}

void setupDisplay() {
  Wire.begin(oledSdaPin, oledSclPin);
  displayReady = display.begin(SSD1306_SWITCHCAPVCC, oledAddress);

  if (!displayReady) {
    logLine(F("OLED non detecte"));
    return;
  }

  display.clearDisplay();
  display.display();
  applyDisplayContrast();
  updateDisplay(true);
}

void waitWithDisplay(unsigned long durationMs) {
  const unsigned long start = millis();

  while ((millis() - start) < durationMs) {
    if (millis() - lastDisplayAt >= 180) {
      updateDisplay(false);
    }

    delay(1);
  }
}

// MARK: - Servo

void attachServoIfNeeded() {
  if (!servoAttached) {
    myservo.attach(servoPin, servoMinUs, servoMaxUs);
    servoAttached = true;
    logLine(F("Servo attache"));
  }
}

void detachServoIfNeeded() {
  if (servoAttached) {
    myservo.detach();
    servoAttached = false;
    pinMode(servoPin, OUTPUT);
    digitalWrite(servoPin, LOW);
    logLine(F("Servo detache"));
  }
}

void savePosition(bool force = false) {
  if (force || abs(pos - lastSavedPos) >= 5) {
    memory.putInt("pos", pos);
    lastSavedPos = pos;
  }
}

void saveConfig() {
  memory.putUInt("cfgv", 3);
  memory.putUInt("timer", timerSetSeconds);
  memory.putUInt("sleep", displaySleepTimeoutSeconds);
  memory.putInt("lang", displayLanguage);
  memory.putString("country", displayCountry);
  memory.putUChar("contrast", displayContrast);
  memory.putBool("showSku", showSkuOnMain);
}

void loadConfig() {
  memory.begin("servoctl", false);

  uint32_t configVersion = memory.getUInt("cfgv", 0);
  timerSetSeconds = constrain((long)memory.getUInt("timer", timerSetSeconds), 0L, 5999L);
  displaySleepTimeoutSeconds = constrain((long)memory.getUInt("sleep", displaySleepTimeoutSeconds), 0L, 3600L);
  displayLanguage = constrain(memory.getInt("lang", displayLanguage), LANGUAGE_EN, LANGUAGE_FR);
  displayCountry = memory.getString("country", displayCountry);
  displayCountry.toUpperCase();
  displayContrast = memory.getUChar("contrast", displayContrast);
  showSkuOnMain = memory.getBool("showSku", showSkuOnMain);

  if (configVersion < 3) {
    if (displaySleepTimeoutSeconds == 60) {
      displaySleepTimeoutSeconds = 600;
      logLine(F("Migration config: veille OLED par defaut -> 600 s"));
    }
    saveConfig();
  }

  pos = constrain(memory.getInt("pos", upPosition), 0, 180);
  lastSavedPos = pos;

  logLine(String(F("Position chargee: ")) + String(pos) + String(F(" deg")));
}

void goToPosition(int target, const char *source) {
  target = constrain(target, 0, 180);

  attachServoIfNeeded();

  motionMode = (target == upPosition) ? 1 : -1;
  notifyPosition("MOVING");
  updateDisplay(true);

  logLine(String(source) + " -> depart " + String(pos) + " deg, cible " + String(target) + " deg");

  while (pos != target) {
    if (target != upPosition && sensorActive()) {
      target = upPosition;
      motionMode = 1;
      timedDownActive = false;
      timerEditMode = false;
      notifyTimer();
      logLine(F("Capteur IR actif pendant mouvement -> retour UP"));
    }

    pos += (pos < target) ? 1 : -1;
    myservo.write(pos);
    savePosition();
    waitWithDisplay(stepDelayMs);
  }

  myservo.write(pos);
  savePosition(true);

  motionMode = 0;

  if (pos == upPosition) {
    notifyPosition("UP");
  } else if (pos == downPosition) {
    notifyPosition("DOWN");
  } else {
    notifyPosition("UNKNOWN");
  }

  updateDisplay(true);

  logLine(String(source) + " -> arrive a " + String(pos) + " deg");
}

void stopMotion(const char *source) {
  timedDownActive = false;
  timerEditMode = false;
  motionMode = 0;

  detachServoIfNeeded();
  notifyPosition("UNKNOWN");
  notifyTimer();
  updateDisplay(true);

  logLine(String(source) + " -> STOP");
}

// MARK: - Wireless Firmware Update

void notifyOtaControl(const String &state) {
  notifyCharacteristic(otaControlCharacteristic, state);
  logLine("OTA -> " + state);
}

void resetOtaState() {
  otaInProgress = false;
  otaExpectedBytes = 0;
  otaWrittenBytes = 0;
  otaLastProgressPercent = -1;
}

void beginOtaUpdate(size_t expectedBytes) {
  if (expectedBytes == 0) {
    notifyOtaControl("OTA_ERROR:EMPTY");
    return;
  }

  stopMotion("OTA");
  timedDownActive = false;
  timerEditMode = false;
  notifyTimer();
  updateDisplay(true);

  if (!Update.begin(expectedBytes)) {
    notifyOtaControl("OTA_ERROR:BEGIN");
    resetOtaState();
    return;
  }

  otaInProgress = true;
  otaExpectedBytes = expectedBytes;
  otaWrittenBytes = 0;
  otaLastProgressPercent = 0;
  notifyOtaControl("OTA_READY");
}

void abortOtaUpdate() {
  if (otaInProgress) {
    Update.abort();
  }
  resetOtaState();
  notifyOtaControl("OTA_ABORTED");
}

void finishOtaUpdate() {
  if (!otaInProgress) {
    notifyOtaControl("OTA_ERROR:NOT_STARTED");
    return;
  }

  if (otaWrittenBytes != otaExpectedBytes) {
    Update.abort();
    notifyOtaControl("OTA_ERROR:SIZE");
    resetOtaState();
    return;
  }

  if (!Update.end(true)) {
    notifyOtaControl("OTA_ERROR:END");
    resetOtaState();
    return;
  }

  notifyOtaControl("OTA_OK");
  delay(700);
  ESP.restart();
}

void handleOtaControl(String command) {
  command.trim();

  if (command.startsWith("OTA_BEGIN:")) {
    String sizeText = command.substring(String("OTA_BEGIN:").length());
    beginOtaUpdate((size_t)sizeText.toInt());
    return;
  }

  if (command == "OTA_END") {
    finishOtaUpdate();
    return;
  }

  if (command == "OTA_ABORT") {
    abortOtaUpdate();
    return;
  }

  notifyOtaControl("OTA_ERROR:COMMAND");
}

void handleOtaData(uint8_t *bytes, size_t length) {
  if (!otaInProgress) {
    notifyOtaControl("OTA_ERROR:NOT_READY");
    return;
  }

  if (length == 0) {
    return;
  }

  if (otaWrittenBytes + length > otaExpectedBytes) {
    Update.abort();
    notifyOtaControl("OTA_ERROR:OVERFLOW");
    resetOtaState();
    return;
  }

  size_t written = Update.write(bytes, length);
  if (written != length) {
    Update.abort();
    notifyOtaControl("OTA_ERROR:WRITE");
    resetOtaState();
    return;
  }

  otaWrittenBytes += written;

  int progress = otaExpectedBytes == 0 ? 0 : (int)((otaWrittenBytes * 100) / otaExpectedBytes);
  if (progress >= otaLastProgressPercent + 10 || progress == 100) {
    otaLastProgressPercent = progress;
    notifyOtaControl("OTA_PROGRESS:" + String(progress));
  }
}

// MARK: - Timer

void changeTimerSeconds(long delta) {
  long updated = (long)timerSetSeconds + delta;
  timerSetSeconds = constrain(updated, 0L, 5999L);

  updateDisplay(true);

  logLine(String(F("Timer regle a ")) + String(timerSetSeconds) + String(F(" seconde(s)")));
}

void setTimerSeconds(unsigned long seconds) {
  timerSetSeconds = constrain((long)seconds, 0L, 5999L);

  saveConfig();
  notifyTimer();
  updateDisplay(true);

  logLine(String(F("Timer defini a ")) + String(timerSetSeconds) + String(F(" seconde(s)")));
}

void cancelTimer(const char *source) {
  timedDownActive = false;
  timerEditMode = false;

  notifyTimer();
  updateDisplay(true);

  logLine(String(source) + " -> timer annule");
}

void startTimer(const char *source) {
  saveConfig();

  if (timerSetSeconds == 0) {
    timedDownActive = false;
    notifyTimer();
    logLine(String(source) + " -> timer a 0 seconde");
    return;
  }

  timedDownActive = true;
  timedDownEndsAt = millis() + (timerSetSeconds * 1000UL);

  notifyTimer();

  logLine(String(source) + " -> timer demarre: DOWN pendant " + String(timerSetSeconds) + " seconde(s)");

  goToPosition(downPosition, source);
}

void toggleTimerEditMode() {
  const unsigned long now = millis();

  if (lastCenterEdgeAt != 0 && (now - lastCenterEdgeAt) < centerDebounceMs) {
    logLine(F("Bouton central ignore: anti-rebond"));
    return;
  }

  lastCenterEdgeAt = now;

  if (!timerEditMode) {
    timedDownActive = false;
    timerEditMode = true;
    logLine(F("Mode edition timer actif"));
    updateDisplay(true);
  } else {
    timerEditMode = false;
    logLine(F("Mode edition timer inactif"));
    updateDisplay(true);
    startTimer("D2 / MINUTERIE");
  }
}

void cycleSettingsPageFromRemote() {
  settingsEditMode = false;
  settingsFieldIndex = 0;

  if (displayPage == DISPLAY_PAGE_MAIN) {
    displayPage = DISPLAY_PAGE_SETTINGS_STATUS;
  } else if (displayPage == DISPLAY_PAGE_SETTINGS_STATUS) {
    displayPage = DISPLAY_PAGE_SETTINGS_PRODUCT;
  } else if (displayPage == DISPLAY_PAGE_SETTINGS_PRODUCT) {
    displayPage = DISPLAY_PAGE_SETTINGS_DISPLAY;
  } else {
    displayPage = DISPLAY_PAGE_MAIN;
  }

  timerEditMode = false;
  registerActivity("Settings telecommande");
  updateDisplay(true);
  notifySettings();
  logLine(String(F("Page settings -> ")) + displayPageName());
}

void cycleCountry() {
  if (displayLanguage == LANGUAGE_EN) {
    if (displayCountry == "US") {
      displayCountry = "CA";
    } else if (displayCountry == "CA") {
      displayCountry = "GB";
    } else if (displayCountry == "GB") {
      displayCountry = "AU";
    } else {
      displayCountry = "US";
    }
  } else {
    if (displayCountry == "CA") {
      displayCountry = "FR";
    } else if (displayCountry == "FR") {
      displayCountry = "BE";
    } else if (displayCountry == "BE") {
      displayCountry = "CH";
    } else {
      displayCountry = "CA";
    }
  }
}

void handleSettingsRemoteButton(bool upButton) {
  registerActivity(upButton ? "Settings UP" : "Settings DOWN");

  if (!settingsEditMode) {
    settingsFieldIndex += upButton ? -1 : 1;
    clampSettingsField();
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (displayPage == DISPLAY_PAGE_SETTINGS_STATUS) {
    if (settingsFieldIndex == 0) {
      displayLanguage = displayLanguage == LANGUAGE_EN ? LANGUAGE_FR : LANGUAGE_EN;
      displayCountry = displayLanguage == LANGUAGE_EN ? "US" : "CA";
    } else if (settingsFieldIndex == 1) {
      cycleCountry();
    }
  } else if (displayPage == DISPLAY_PAGE_SETTINGS_DISPLAY) {
    if (settingsFieldIndex == 0) {
      displaySleepTimeoutSeconds = constrain((long)displaySleepTimeoutSeconds + (upButton ? 15L : -15L), 0L, 3600L);
    } else if (settingsFieldIndex == 1) {
      displayContrast = constrain((long)displayContrast + (upButton ? 15L : -15L), 0L, 255L);
      applyDisplayContrast();
    } else if (settingsFieldIndex == 2) {
      showSkuOnMain = !showSkuOnMain;
    }
  }

  saveConfig();
  updateDisplay(true);
  notifySettings();
}

void handleSettingsCenterPress() {
  registerActivity("Settings OK");

  if (displayPage == DISPLAY_PAGE_SETTINGS_PRODUCT) {
    cycleSettingsPageFromRemote();
    return;
  }

  settingsEditMode = !settingsEditMode;
  saveConfig();
  updateDisplay(true);
  notifySettings();
}

void updateTimerHold(bool upNow, bool downNow) {
  if (!timerEditMode) {
    upHoldStartedAt = 0;
    downHoldStartedAt = 0;
    return;
  }

  const unsigned long now = millis();

  if (upNow) {
    if (upHoldStartedAt == 0) {
      upHoldStartedAt = now;
      lastHoldRepeatAt = now;
    } else if ((now - upHoldStartedAt) >= holdStartMs && (now - lastHoldRepeatAt) >= holdRepeatMs) {
      lastHoldRepeatAt = now;
      changeTimerSeconds(10);
    }
  } else {
    upHoldStartedAt = 0;
  }

  if (downNow) {
    if (downHoldStartedAt == 0) {
      downHoldStartedAt = now;
      lastHoldRepeatAt = now;
    } else if ((now - downHoldStartedAt) >= holdStartMs && (now - lastHoldRepeatAt) >= holdRepeatMs) {
      lastHoldRepeatAt = now;
      changeTimerSeconds(-10);
    }
  } else {
    downHoldStartedAt = 0;
  }
}

void updateTimedDown() {
  if (timedDownActive && (long)(millis() - timedDownEndsAt) >= 0) {
    timedDownActive = false;
    notifyTimer();

    logLine(F("Minuterie terminee -> retour UP"));
    goToPosition(upPosition, "Fin minuterie");
  }
}

// MARK: - BLE Command Protocol

void handleCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (otaInProgress) {
    notifyPosition("UPDATING");
    return;
  }

  logLine("BLE command <- " + command);

  if (command == "UP") {
    registerActivity("BLE UP");
    cancelTimer("BLE UP");
    goToPosition(upPosition, "BLE UP");
    return;
  }

  if (command == "DOWN") {
    registerActivity("BLE DOWN");
    cancelTimer("BLE DOWN");
    goToPosition(downPosition, "BLE DOWN");
    return;
  }

  if (command == "STOP") {
    registerActivity("BLE STOP");
    stopMotion("BLE STOP");
    return;
  }

  if (command == "GET_STATUS") {
    notifyAllStatus();
    return;
  }

  if (command == "SCREEN_WAKE") {
    wakeDisplay("BLE SCREEN_WAKE");
    notifySettings();
    return;
  }

  if (command == "SCREEN_MAIN") {
    displayPage = DISPLAY_PAGE_MAIN;
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command == "SETTINGS_STATUS") {
    displayPage = DISPLAY_PAGE_SETTINGS_STATUS;
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command == "SETTINGS_PRODUCT") {
    displayPage = DISPLAY_PAGE_SETTINGS_PRODUCT;
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command == "SETTINGS_DISPLAY") {
    displayPage = DISPLAY_PAGE_SETTINGS_DISPLAY;
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command == "SETTINGS_NEXT") {
    if (displayPage == DISPLAY_PAGE_MAIN) {
      displayPage = DISPLAY_PAGE_SETTINGS_STATUS;
    } else if (displayPage == DISPLAY_PAGE_SETTINGS_STATUS) {
      displayPage = DISPLAY_PAGE_SETTINGS_PRODUCT;
    } else if (displayPage == DISPLAY_PAGE_SETTINGS_PRODUCT) {
      displayPage = DISPLAY_PAGE_SETTINGS_DISPLAY;
    } else {
      displayPage = DISPLAY_PAGE_MAIN;
    }
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command.startsWith("SETTINGS_FIELD:")) {
    String value = command.substring(String("SETTINGS_FIELD:").length());
    settingsFieldIndex = value.toInt();
    clampSettingsField();
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command == "SETTINGS_EDIT") {
    settingsEditMode = true;
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command == "SETTINGS_CONFIRM") {
    settingsEditMode = false;
    saveConfig();
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command.startsWith("SET_LANG:")) {
    String value = command.substring(String("SET_LANG:").length());
    displayLanguage = value == "EN" ? LANGUAGE_EN : LANGUAGE_FR;
    saveConfig();
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command.startsWith("SET_COUNTRY:")) {
    displayCountry = command.substring(String("SET_COUNTRY:").length());
    displayCountry.toUpperCase();
    if (displayCountry.length() == 0) {
      displayCountry = displayLanguage == LANGUAGE_EN ? "US" : "CA";
    }
    saveConfig();
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command.startsWith("SET_LOCALE:")) {
    String value = command.substring(String("SET_LOCALE:").length());
    value.toUpperCase();
    int separator = value.indexOf('-');
    if (separator < 0) {
      separator = value.indexOf('_');
    }

    String lang = separator >= 0 ? value.substring(0, separator) : value;
    String country = separator >= 0 ? value.substring(separator + 1) : "";

    displayLanguage = lang == "FR" ? LANGUAGE_FR : LANGUAGE_EN;
    if (country.length() > 0) {
      displayCountry = country;
    } else {
      displayCountry = displayLanguage == LANGUAGE_EN ? "US" : "CA";
    }

    saveConfig();
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command.startsWith("SET_DISPLAY_SLEEP:")) {
    String value = command.substring(String("SET_DISPLAY_SLEEP:").length());
    displaySleepTimeoutSeconds = constrain((long)value.toInt(), 0L, 3600L);
    saveConfig();
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command.startsWith("SET_DISPLAY_CONTRAST:")) {
    String value = command.substring(String("SET_DISPLAY_CONTRAST:").length());
    displayContrast = constrain((long)value.toInt(), 0L, 255L);
    applyDisplayContrast();
    saveConfig();
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command.startsWith("SET_SHOW_SKU:")) {
    String value = command.substring(String("SET_SHOW_SKU:").length());
    showSkuOnMain = value == "1" || value == "TRUE" || value == "ON" || value == "YES";
    saveConfig();
    updateDisplay(true);
    notifySettings();
    return;
  }

  if (command == "TIMER_START") {
    registerActivity("BLE TIMER_START");
    timerEditMode = false;
    startTimer("BLE TIMER_START");
    return;
  }

  if (command == "TIMER_CANCEL") {
    cancelTimer("BLE TIMER_CANCEL");
    return;
  }

  if (command == "TIMER_EDIT") {
    timerEditMode = true;
    timedDownActive = false;
    notifyTimer();
    updateDisplay(true);
    return;
  }

  if (command.startsWith("TIMER_SET:")) {
    String value = command.substring(String("TIMER_SET:").length());
    setTimerSeconds(value.toInt());
    return;
  }

  notifyPosition("UNKNOWN");
}

class StableCommandCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String command = characteristic->getValue().c_str();

    if (command.length() == 0) {
      return;
    }

    handleCommand(command);
  }
};

class StableOtaControlCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String command = characteristic->getValue().c_str();

    if (command.length() == 0) {
      return;
    }

    handleOtaControl(command);
  }
};

class StableOtaDataCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    uint8_t *data = characteristic->getData();
    size_t length = characteristic->getLength();

    if (data == nullptr || length == 0) {
      return;
    }

    handleOtaData(data, length);
  }
};

class StableServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleClientConnected = true;
    logLine(F("BLE client connecte"));

    if (displaySleeping) {
      showSleepingDisplay();
      notifySettings();
    } else {
      registerActivity("BLE connect");
      updateDisplay(true);
    }

    notifyAllStatus();
  }

  void onDisconnect(BLEServer *server) override {
    bleClientConnected = false;
    logLine(F("BLE client deconnecte"));

    if (displaySleeping) {
      showSleepingDisplay();
      notifySettings();
    } else {
      registerActivity("BLE disconnect");
      updateDisplay(true);
    }

    delay(250);
    server->startAdvertising();
  }
};

void setupBLE() {
  deviceID = buildDeviceID();
  deviceName = "Stable 33.33-" + shortDeviceSuffix();

  BLEDevice::init(deviceName.c_str());

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new StableServerCallbacks());

  BLEService *service = bleServer->createService(SERVICE_UUID);

  commandCharacteristic = service->createCharacteristic(
    CHAR_COMMAND_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  commandCharacteristic->setCallbacks(new StableCommandCallbacks());

  positionCharacteristic = service->createCharacteristic(
    CHAR_POSITION_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  positionCharacteristic->addDescriptor(new BLE2902());
  positionCharacteristic->setValue(currentPositionState.c_str());

  deviceInfoCharacteristic = service->createCharacteristic(
    CHAR_DEVICE_INFO_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  deviceInfoCharacteristic->addDescriptor(new BLE2902());
  deviceInfoCharacteristic->setValue(deviceInfoPayload().c_str());

  timerCharacteristic = service->createCharacteristic(
    CHAR_TIMER_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  timerCharacteristic->addDescriptor(new BLE2902());
  timerCharacteristic->setValue(timerPayload().c_str());

  settingsCharacteristic = service->createCharacteristic(
    CHAR_SETTINGS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  settingsCharacteristic->addDescriptor(new BLE2902());
  settingsCharacteristic->setValue(settingsPayload().c_str());

  otaControlCharacteristic = service->createCharacteristic(
    CHAR_OTA_CONTROL_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  otaControlCharacteristic->addDescriptor(new BLE2902());
  otaControlCharacteristic->setCallbacks(new StableOtaControlCallbacks());
  otaControlCharacteristic->setValue("OTA_IDLE");

  otaDataCharacteristic = service->createCharacteristic(
    CHAR_OTA_DATA_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  otaDataCharacteristic->setCallbacks(new StableOtaDataCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  logLine("BLE pret: " + deviceName);
  logLine("Device ID: " + deviceID);
}

// MARK: - Arduino Setup / Loop

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(upPin, INPUT_PULLDOWN);
  pinMode(downPin, INPUT_PULLDOWN);
  pinMode(sleepPin, INPUT_PULLDOWN);
  pinMode(sensorPin, INPUT_PULLUP);

  deviceID = buildDeviceID();
  deviceName = "Stable 33.33-" + shortDeviceSuffix();

  setupDisplay();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myservo.setPeriodHertz(50);

  loadConfig();
  applyDisplayContrast();
  lastActivityAt = millis();
  setupBLE();

  logLine(F("Demarrage termine"));
  logLine(String(F("Servo: GPIO")) + String(servoPin) +
          String(F(", min/max=")) + String(servoMinUs) + String(F("/")) + String(servoMaxUs) +
          String(F(" us, retour demarrage vers UP=")) + String(upPosition) + String(F(" deg")));
  logLine(String(F("RX480: UP GPIO")) + String(upPin) +
          String(F(", DOWN GPIO")) + String(downPin) +
          String(F(", MINUTERIE GPIO")) + String(sleepPin));
  logLine(String(F("Capteur IR: OUT GPIO")) + String(sensorPin) +
          String(sensorActiveLow ? F(", actif LOW") : F(", actif HIGH")));

  notifyPosition("UNKNOWN");
  goToPosition(upPosition, "Demarrage / retour UP");
}

void loop() {
  if (otaInProgress) {
    updateDisplay(false);
    delay(1);
    return;
  }

  bool upNow = isPressed(upPin);
  bool downNow = isPressed(downPin);
  bool sleepNow = isPressed(sleepPin);
  bool sensorNow = sensorActive();

  if (sensorNow != lastSensor) {
    lastSensor = sensorNow;
    logLine(String(F("Capteur IR -> ")) + (sensorNow ? F("actif") : F("inactif")));

    if (sensorNow && inputAllowed("Capteur IR")) {
      registerActivity("Capteur IR");
      cancelTimer("Capteur IR");
      goToPosition(upPosition, "Capteur IR");
    }
  }

  if (displaySleeping &&
      ((upNow && !lastUp) || (downNow && !lastDown) || (sleepNow && !lastSleep))) {
    lastUp = upNow;
    lastDown = downNow;
    lastSleep = sleepNow;
    wakeDisplay("Manette");
    updateTimedDown();
    updateDisplay(false);
    return;
  }

  if (upNow != lastUp) {
    lastUp = upNow;
    logLine(String(F("UP -> ")) + (upNow ? F("actif") : F("inactif")));

    if (upNow && displayPage != DISPLAY_PAGE_MAIN && !timerEditMode) {
      handleSettingsRemoteButton(true);
    } else if (upNow && timerEditMode) {
      registerActivity("UP");
      changeTimerSeconds(1);
      updateDisplay(true);
    } else if (upNow && inputAllowed("UP")) {
      registerActivity("UP");
      cancelTimer("UP manuel");
      goToPosition(upPosition, "UP manuel");
    }
  }

  if (downNow != lastDown) {
    lastDown = downNow;
    logLine(String(F("DOWN -> ")) + (downNow ? F("actif") : F("inactif")));

    if (downNow && displayPage != DISPLAY_PAGE_MAIN && !timerEditMode) {
      handleSettingsRemoteButton(false);
    } else if (downNow && timerEditMode) {
      registerActivity("DOWN");
      changeTimerSeconds(-1);
      updateDisplay(true);
    } else if (downNow && inputAllowed("DOWN")) {
      registerActivity("DOWN");
      cancelTimer("DOWN manuel");
      goToPosition(downPosition, "DOWN manuel");
    }
  }

  if (sleepNow && !lastSleep) {
    centerPressedAt = millis();
    centerLongHandled = false;
    logLine(F("MINUTERIE -> actif"));
  }

  if (sleepNow && !centerLongHandled && centerPressedAt != 0 &&
      (millis() - centerPressedAt) >= centerSettingsHoldMs) {
    centerLongHandled = true;
    cycleSettingsPageFromRemote();
  }

  if (!sleepNow && lastSleep) {
    logLine(F("MINUTERIE -> inactif"));

    if (!centerLongHandled) {
      if (displayPage != DISPLAY_PAGE_MAIN) {
        handleSettingsCenterPress();
      } else {
        registerActivity("MINUTERIE");
        toggleTimerEditMode();
        updateDisplay(true);
      }
    }

    centerPressedAt = 0;
  }

  lastSleep = sleepNow;

  updateTimerHold(upNow, downNow);
  updateTimedDown();
  updateDisplay(false);
  updateDisplaySleep();

  static unsigned long lastTimerNotifyAt = 0;
  if (timedDownActive && millis() - lastTimerNotifyAt >= 1000) {
    lastTimerNotifyAt = millis();
    notifyTimer();
  }
}
