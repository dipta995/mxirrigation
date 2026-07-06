

/*
  MxIrrigation Status Display
  Target: LILYGO TTGO T4 V1.3 / ESP32 / ILI9341

  The display connects to the irrigation controller JSON endpoint:
  http://192.168.5.44/status

  Screens, changed with the on-board GPIO38 button:
    1. Overview
    2. Valves 1-4
    3. Valves 5-8
    4. Controller details

  Required Arduino IDE libraries:
    - GFX Library for Arduino (Moon On Our Nation)
    - ArduinoJson (Benoit Blanchon)

  Board in Arduino IDE:
    ESP32 Dev Module

  IMPORTANT:
  GPIO 37, 38 and 39 are input-only pins on ESP32 and do not provide
  internal pull-up resistors. The TTGO T4 V1.3 hardware buttons already
  provide the required external circuitry.

  If the page does not change when pressing the selected button, first try:
    1. changing BUTTON_NEXT_PIN to 37 or 39;
    2. changing BUTTON_ACTIVE_LEVEL from LOW to HIGH.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <time.h>
#include <stdlib.h>

// ============================================================================
// NETWORK AND CONTROLLER CONFIGURATION
// ============================================================================

const char *WIFI_SSID     = "WMPHOUSE";
const char *WIFI_PASSWORD = "motocross";

// If AUTH_ENABLED is enabled on the controller, add the password query here.
// Example: "http://192.168.5.44/status?pass=YOUR_PASSWORD"
const char *CONTROLLER_STATUS_URL = "http://192.168.5.44/status";

const uint32_t STATUS_REFRESH_MS     = 5000;
const uint32_t CONTROLLER_STALE_MS   = 20000;
const uint32_t WIFI_RETRY_MS         = 10000;
const uint16_t HTTP_CONNECT_TIMEOUT  = 1200;
const uint16_t HTTP_RESPONSE_TIMEOUT = 1500;

// Same timezone used by the controller firmware. It is only used to calculate
// controller uptime from the "datetime" and "boot_datetime" JSON fields.
const char *TIME_ZONE = "CET-1CEST,M3.5.0,M10.5.0/3";

// ============================================================================
// TTGO T4 V1.3 DISPLAY AND BUTTON PINS
// ============================================================================

// ILI9341 integrated display wiring for TTGO T4 V1.3.
constexpr int TFT_DC   = 32;
constexpr int TFT_CS   = 27;
constexpr int TFT_SCLK = 18;
constexpr int TFT_MOSI = 23;
constexpr int TFT_MISO = 12;
constexpr int TFT_RST  = 5;
constexpr int TFT_BL   = 4;

// GPIO38 is one of the three user buttons on the TTGO T4 V1.3.
// Alternatives: GPIO37 or GPIO39.
constexpr int BUTTON_NEXT_PIN = 38;
constexpr int BUTTON_ACTIVE_LEVEL = LOW;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;

Arduino_DataBus *displayBus = new Arduino_ESP32SPI(
  TFT_DC,
  TFT_CS,
  TFT_SCLK,
  TFT_MOSI,
  TFT_MISO,
  HSPI
);

// Rotation 1 = landscape, 320 x 240.
Arduino_GFX *gfx = new Arduino_ILI9341(displayBus, TFT_RST, 1);

// ============================================================================
// UI COLORS
// ============================================================================

constexpr uint16_t COLOR_BACKGROUND = 0x0000;
constexpr uint16_t COLOR_TEXT       = 0xFFFF;
constexpr uint16_t COLOR_MUTED      = 0x8410;
constexpr uint16_t COLOR_ACCENT     = 0xFD20; // orange
constexpr uint16_t COLOR_OK         = 0x07E0; // green
constexpr uint16_t COLOR_ALERT      = 0xF800; // red
constexpr uint16_t COLOR_WARNING    = 0xFFE0; // yellow
constexpr uint16_t COLOR_CARD       = 0x2104;
constexpr uint16_t COLOR_BORDER     = 0x52AA;

// ============================================================================
// STATUS DATA
// ============================================================================

constexpr int VALVE_COUNT = 8;

struct ControllerStatus {
  bool valid = false;
  bool wifiConnected = false;
  bool monitorOnline = false;
  bool timeSynced = false;
  bool valves[VALVE_COUNT] = {false};

  int wifiRssi = 0;
  int httpCode = 0;

  bool uptimeValid = false;
  uint32_t uptimeSecondsAtFetch = 0;

  uint32_t lastSuccessMs = 0;
  uint32_t lastAttemptMs = 0;

  char firmware[20] = "--";
  char ip[20] = "--";
  char controllerSsid[40] = "--";
  char dateTime[32] = "--";
  char bootDateTime[32] = "--";
  char error[64] = "--";
};

ControllerStatus controller;

// ============================================================================
// APPLICATION STATE
// ============================================================================

enum Screen : uint8_t {
  SCREEN_OVERVIEW = 0,
  SCREEN_VALVES_1_4,
  SCREEN_VALVES_5_8,
  SCREEN_SYSTEM,
  SCREEN_COUNT
};

Screen currentScreen = SCREEN_OVERVIEW;

bool buttonRawPressed = false;
bool buttonStablePressed = false;
uint32_t buttonLastRawChangeMs = 0;

uint32_t lastStatusRequestMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t lastUiSecondMs = 0;

bool screenDirty = true;

// ============================================================================
// GENERIC HELPERS
// ============================================================================

void copyText(char *destination, size_t destinationSize, const char *source) {
  if (source == nullptr) source = "--";
  snprintf(destination, destinationSize, "%s", source);
}

bool isControllerFresh() {
  if (!controller.valid) return false;
  return (millis() - controller.lastSuccessMs) < CONTROLLER_STALE_MS;
}

bool isButtonPressed() {
  return digitalRead(BUTTON_NEXT_PIN) == BUTTON_ACTIVE_LEVEL;
}

uint8_t openValveCount() {
  uint8_t count = 0;
  for (int i = 0; i < VALVE_COUNT; i++) {
    if (controller.valves[i]) count++;
  }
  return count;
}

void formatElapsed(uint32_t seconds, char *out, size_t outSize) {
  const uint32_t days = seconds / 86400UL;
  seconds %= 86400UL;
  const uint32_t hours = seconds / 3600UL;
  seconds %= 3600UL;
  const uint32_t minutes = seconds / 60UL;
  seconds %= 60UL;

  if (days > 0) {
    snprintf(out, outSize, "%lud %02luh %02lum",
             (unsigned long)days,
             (unsigned long)hours,
             (unsigned long)minutes);
  } else if (hours > 0) {
    snprintf(out, outSize, "%02luh %02lum %02lus",
             (unsigned long)hours,
             (unsigned long)minutes,
             (unsigned long)seconds);
  } else {
    snprintf(out, outSize, "%02lum %02lus",
             (unsigned long)minutes,
             (unsigned long)seconds);
  }
}

void formatAge(uint32_t eventMs, char *out, size_t outSize) {
  if (eventMs == 0) {
    snprintf(out, outSize, "--");
    return;
  }

  const uint32_t elapsedSeconds = (millis() - eventMs) / 1000UL;

  if (elapsedSeconds < 60) {
    snprintf(out, outSize, "%lus fa", (unsigned long)elapsedSeconds);
  } else if (elapsedSeconds < 3600) {
    snprintf(out, outSize, "%lum fa", (unsigned long)(elapsedSeconds / 60UL));
  } else {
    snprintf(out, outSize, "%luh fa", (unsigned long)(elapsedSeconds / 3600UL));
  }
}

bool parseControllerDateTime(const char *source, time_t &result) {
  if (source == nullptr) return false;

  int day, month, year, hour, minute, second;
  const int parsed = sscanf(
    source,
    "%d/%d/%d %d:%d:%d",
    &day, &month, &year, &hour, &minute, &second
  );

  if (parsed != 6) return false;
  if (year < 2024 || month < 1 || month > 12 || day < 1 || day > 31) return false;

  struct tm value = {};
  value.tm_mday = day;
  value.tm_mon = month - 1;
  value.tm_year = year - 1900;
  value.tm_hour = hour;
  value.tm_min = minute;
  value.tm_sec = second;
  value.tm_isdst = -1;

  result = mktime(&value);
  return result > 0;
}

void calculateControllerUptime() {
  controller.uptimeValid = false;

  time_t remoteNow;
  time_t remoteBoot;

  if (!parseControllerDateTime(controller.dateTime, remoteNow)) return;
  if (!parseControllerDateTime(controller.bootDateTime, remoteBoot)) return;
  if (remoteNow < remoteBoot) return;

  controller.uptimeSecondsAtFetch = (uint32_t)difftime(remoteNow, remoteBoot);
  controller.uptimeValid = true;
}

uint32_t currentControllerUptimeSeconds() {
  if (!controller.uptimeValid || controller.lastSuccessMs == 0) return 0;

  const uint32_t secondsSinceFetch =
    (millis() - controller.lastSuccessMs) / 1000UL;

  return controller.uptimeSecondsAtFetch + secondsSinceFetch;
}

// ============================================================================
// WIFI AND HTTP
// ============================================================================

void beginWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttemptMs = millis();
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  if (millis() - lastWifiAttemptMs >= WIFI_RETRY_MS) {
    Serial.println("WiFi not connected. Retrying...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttemptMs = millis();
    screenDirty = true;
  }
}

void fetchControllerStatus() {
  controller.lastAttemptMs = millis();

  if (WiFi.status() != WL_CONNECTED) {
    copyText(controller.error, sizeof(controller.error), "TTGO WiFi non connesso");
    screenDirty = true;
    return;
  }

  WiFiClient client;
  HTTPClient http;

  http.setConnectTimeout(HTTP_CONNECT_TIMEOUT);
  http.setTimeout(HTTP_RESPONSE_TIMEOUT);

  if (!http.begin(client, CONTROLLER_STATUS_URL)) {
    copyText(controller.error, sizeof(controller.error), "HTTP begin failed");
    screenDirty = true;
    return;
  }

  const int httpCode = http.GET();
  controller.httpCode = httpCode;

  if (httpCode != HTTP_CODE_OK) {
    char errorText[64];
    snprintf(errorText, sizeof(errorText), "HTTP error %d", httpCode);
    copyText(controller.error, sizeof(controller.error), errorText);
    http.end();
    screenDirty = true;
    return;
  }

  const String payload = http.getString();
  http.end();

  DynamicJsonDocument json(3072);
  const DeserializationError jsonError = deserializeJson(json, payload);

  if (jsonError) {
    copyText(controller.error, sizeof(controller.error), "JSON non valido");
    screenDirty = true;
    return;
  }

  copyText(controller.firmware, sizeof(controller.firmware), json["firmware"] | "--");
  copyText(controller.ip, sizeof(controller.ip), json["ip"] | "--");
  copyText(controller.controllerSsid, sizeof(controller.controllerSsid), json["wifi_ssid"] | "--");
  copyText(controller.dateTime, sizeof(controller.dateTime), json["datetime"] | "--");
  copyText(controller.bootDateTime, sizeof(controller.bootDateTime), json["boot_datetime"] | "--");

  controller.wifiConnected = json["wifi_connected"] | false;
  controller.wifiRssi = json["wifi_rssi"] | 0;
  controller.monitorOnline = json["monitor_online"] | false;
  controller.timeSynced = json["time_synced"] | false;

  for (int i = 0; i < VALVE_COUNT; i++) {
    controller.valves[i] = false;
  }

  JsonArray valves = json["valves"].as<JsonArray>();
  for (JsonObject valve : valves) {
    const int id = valve["id"] | 0;
    if (id >= 1 && id <= VALVE_COUNT) {
      controller.valves[id - 1] = valve["on"] | false;
    }
  }

  controller.valid = true;
  controller.lastSuccessMs = millis();
  copyText(controller.error, sizeof(controller.error), "");
  calculateControllerUptime();

  Serial.printf(
    "Controller status updated. Open valves: %u\n",
    openValveCount()
  );

  screenDirty = true;
}

// ============================================================================
// DISPLAY HELPERS
// ============================================================================

void drawText(int16_t x, int16_t y, const char *text, uint16_t color, uint8_t size = 1) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(x, y);
  gfx->print(text);
}

void drawHeader(const char *title, uint8_t pageNumber) {
  gfx->fillScreen(COLOR_BACKGROUND);
  gfx->fillRect(0, 0, 320, 30, COLOR_ACCENT);

  drawText(10, 7, title, COLOR_BACKGROUND, 2);

  char pageText[12];
  snprintf(pageText, sizeof(pageText), "%u/%u", pageNumber, SCREEN_COUNT);
  drawText(280, 10, pageText, COLOR_BACKGROUND, 1);
}

void drawFooter() {
  gfx->fillRect(0, 220, 320, 20, COLOR_CARD);
  drawText(9, 226, "GPIO38: pagina successiva", COLOR_TEXT, 1);
}

void drawConnectionPill(int16_t x, int16_t y, bool online, const char *label) {
  const uint16_t color = online ? COLOR_OK : COLOR_ALERT;
  gfx->fillRoundRect(x, y, 120, 24, 5, color);
  drawText(x + 9, y + 7, label, COLOR_BACKGROUND, 1);
}

void drawLabelValue(int16_t y, const char *label, const char *value, uint16_t valueColor = COLOR_TEXT) {
  drawText(16, y, label, COLOR_MUTED, 1);
  drawText(140, y, value, valueColor, 1);
}

void drawValveCard(int16_t x, int16_t y, int valveIndex) {
  const bool isOpen = controller.valid && controller.valves[valveIndex];
  const uint16_t stateColor = isOpen ? COLOR_OK : COLOR_ALERT;

  gfx->fillRoundRect(x, y, 142, 66, 7, COLOR_CARD);
  gfx->drawRoundRect(x, y, 142, 66, 7, COLOR_BORDER);

  char valveText[8];
  snprintf(valveText, sizeof(valveText), "V%d", valveIndex + 1);
  drawText(x + 10, y + 11, valveText, COLOR_TEXT, 2);

  gfx->fillRoundRect(x + 67, y + 11, 61, 18, 4, stateColor);
  drawText(x + 74, y + 16, isOpen ? "APERTA" : "CHIUSA", COLOR_BACKGROUND, 1);

  drawText(
    x + 10,
    y + 42,
    controller.valid ? (isOpen ? "Irrigazione attiva" : "Irrigazione ferma") : "Nessun dato",
    COLOR_MUTED,
    1
  );
}

// ============================================================================
// DISPLAY SCREENS
// ============================================================================

void drawOverviewScreen() {
  drawHeader("MxIrrigation", 1);

  const bool controllerOnline = isControllerFresh();
  const bool ttgoWiFiOnline = WiFi.status() == WL_CONNECTED;

  drawConnectionPill(12, 42, controllerOnline, controllerOnline ? "CENTRALINA ONLINE" : "CENTRALINA OFFLINE");
  drawConnectionPill(188, 42, ttgoWiFiOnline, ttgoWiFiOnline ? "TTGO WIFI OK" : "TTGO WIFI KO");

  char value[64];

  snprintf(value, sizeof(value), "%u / %d", openValveCount(), VALVE_COUNT);
  drawLabelValue(87, "Valvole aperte", value, openValveCount() > 0 ? COLOR_OK : COLOR_TEXT);

  if (controller.valid) {
    snprintf(value, sizeof(value), "%s (%d dBm)",
             controller.wifiConnected ? "COLLEGATO" : "SCOLLEGATO",
             controller.wifiRssi);
    drawLabelValue(111, "WiFi centralina", value,
                   controller.wifiConnected ? COLOR_OK : COLOR_ALERT);

    drawLabelValue(135, "Ping host rete",
                   controller.monitorOnline ? "OK" : "FAIL",
                   controller.monitorOnline ? COLOR_OK : COLOR_ALERT);
  } else {
    drawLabelValue(111, "WiFi centralina", "--", COLOR_MUTED);
    drawLabelValue(135, "Ping host rete", "--", COLOR_MUTED);
  }

  formatAge(controller.lastSuccessMs, value, sizeof(value));
  drawLabelValue(159, "Ultimo aggiornamento", value,
                 controllerOnline ? COLOR_TEXT : COLOR_WARNING);

  if (!controllerOnline) {
    drawText(16, 190, controller.error, COLOR_WARNING, 1);
  } else {
    drawText(16, 190, "Aggiornamento automatico ogni 5 secondi", COLOR_MUTED, 1);
  }

  drawFooter();
}

void drawValvesScreen(int firstValveIndex, uint8_t pageNumber) {
  char title[20];
  snprintf(title, sizeof(title), "Valvole %d-%d",
           firstValveIndex + 1,
           firstValveIndex + 4);

  drawHeader(title, pageNumber);

  drawValveCard(12, 44, firstValveIndex);
  drawValveCard(166, 44, firstValveIndex + 1);
  drawValveCard(12, 123, firstValveIndex + 2);
  drawValveCard(166, 123, firstValveIndex + 3);

  drawFooter();
}

void drawSystemScreen() {
  drawHeader("Dettagli centralina", 4);

  char value[80];

  drawLabelValue(45, "IP centralina", controller.valid ? controller.ip : "--");
  drawLabelValue(67, "Firmware", controller.valid ? controller.firmware : "--");

  if (controller.uptimeValid) {
    formatElapsed(currentControllerUptimeSeconds(), value, sizeof(value));
  } else {
    snprintf(value, sizeof(value), "--");
  }
  drawLabelValue(89, "Uptime centralina", value, controller.uptimeValid ? COLOR_OK : COLOR_WARNING);

  drawLabelValue(111, "Ora centralina",
                 controller.valid ? controller.dateTime : "--");

  drawLabelValue(133, "Avvio centralina",
                 controller.valid ? controller.bootDateTime : "--");

  drawLabelValue(155, "NTP centralina",
                 controller.valid ? (controller.timeSynced ? "SINCRONIZZATO" : "NON DISPONIBILE") : "--",
                 controller.timeSynced ? COLOR_OK : COLOR_WARNING);

  if (WiFi.status() == WL_CONNECTED) {
    snprintf(value, sizeof(value), "%s", WiFi.localIP().toString().c_str());
  } else {
    snprintf(value, sizeof(value), "NON COLLEGATO");
  }
  drawLabelValue(177, "IP display TTGO", value,
                 WiFi.status() == WL_CONNECTED ? COLOR_TEXT : COLOR_ALERT);

  drawFooter();
}

void renderCurrentScreen() {
  switch (currentScreen) {
    case SCREEN_OVERVIEW:
      drawOverviewScreen();
      break;

    case SCREEN_VALVES_1_4:
      drawValvesScreen(0, 2);
      break;

    case SCREEN_VALVES_5_8:
      drawValvesScreen(4, 3);
      break;

    case SCREEN_SYSTEM:
      drawSystemScreen();
      break;

    default:
      currentScreen = SCREEN_OVERVIEW;
      drawOverviewScreen();
      break;
  }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void handleNavigationButton() {
  const bool rawPressed = isButtonPressed();

  if (rawPressed != buttonRawPressed) {
    buttonRawPressed = rawPressed;
    buttonLastRawChangeMs = millis();
  }

  if ((millis() - buttonLastRawChangeMs) < BUTTON_DEBOUNCE_MS) {
    return;
  }

  if (buttonStablePressed == buttonRawPressed) {
    return;
  }

  buttonStablePressed = buttonRawPressed;

  // Only act on the press edge, not when the button is released.
  if (buttonStablePressed) {
    currentScreen = static_cast<Screen>((currentScreen + 1) % SCREEN_COUNT);
    screenDirty = true;
    Serial.printf("Screen changed to %u\n", currentScreen + 1);
  }
}

// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  // Uses the same timezone as the irrigation controller to calculate its uptime.
  setenv("TZ", TIME_ZONE, 1);
  tzset();

  // TTGO T4 V1.3 backlight is active HIGH.
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  gfx->begin();
  gfx->fillScreen(COLOR_BACKGROUND);

  pinMode(BUTTON_NEXT_PIN, INPUT);
  buttonRawPressed = isButtonPressed();
  buttonStablePressed = buttonRawPressed;
  buttonLastRawChangeMs = millis();

  renderCurrentScreen();

  beginWiFi();

  // Fetch controller status as soon as WiFi gets connected.
  lastStatusRequestMs = millis() - STATUS_REFRESH_MS;

  Serial.println("MxIrrigation Status Display started");
}

void loop() {
  handleNavigationButton();
  maintainWiFi();

  const uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED &&
      now - lastStatusRequestMs >= STATUS_REFRESH_MS) {
    lastStatusRequestMs = now;
    fetchControllerStatus();
  }

  // Keep age counters and uptime text current on screens that use them.
  if (now - lastUiSecondMs >= 1000) {
    lastUiSecondMs = now;

    if (currentScreen == SCREEN_OVERVIEW || currentScreen == SCREEN_SYSTEM) {
      screenDirty = true;
    }
  }

  if (screenDirty) {
    renderCurrentScreen();
    screenDirty = false;
  }

  delay(5);
}
