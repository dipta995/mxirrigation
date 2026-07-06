/*
  Valve Latch Admin by MxSolutions.it
  www.mxsolutions.it

  Board: T7 V1.3 MINI 32 ESP32
  Firmware: 1011

  Centralina per comando elettrovalvole latch tramite ponti ad H 

  arduino IDE settings:
  lolin s2 mini
  cdc on boot enabled
  firmare msc disabled
  default 4mb with spiffs
  
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPping.h>
#include "time.h"
#include "esp_sntp.h"
#include <stdarg.h>

const char* FW_VERSION = "1011";

// =========================
// AUTH CONFIG
// =========================
#define AUTH_ENABLED 0
const char* URL_PASSWORD = "change-me";

// =========================
// SAFETY / RECOVERY CONFIG
// =========================
#define ENABLE_AUTO_RESTART_ON_LOW_HEAP 1
const uint32_t LOW_HEAP_THRESHOLD = 12000;
const uint32_t LOW_HEAP_GRACE_MS = 60000;

// =========================
// TIMING CONFIG
// =========================
#define VALVE_PULSE_MS 400
const unsigned long WIFI_RETRY_MS = 10000;
const unsigned long PING_INTERVAL_MS = 30000;
const unsigned long TIME_SYNC_RETRY_MS = 30000;

// =========================
// DEBUG CONFIG
// =========================
#define SERIAL_VALVE_DEBUG 1

// Wi-Fi
const char* ssid = "WMPHOUSE";
const char* password = "motocross";

// Network
IPAddress staticIP(192, 168, 5, 44);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2(1, 1, 1, 1);

// Time / NTP
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";

// Remote host to ping for link health
const char* monitorHost = "192.168.5.40";

const int COMMAND_LOG_SIZE = 4;
const int VALVE_COUNT = 8;

// Definitive GPIOs
const int valveA[VALVE_COUNT] = {22, 17, 27, 32, 0, 19, 5, 26};
const int valveB[VALVE_COUNT] = {21, 16, 25, 4, 2, 23, 33, 18};

// GPIO vers. 1 ( crono ) 
//const int valveA[VALVE_COUNT] = {0, 18, 21, 23, 26, 16, 22, 32};
//const int valveB[VALVE_COUNT] = {2, 19, 22, 25, 27, 17, 33, 4};

bool valveStatus[VALVE_COUNT] = {false, false, false, false, false, false, false, false};

struct ValvePulse {
  bool active;
  int idx;
  bool targetOn;
  unsigned long startMs;
};

struct CommandLogEntry {
  bool valid;
  char timestamp[20];
  char action[4];
  uint8_t valveNumber;
};

ValvePulse currentPulse = {false, -1, false, 0};
CommandLogEntry commandLog[COMMAND_LOG_SIZE];

WebServer server(80);

bool online = false;
bool timeSynced = false;
bool bootTimeCaptured = false;

time_t bootEpoch = 0;

unsigned long lastPingMs = 0;
unsigned long lastWifiRetryMs = 0;
unsigned long lastTimeSyncRetryMs = 0;
unsigned long lowHeapSinceMs = 0;

bool getNowTm(struct tm* timeinfo, uint32_t timeoutMs = 100) {
  return getLocalTime(timeinfo, timeoutMs);
}

bool isTimeValid() {
  time_t now;
  time(&now);
  return now > 1700000000;
}

void formatDateTimeNow(char* out, size_t outSize) {
  struct tm timeinfo;
  if (!getNowTm(&timeinfo, 100)) {
    snprintf(out, outSize, "Time not available");
    return;
  }
  strftime(out, outSize, "%d/%m/%Y %H:%M:%S", &timeinfo);
}

void formatDateNow(char* out, size_t outSize) {
  struct tm timeinfo;
  if (!getNowTm(&timeinfo, 100)) {
    snprintf(out, outSize, "--/--/----");
    return;
  }
  strftime(out, outSize, "%d/%m/%Y", &timeinfo);
}

void formatTimeNow(char* out, size_t outSize) {
  struct tm timeinfo;
  if (!getNowTm(&timeinfo, 100)) {
    snprintf(out, outSize, "--:--:--");
    return;
  }
  strftime(out, outSize, "%H:%M:%S", &timeinfo);
}

void formatEpochDateTime(time_t epoch, char* out, size_t outSize) {
  if (epoch <= 0) {
    snprintf(out, outSize, "Not available");
    return;
  }

  struct tm timeinfo;
  localtime_r(&epoch, &timeinfo);
  strftime(out, outSize, "%d/%m/%Y %H:%M:%S", &timeinfo);
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getNowTm(&timeinfo, 1000)) {
    Serial.println("No time available");
    return;
  }
  Serial.println(&timeinfo, "%A, %d %B %Y %H:%M:%S");
}

void captureBootEpochIfNeeded() {
  if (!bootTimeCaptured && isTimeValid()) {
    time(&bootEpoch);
    bootTimeCaptured = true;
    char buf[32];
    formatEpochDateTime(bootEpoch, buf, sizeof(buf));
    Serial.print("Boot time captured: ");
    Serial.println(buf);
  }
}

void timeavailable(struct timeval* t) {
  Serial.println("Got time adjustment from NTP");
  timeSynced = isTimeValid();
  printLocalTime();
  captureBootEpochIfNeeded();
}

void initTime() {
  sntp_set_time_sync_notification_cb(timeavailable);
  configTzTime(time_zone, ntpServer1, ntpServer2);
}

void addCommandLog(uint8_t valveNumber, const char* action) {
  for (int i = COMMAND_LOG_SIZE - 1; i > 0; i--) {
    commandLog[i] = commandLog[i - 1];
  }

  commandLog[0].valid = true;
  formatDateTimeNow(commandLog[0].timestamp, sizeof(commandLog[0].timestamp));
  snprintf(commandLog[0].action, sizeof(commandLog[0].action), "%s", action);
  commandLog[0].valveNumber = valveNumber;
}

String getAuthQuery() {
#if AUTH_ENABLED
  return String("?pass=") + URL_PASSWORD;
#else
  return "";
#endif
}

bool isAuthorized() {
#if AUTH_ENABLED
  if (!server.hasArg("pass")) return false;
  return server.arg("pass") == URL_PASSWORD;
#else
  return true;
#endif
}

bool ensureAuthorized() {
  if (isAuthorized()) return true;
  server.send(401, "text/plain", "Unauthorized. Use ?pass=YOUR_PASSWORD in URL");
  return false;
}

void sendChunk(const char* s) {
  server.sendContent(s);
}

void sendFmt(const char* fmt, ...) {
  char buf[384];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  server.sendContent(buf);
}

#if SERIAL_VALVE_DEBUG
void logValvePinsState(const char* phase, int idx) {
  if (idx < 0 || idx >= VALVE_COUNT) return;

  Serial.printf(
    "[VALVE %d] %s | A pin=%d state=%s | B pin=%d state=%s\n",
    idx + 1,
    phase,
    valveA[idx], digitalRead(valveA[idx]) ? "HIGH" : "LOW",
    valveB[idx], digitalRead(valveB[idx]) ? "HIGH" : "LOW"
  );
}
#endif

void startHtml(const char* title) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  sendChunk("<!DOCTYPE html><html><head>");
  sendChunk("<meta charset='utf-8'>");
  sendChunk("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  sendChunk("<meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate'>");
  sendChunk("<meta http-equiv='Pragma' content='no-cache'>");
  sendChunk("<meta http-equiv='Expires' content='0'>");
  sendChunk("<title>");
  sendChunk(title);
  sendChunk("</title>");
  sendChunk("</head><body style='font-family:Arial,sans-serif;font-size:14px;'>");
}

void endHtml() {
  sendChunk("</body></html>");
}

void sendNavCompact() {
  String authQuery = getAuthQuery();
  sendFmt("<p><a href='/%s'>Home</a> | <a href='/info%s'>Info</a> | <a href='/logs%s'>Logs</a> | <a href='/status%s'>JSON</a></p>",
          authQuery.c_str(),
          authQuery.c_str(),
          authQuery.c_str(),
          authQuery.c_str());
}

void stopPulse(int idx) {
  if (idx < 0 || idx >= VALVE_COUNT) return;
  digitalWrite(valveA[idx], LOW);
  digitalWrite(valveB[idx], LOW);
}

void beginValvePulse(int idx, bool turnOn) {
  if (idx < 0 || idx >= VALVE_COUNT) return;
  if (currentPulse.active) return;

#if SERIAL_VALVE_DEBUG
  Serial.printf("[VALVE %d] COMMAND %s\n", idx + 1, turnOn ? "ON" : "OFF");
  logValvePinsState("BEFORE", idx);
#endif

  if (turnOn) {
    digitalWrite(valveA[idx], LOW);
    digitalWrite(valveB[idx], HIGH);
  } else {
    digitalWrite(valveA[idx], HIGH);
    digitalWrite(valveB[idx], LOW);
  }

#if SERIAL_VALVE_DEBUG
  logValvePinsState("ACTIVE", idx);
#endif

  currentPulse.active = true;
  currentPulse.idx = idx;
  currentPulse.targetOn = turnOn;
  currentPulse.startMs = millis();

  valveStatus[idx] = turnOn;
  addCommandLog(idx + 1, turnOn ? "ON" : "OFF");

  Serial.printf("Valve %d pulse started -> %s\n", idx + 1, turnOn ? "ON" : "OFF");
}

void updateValvePulse() {
  if (!currentPulse.active) return;

  if (millis() - currentPulse.startMs >= (unsigned long)VALVE_PULSE_MS) {
    stopPulse(currentPulse.idx);

#if SERIAL_VALVE_DEBUG
    logValvePinsState("PULSE END", currentPulse.idx);
#endif

    Serial.printf("Valve %d pulse completed\n", currentPulse.idx + 1);
    currentPulse.active = false;
    currentPulse.idx = -1;
    currentPulse.targetOn = false;
    currentPulse.startMs = 0;
  }
}

void primeValvesOff() {
  for (int i = 0; i < VALVE_COUNT; i++) {
    pinMode(valveA[i], OUTPUT);
    pinMode(valveB[i], OUTPUT);

#if SERIAL_VALVE_DEBUG
    Serial.printf("[VALVE %d] STARTUP RESET\n", i + 1);
    logValvePinsState("INIT BEFORE", i);
#endif

    digitalWrite(valveA[i], HIGH);
    digitalWrite(valveB[i], LOW);

#if SERIAL_VALVE_DEBUG
    logValvePinsState("INIT ACTIVE", i);
#endif

    delay(VALVE_PULSE_MS);
    stopPulse(i);

#if SERIAL_VALVE_DEBUG
    logValvePinsState("INIT END", i);
#endif

    valveStatus[i] = false;
  }
}

void connectWifi(bool verbose = true) {
  if (verbose) {
    Serial.printf("Connecting to WiFi SSID: %s\n", ssid);
  }

  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, gateway, subnet, dns1, dns2);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    if (verbose) Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (verbose) {
      Serial.println();
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
  } else {
    if (verbose) {
      Serial.println();
      Serial.println("WiFi connect timeout");
    }
  }
}

void getWiFiSignalText(char* out, size_t outSize) {
  if (WiFi.status() != WL_CONNECTED) {
    snprintf(out, outSize, "Disconnected");
    return;
  }

  long rssi = WiFi.RSSI();
  if (rssi >= -50) snprintf(out, outSize, "%ld dBm (Excellent)", rssi);
  else if (rssi >= -60) snprintf(out, outSize, "%ld dBm (Good)", rssi);
  else if (rssi >= -70) snprintf(out, outSize, "%ld dBm (Fair)", rssi);
  else snprintf(out, outSize, "%ld dBm (Weak)", rssi);
}

void sendHomePage() {
  char dateBuf[16];
  char timeBuf[16];
  formatDateNow(dateBuf, sizeof(dateBuf));
  formatTimeNow(timeBuf, sizeof(timeBuf));

  startHtml("MxIrrigation");
  sendFmt("<h3>MxIrrigation %s</h3>", FW_VERSION);
  sendNavCompact();
  sendFmt("<p>%s %s</p>", dateBuf, timeBuf);
  sendChunk("<ul>");

  String authQuery = getAuthQuery();
  for (int i = 0; i < VALVE_COUNT; i++) {
    sendFmt("<li>V%d [%s] <a href='/%d/on%s'>ON</a> <a href='/%d/off%s'>OFF</a></li>",
            i + 1,
            valveStatus[i] ? "ON" : "OFF",
            i + 1, authQuery.c_str(),
            i + 1, authQuery.c_str());
  }

  sendChunk("</ul>");
  endHtml();
}

void sendInfoPage() {
  char dateBuf[16];
  char timeBuf[16];
  char bootBuf[32];
  char wifiBuf[40];

  formatDateNow(dateBuf, sizeof(dateBuf));
  formatTimeNow(timeBuf, sizeof(timeBuf));
  formatEpochDateTime(bootEpoch, bootBuf, sizeof(bootBuf));
  getWiFiSignalText(wifiBuf, sizeof(wifiBuf));

  unsigned long uptimeSeconds = millis() / 1000UL;
  unsigned long months  = uptimeSeconds / (30UL * 24UL * 3600UL);
  uptimeSeconds        %= (30UL * 24UL * 3600UL);
  unsigned long days    = uptimeSeconds / (24UL * 3600UL);
  uptimeSeconds        %= (24UL * 3600UL);
  unsigned long hours   = uptimeSeconds / 3600UL;
  uptimeSeconds        %= 3600UL;
  unsigned long minutes = uptimeSeconds / 60UL;
  uptimeSeconds        %= 60UL;

  startHtml("MxIrrigation - Info");
  sendChunk("<h3>System Info</h3>");
  sendNavCompact();
  sendChunk("<pre>");
  sendFmt("Firmware        : %s\n", FW_VERSION);
  sendFmt("IP              : %s\n", WiFi.localIP().toString().c_str());
  sendFmt("WiFi SSID       : %s\n", ssid);
  sendFmt("WiFi connected  : %s\n", WiFi.status() == WL_CONNECTED ? "YES" : "NO");
  sendFmt("Signal          : %s\n", wifiBuf);
  sendFmt("Monitor host    : %s (%s)\n", monitorHost, online ? "ONLINE" : "OFFLINE");
  sendFmt("Date            : %s\n", dateBuf);
  sendFmt("Time            : %s\n", timeBuf);
  sendFmt("NTP sync        : %s\n", timeSynced ? "YES" : "NO");
  sendFmt("Boot time       : %s\n", bootBuf);
  sendFmt("Timezone        : %s\n", time_zone);
  sendFmt("Valve pulse     : %d ms\n", VALVE_PULSE_MS);
  sendFmt("Free heap       : %u\n", ESP.getFreeHeap());
  sendFmt("Min free heap   : %u\n", ESP.getMinFreeHeap());
  sendFmt("Uptime          : %lu months, %lu days, %lu hours, %lu minutes, %lu seconds\n",
          months, days, hours, minutes, uptimeSeconds);
  sendChunk("</pre>");

  String authQuery = getAuthQuery();
  sendFmt("<p><a href='/reboot%s' onclick=\"return confirm('Are you sure you want to reboot the controller?');\">REBOOT DEVICE</a></p>",
          authQuery.c_str());

  endHtml();
}

void sendLogsPage() {
  startHtml("MxIrrigation - Logs");
  sendChunk("<h3>Last 4 Commands</h3>");
  sendNavCompact();
  sendChunk("<ul>");

  bool hasLogs = false;
  for (int i = 0; i < COMMAND_LOG_SIZE; i++) {
    if (commandLog[i].valid) {
      hasLogs = true;
      sendFmt("<li>%s - Valve %u -> %s</li>",
              commandLog[i].timestamp,
              commandLog[i].valveNumber,
              commandLog[i].action);
    }
  }

  if (!hasLogs) {
    sendChunk("<li>No commands yet</li>");
  }

  sendChunk("</ul>");
  endHtml();
}

void sendStatusJson() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  char nowBuf[32];
  char bootBuf[32];
  formatDateTimeNow(nowBuf, sizeof(nowBuf));
  formatEpochDateTime(bootEpoch, bootBuf, sizeof(bootBuf));

  sendChunk("{");
  sendFmt("\"firmware\":\"%s\",", FW_VERSION);
  sendFmt("\"ip\":\"%s\",", WiFi.localIP().toString().c_str());
  sendFmt("\"wifi_ssid\":\"%s\",", ssid);
  sendFmt("\"wifi_connected\":%s,", WiFi.status() == WL_CONNECTED ? "true" : "false");
  sendFmt("\"wifi_rssi\":%d,", WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  sendFmt("\"monitor_online\":%s,", online ? "true" : "false");
  sendFmt("\"time_synced\":%s,", timeSynced ? "true" : "false");
  sendFmt("\"datetime\":\"%s\",", nowBuf);
  sendFmt("\"boot_datetime\":\"%s\",", bootBuf);
  sendFmt("\"timezone\":\"%s\",", time_zone);
  sendFmt("\"valve_pulse_ms\":%d,", VALVE_PULSE_MS);
  sendChunk("\"valves\":[");
  for (int i = 0; i < VALVE_COUNT; i++) {
    sendFmt("{\"id\":%d,\"on\":%s}%s",
            i + 1,
            valveStatus[i] ? "true" : "false",
            (i < VALVE_COUNT - 1) ? "," : "");
  }
  sendChunk("],");
  sendChunk("\"last_commands\":[");

  bool first = true;
  for (int i = 0; i < COMMAND_LOG_SIZE; i++) {
    if (!commandLog[i].valid) continue;
    if (!first) sendChunk(",");
    first = false;
    sendFmt("{\"timestamp\":\"%s\",\"valve\":%u,\"action\":\"%s\"}",
            commandLog[i].timestamp,
            commandLog[i].valveNumber,
            commandLog[i].action);
  }

  sendChunk("]}");
}

void handleRoot() {
  if (!ensureAuthorized()) return;
  sendHomePage();
}

void handleInfo() {
  if (!ensureAuthorized()) return;
  sendInfoPage();
}

void handleLogs() {
  if (!ensureAuthorized()) return;
  sendLogsPage();
}

void handleStatus() {
  if (!ensureAuthorized()) return;
  sendStatusJson();
}

void handleReboot() {
  if (!ensureAuthorized()) return;

  startHtml("MxIrrigation - Reboot");
  sendChunk("<h3>Rebooting...</h3>");
  sendChunk("<p>The device is restarting now.</p>");
  endHtml();

  delay(500);
  ESP.restart();
}

void handleValveOn(int idx) {
  if (!ensureAuthorized()) return;

  if (idx < 0 || idx >= VALVE_COUNT) {
    server.send(404, "text/plain", "Invalid valve");
    return;
  }

  beginValvePulse(idx, true);
  sendHomePage();
}

void handleValveOff(int idx) {
  if (!ensureAuthorized()) return;

  if (idx < 0 || idx >= VALVE_COUNT) {
    server.send(404, "text/plain", "Invalid valve");
    return;
  }

  beginValvePulse(idx, false);
  sendHomePage();
}

void registerRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/info", HTTP_GET, handleInfo);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/reboot", HTTP_GET, handleReboot);

  for (int i = 0; i < VALVE_COUNT; i++) {
    const int idx = i;
    String onPath = "/" + String(i + 1) + "/on";
    String offPath = "/" + String(i + 1) + "/off";

    server.on(onPath.c_str(), HTTP_GET, [idx]() { handleValveOn(idx); });
    server.on(offPath.c_str(), HTTP_GET, [idx]() { handleValveOff(idx); });
  }

  server.onNotFound([]() {
    if (!ensureAuthorized()) return;
    server.send(404, "text/plain", "Not found");
  });
}

void checkLowHeapRecovery() {
#if ENABLE_AUTO_RESTART_ON_LOW_HEAP
  uint32_t heap = ESP.getFreeHeap();

  if (heap < LOW_HEAP_THRESHOLD) {
    if (lowHeapSinceMs == 0) {
      lowHeapSinceMs = millis();
      Serial.printf("Low heap detected: %u bytes\n", heap);
    } else if (millis() - lowHeapSinceMs >= LOW_HEAP_GRACE_MS) {
      Serial.printf("Heap stayed below %u bytes for %u ms. Restarting...\n",
                    LOW_HEAP_THRESHOLD, LOW_HEAP_GRACE_MS);
      delay(100);
      ESP.restart();
    }
  } else {
    lowHeapSinceMs = 0;
  }
#endif
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Valve Latch Admin by MxSolutions.it");
  Serial.println("www.mxsolutions.it");
  Serial.print("Firmware: ");
  Serial.println(FW_VERSION);
  Serial.println("----------------------------------------");

#if AUTH_ENABLED
  Serial.println("URL auth enabled");
#else
  Serial.println("URL auth disabled");
#endif

#if ENABLE_AUTO_RESTART_ON_LOW_HEAP
  Serial.println("Low-heap auto-restart enabled");
#else
  Serial.println("Low-heap auto-restart disabled");
#endif

#if SERIAL_VALVE_DEBUG
  Serial.println("Serial valve debug enabled");
#else
  Serial.println("Serial valve debug disabled");
#endif

  primeValvesOff();
  connectWifi(true);

  if (WiFi.status() == WL_CONNECTED) {
    initTime();
  }

  registerRoutes();
  server.begin();

  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  updateValvePulse();
  checkLowHeapRecovery();

  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    online = false;
    timeSynced = false;

    if (now - lastWifiRetryMs >= WIFI_RETRY_MS) {
      lastWifiRetryMs = now;
      Serial.println("WiFi disconnected, attempting reconnect...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
  } else {
    timeSynced = isTimeValid();

    if (!timeSynced && now - lastTimeSyncRetryMs >= TIME_SYNC_RETRY_MS) {
      lastTimeSyncRetryMs = now;
      Serial.println("Retrying NTP sync...");
      initTime();
    }

    captureBootEpochIfNeeded();

    if (now - lastPingMs >= PING_INTERVAL_MS) {
      lastPingMs = now;
      online = Ping.ping(monitorHost);
      Serial.printf("Ping %s => %s\n", monitorHost, online ? "OK" : "FAIL");
    }
  }
}
