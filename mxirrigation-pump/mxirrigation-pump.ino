/*
  MxIrrigation by MxSolutions.it
  Author: Nicola Deboni, Mx Solutions
  Firmware version: 1.2.1

  Description
  -----------
  ESP32-based irrigation pump controller with:
  - Web UI (status + control links; optional password to show controls)
  - Non-blocking pump start/stop sequencing (timers instead of delays)
  - Pressure monitoring (high-pressure trip and low-pressure trip)
  - Network watchdog via ICMP ping (reboot after consecutive failures when pumps are off)
  - NTP time for timestamped alert logs displayed on the web root page
  - /logs page: shows last 10 events since boot (pump ON/OFF, alerts, restarts, etc.)
  - /pressure page: shows a simple cartesian graph (HTML5 canvas) of last hour pressure,
    sampled once per second (3600 samples). Graph uses converted pressure (bar).

  Notes
  -----
  - Storing 3600 samples uses RAM. This implementation stores uint16_t pressure samples in
    centibar (bar*100) (~7.2 KB). This avoids floats in the history buffer.
  - Uptime month formatting is approximate and uses 30-day months.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPping.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>

const char* FW_VERSION = "1.2.1";

const char *ssid = "WMPSERVICE";
const char *password = "motocross";

IPAddress staticIP(192, 168, 5, 43);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

const int relayPins[] = {21, 19, 18, 5};
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int ledPin = 25;

// pressure sensor
int analogPin = 36;
int raw = 0;
float valore = 0; // current pressure in bar
unsigned long pressureTimer;

// status
bool masterOn = false;

// ---- Web UI password (query string) ----
#define ENABLE_WEB_PASSWORD 1
const char* WEB_PASSWORD = "1234";   // <-- change this
const char* WEB_PW_PARAM = "pw";     // URL: /?pw=1234

// ---- Timezone / NTP ----
const char* TZ_INFO = "CET-1CEST,M3.5.0/2,M10.5.0/3"; // Italy local time with DST
const unsigned long NTP_RETRY_PERIOD_MS = 60000;
unsigned long lastNtpSyncAttemptMs = 0;

// ---- ntfy alerts ----
const char* NTFY_SERVER = "https://ntfy.sh";
const char* NTFY_TOPIC  = "wmp-irrigation";

// ---- High/Low pressure shutdown settings/state ----
const int RAW_LIMIT = 2000;
const unsigned long RAW_OVER_LIMIT_MS = 15000;

const int RAW_MIN_LIMIT = 500;
const unsigned long RAW_UNDER_LIMIT_MS = 60000;

unsigned long rawOverStartMs = 0;
unsigned long rawUnderStartMs = 0;

// Alert logs shown on root page
String pressureTripLog = "";
String lowPressureTripLog = "";

// ---- Last 10 events ring buffer (/logs) ----
static const int EVENT_LOG_CAPACITY = 10;
String eventLogs[EVENT_LOG_CAPACITY];
int eventLogHead = 0;   // next write index
int eventLogCount = 0;  // number of valid entries (<= capacity)

// ---- Pressure history ring buffer: last hour @ 1 Hz (/pressure) ----
// We store pressure in "centibar" (bar * 100) as integer to avoid floats in RAM/history.
static const int PRESSURE_HISTORY_SECONDS = 3600;
uint16_t pressureHistoryCb[PRESSURE_HISTORY_SECONDS];
int pressureHistHead = 0;   // next write index
int pressureHistCount = 0;  // number of valid samples (<= 3600)

// ---- WiFi reconnect monitor ----
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 30000;
const unsigned long WIFI_RECONNECT_PERIOD_MS = 10000;
unsigned long wifiConnectAttemptStartMs = 0;
unsigned long lastWiFiReconnectAttemptMs = 0;
bool wifiConnecting = false;

// ---- Non-blocking ping/reboot monitor ----
const unsigned long PING_START_DELAY_MS = 120000;   // start 120s after boot
const unsigned long PING_PERIOD_MS      = 30000;    // every 30s when pumps off
const int PING_FAIL_REBOOT_COUNT        = 8;
const unsigned long PING_REBOOT_COOLDOWN_MS = 300000; // 5 minutes

unsigned long bootMs = 0;
unsigned long lastPingMs = 0;
int pingFailCount = 0;
unsigned long lastPingTriggeredRestartMs = 0;

// ---- Non-blocking pump sequencer ----
enum PumpSequenceState {
  PSEQ_IDLE = 0,
  PSEQ_RELAY4_WAIT5S,
  PSEQ_MASTER_WAIT2S,
  PSEQ_PUMP1_WAIT4S,
  PSEQ_STAGGERED_PUMP2_WAIT,
  PSEQ_STAGGERED_PUMP2_HOLD
};

enum StartMode {
  START_NONE = 0,
  START_MASTER_BOTH,
  START_MASTER_SOLAR_BOTH,
  START_SINGLE_ONLY,
  START_STAGGERED_AUTO_OFF
};

PumpSequenceState pumpSeqState = PSEQ_IDLE;
unsigned long pumpSeqMs = 0;
StartMode pendingStartMode = START_NONE;
StartMode activeStartMode = START_NONE;
bool pendingStop = false;

// Forward declarations
static int median3(int a, int b, int c);
String nowString();
String formatUptime(unsigned long uptimeMs);
String htmlEscape(const String& s);
String urlEncode(const String& s);
void addEventLog(const String& msg);
void addPressureSampleCentibar(uint16_t centibar);
uint16_t rawToCentibar(int rawValue);
void clearPressureAlertLogs();
bool isWebAuthorized();
bool ensureAuthorized();
void resetSequencer();
void stopAllPumps(const String& reason);
void queueStartMode(StartMode mode, const String& actionLabel);
void setupWiFi();
void ensureWiFiConnected();
void configureLocalTime();
void ensureTimeConfigured();
void sendNtfyNotification(const String& title, const String& message, const String& priority, const String& tags);
void checkHighPressureTrip();
void checkLowPressureTripWhilePumpsOn();
void monitorPingAndRebootWhenPumpsOff();
void runPumpSequencer();
void handleRoot();
void handleLogs();
void handlePressure();
void handleReboot();

static int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b; // median
}

String formatUptime(unsigned long uptimeMs) {
  unsigned long totalSeconds = uptimeMs / 1000UL;
  unsigned long seconds = totalSeconds % 60UL;
  unsigned long totalMinutes = totalSeconds / 60UL;
  unsigned long minutes = totalMinutes % 60UL;
  unsigned long totalHours = totalMinutes / 60UL;
  unsigned long hours = totalHours % 24UL;
  unsigned long totalDays = totalHours / 24UL;
  unsigned long months = totalDays / 30UL; // approximate months
  unsigned long days = totalDays % 30UL;

  String out;
  if (months > 0) out += String(months) + " month" + (months == 1 ? "" : "s") + " ";
  out += String(days) + " day" + (days == 1 ? "" : "s") + " ";
  out += String(hours) + "h ";
  out += String(minutes) + "m ";
  out += String(seconds) + "s";
  return out;
}

String nowString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "NTP not set (uptime " + formatUptime(millis()) + ")";
  }
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}

String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length() + 16);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c; break;
    }
  }
  return out;
}

String urlEncode(const String& s) {
  String out;
  char hex[4];
  out.reserve(s.length() * 3);

  for (size_t i = 0; i < s.length(); i++) {
    unsigned char c = (unsigned char)s[i];
    if ((c >= 'a' && c <= 'z') ||
        (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') ||
        c == '-' || c == '_' || c == '.' || c == '~') {
      out += (char)c;
    } else if (c == ' ') {
      out += "%20";
    } else {
      snprintf(hex, sizeof(hex), "%%%02X", c);
      out += hex;
    }
  }

  return out;
}

void addEventLog(const String& msg) {
  eventLogs[eventLogHead] = nowString() + " - " + msg;
  eventLogHead = (eventLogHead + 1) % EVENT_LOG_CAPACITY;
  if (eventLogCount < EVENT_LOG_CAPACITY) eventLogCount++;
}

void addPressureSampleCentibar(uint16_t centibar) {
  pressureHistoryCb[pressureHistHead] = centibar;
  pressureHistHead = (pressureHistHead + 1) % PRESSURE_HISTORY_SECONDS;
  if (pressureHistCount < PRESSURE_HISTORY_SECONDS) pressureHistCount++;
}

uint16_t rawToCentibar(int rawValue) {
  if (rawValue <= 0) return 0;
  if (rawValue >= 2095) return 1000; // 10.00 bar
  return (uint16_t)((rawValue * 1000L + (2095 / 2)) / 2095);
}

void clearPressureAlertLogs() {
  pressureTripLog = "";
  lowPressureTripLog = "";
  rawOverStartMs = 0;
  rawUnderStartMs = 0;
}

bool isWebAuthorized() {
#if ENABLE_WEB_PASSWORD
  if (!server.hasArg(WEB_PW_PARAM)) return false;
  return server.arg(WEB_PW_PARAM) == WEB_PASSWORD;
#else
  return true;
#endif
}

bool ensureAuthorized() {
  if (isWebAuthorized()) return true;
  server.send(403, "text/plain", "Forbidden");
  return false;
}

void resetSequencer() {
  pumpSeqState = PSEQ_IDLE;
  pumpSeqMs = 0;
  pendingStartMode = START_NONE;
  activeStartMode = START_NONE;
  pendingStop = false;
}

void stopAllPumps(const String& reason) {
  for (int i = 0; i < numRelays; i++) digitalWrite(relayPins[i], LOW);
  masterOn = false;
  resetSequencer();
  addEventLog(reason);
}

void queueStartMode(StartMode mode, const String& actionLabel) {
  clearPressureAlertLogs();
  pendingStartMode = mode;
  pendingStop = false;
  addEventLog("REQUEST: " + actionLabel);
}

void configureLocalTime() {
  setenv("TZ", TZ_INFO, 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  lastNtpSyncAttemptMs = millis();
}

void ensureTimeConfigured() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 100)) return;

  unsigned long now = millis();
  if (lastNtpSyncAttemptMs == 0 || now - lastNtpSyncAttemptMs >= NTP_RETRY_PERIOD_MS) {
    Serial.println("NTP retry");
    configureLocalTime();
  }
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid, password);
  wifiConnecting = true;
  wifiConnectAttemptStartMs = millis();
  lastWiFiReconnectAttemptMs = millis();
}

void ensureWiFiConnected() {
  wl_status_t status = WiFi.status();
  unsigned long now = millis();

  if (status == WL_CONNECTED) {
    if (wifiConnecting) {
      wifiConnecting = false;
      addEventLog("WIFI: connected, IP " + WiFi.localIP().toString());
      Serial.println("Connected to WiFi");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
    return;
  }

  if (!wifiConnecting) {
    wifiConnecting = true;
    wifiConnectAttemptStartMs = now;
  }

  if (now - lastWiFiReconnectAttemptMs >= WIFI_RECONNECT_PERIOD_MS) {
    lastWiFiReconnectAttemptMs = now;
    Serial.println("WIFI: reconnect attempt");
    WiFi.disconnect(false, false);
    WiFi.begin(ssid, password);
  }

  if (wifiConnecting && now - wifiConnectAttemptStartMs >= WIFI_CONNECT_TIMEOUT_MS) {
    addEventLog("WIFI: still disconnected after reconnect timeout, keeping controller running");
    wifiConnectAttemptStartMs = now;
  }
}

void sendNtfyNotification(const String& title, const String& message, const String& priority, const String& tags) {
  if (WiFi.status() != WL_CONNECTED) {
    addEventLog("NTFY: skipped notification, WiFi disconnected");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = String(NTFY_SERVER) + "/" + String(NTFY_TOPIC) + "/publish?";
  url += "title=" + urlEncode(title);
  url += "&message=" + urlEncode(message);
  url += "&priority=" + urlEncode(priority);
  if (tags.length() > 0) {
    url += "&tags=" + urlEncode(tags);
  }

  if (!http.begin(client, url)) {
    addEventLog("NTFY: begin failed");
    return;
  }

  int httpCode = http.GET();
  if (httpCode > 0) {
    addEventLog("NTFY: sent (" + String(httpCode) + ") " + title);
    Serial.println("NTFY sent: " + title);
  } else {
    addEventLog("NTFY: failed to send " + title + " err=" + String(httpCode));
    Serial.println("NTFY send failed");
  }

  http.end();
}

void checkHighPressureTrip() {
  if (raw > RAW_LIMIT) {
    if (rawOverStartMs == 0) {
      rawOverStartMs = millis();
      return;
    }

    if (millis() - rawOverStartMs >= RAW_OVER_LIMIT_MS) {
      stopAllPumps("ALERT: High pressure trip, pumps shut off (raw=" + String(raw) + ")");
      rawOverStartMs = 0;

      pressureTripLog  = "<div style='margin-top:20px;padding:10px;border:2px solid red;color:red;'>";
      pressureTripLog += "<b>ALERT: PUMPS SHUTOFF - HIGH PRESSURE</b><br>";
      pressureTripLog += "Time: " + nowString() + "<br>";
      pressureTripLog += "Raw: " + String(raw) + " (limit " + String(RAW_LIMIT) + ")<br>";
      pressureTripLog += "</div>";

      sendNtfyNotification(
        "MxIrrigation high pressure alert",
        "Pumps stopped due to high pressure at " + nowString() +
        ". Raw=" + String(raw) +
        ", limit=" + String(RAW_LIMIT),
        "high",
        "warning,pressure"
      );

      Serial.println("ALERT: High pressure trip. Pumps shut off. raw=" + String(raw));
    }
  } else {
    rawOverStartMs = 0;
  }
}

void checkLowPressureTripWhilePumpsOn() {
  if (!masterOn) {
    rawUnderStartMs = 0;
    return;
  }

  if (raw < RAW_MIN_LIMIT) {
    if (rawUnderStartMs == 0) {
      rawUnderStartMs = millis();
      return;
    }

    if (millis() - rawUnderStartMs >= RAW_UNDER_LIMIT_MS) {
      stopAllPumps("ALERT: Low pressure trip, pumps shut off (raw=" + String(raw) + ")");
      rawUnderStartMs = 0;

      lowPressureTripLog  = "<div style='margin-top:20px;padding:10px;border:2px solid orange;color:orange;'>";
      lowPressureTripLog += "<b>ALERT: PUMPS SHUTOFF - LOW PRESSURE</b><br>";
      lowPressureTripLog += "Time: " + nowString() + "<br>";
      lowPressureTripLog += "Raw: " + String(raw) + " (min " + String(RAW_MIN_LIMIT) + ")<br>";
      lowPressureTripLog += "</div>";

      sendNtfyNotification(
        "MxIrrigation low pressure alert",
        "Pumps stopped due to low pressure at " + nowString() +
        ". Raw=" + String(raw) +
        ", min=" + String(RAW_MIN_LIMIT),
        "high",
        "warning,pressure"
      );

      Serial.println("ALERT: Low pressure trip. Pumps shut off. raw=" + String(raw));
    }
  } else {
    rawUnderStartMs = 0;
  }
}

void monitorPingAndRebootWhenPumpsOff() {
  if (masterOn) {
    pingFailCount = 0;
    lastPingMs = 0;
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    pingFailCount = 0;
    return;
  }

  unsigned long now = millis();

  if (now - bootMs < PING_START_DELAY_MS) return;
  if (lastPingMs != 0 && (now - lastPingMs < PING_PERIOD_MS)) return;
  lastPingMs = now;

  bool ok = Ping.ping("192.168.5.40", 1);

  if (!ok) {
    pingFailCount++;
    Serial.printf("PING FAIL %d/%d\n", pingFailCount, PING_FAIL_REBOOT_COUNT);
  } else {
    pingFailCount = 0;
    Serial.println("PING OK");
  }

  if (pingFailCount >= PING_FAIL_REBOOT_COUNT) {
    if (lastPingTriggeredRestartMs != 0 && (now - lastPingTriggeredRestartMs < PING_REBOOT_COOLDOWN_MS)) {
      addEventLog("WATCHDOG: ping failures detected, reboot suppressed by cooldown");
      pingFailCount = 0;
      return;
    }

    addEventLog("RESTART: ping failed " + String(PING_FAIL_REBOOT_COUNT) + " times (pumps off)");
    Serial.println("RESTART: ping failed repeatedly (pumps off)");
    lastPingTriggeredRestartMs = now;
    delay(100);
    ESP.restart();
  }
}

void runPumpSequencer() {
  unsigned long now = millis();

  if (pendingStop) {
    stopAllPumps("PUMPS OFF (stop requested)");
    return;
  }

  if (pumpSeqState == PSEQ_IDLE && pendingStartMode != START_NONE) {
    activeStartMode = pendingStartMode;
    pendingStartMode = START_NONE;

    switch (activeStartMode) {
      case START_MASTER_BOTH:
        digitalWrite(relayPins[3], HIGH); // relay4 ON first
        digitalWrite(relayPins[0], LOW);
        digitalWrite(relayPins[1], LOW);
        digitalWrite(relayPins[2], LOW);
        masterOn = false;
        pumpSeqState = PSEQ_RELAY4_WAIT5S;
        pumpSeqMs = now;
        addEventLog("START: master/on requested (relay4 ON, sequencing both pumps)");
        break;

      case START_MASTER_SOLAR_BOTH:
        digitalWrite(relayPins[3], LOW);
        digitalWrite(relayPins[0], HIGH); // master ON
        digitalWrite(relayPins[1], LOW);
        digitalWrite(relayPins[2], LOW);
        masterOn = true;
        pumpSeqState = PSEQ_MASTER_WAIT2S;
        pumpSeqMs = now;
        addEventLog("START: master-solar/on requested (master ON, sequencing both pumps)");
        break;

      case START_SINGLE_ONLY:
        digitalWrite(relayPins[3], LOW);
        digitalWrite(relayPins[0], HIGH); // master ON
        digitalWrite(relayPins[1], LOW);
        digitalWrite(relayPins[2], LOW);
        masterOn = true;
        pumpSeqState = PSEQ_MASTER_WAIT2S;
        pumpSeqMs = now;
        addEventLog("START: single requested (master ON, pump1 only)");
        break;

      case START_STAGGERED_AUTO_OFF:
        digitalWrite(relayPins[3], LOW);
        digitalWrite(relayPins[0], HIGH); // master ON
        digitalWrite(relayPins[1], HIGH); // pump1 ON immediately
        digitalWrite(relayPins[2], LOW);  // pump2 OFF initially
        masterOn = true;
        pumpSeqState = PSEQ_STAGGERED_PUMP2_WAIT;
        pumpSeqMs = now;
        addEventLog("START: staggered auto-off requested (pump1 ON now, pump2 in 5s, pump2 OFF after 40s)");
        break;

      case START_NONE:
      default:
        activeStartMode = START_NONE;
        break;
    }
  }

  switch (pumpSeqState) {
    case PSEQ_IDLE:
      break;

    case PSEQ_RELAY4_WAIT5S:
      if (now - pumpSeqMs >= 5000) {
        digitalWrite(relayPins[0], HIGH); // master ON
        masterOn = true;
        pumpSeqState = PSEQ_MASTER_WAIT2S;
        pumpSeqMs = now;
        addEventLog("SEQ: master ON (after relay4 wait 5s)");
      }
      break;

    case PSEQ_MASTER_WAIT2S:
      if (now - pumpSeqMs >= 2000) {
        digitalWrite(relayPins[1], HIGH); // pump1 ON
        addEventLog("SEQ: pump1 ON");

        if (activeStartMode == START_SINGLE_ONLY) {
          digitalWrite(relayPins[2], LOW);
          pumpSeqState = PSEQ_IDLE;
          activeStartMode = START_NONE;
          addEventLog("SEQ: single mode complete (pump2 OFF, idle)");
        } else {
          pumpSeqState = PSEQ_PUMP1_WAIT4S;
          pumpSeqMs = now;
        }
      }
      break;

    case PSEQ_PUMP1_WAIT4S:
      if (now - pumpSeqMs >= 4000) {
        digitalWrite(relayPins[2], HIGH); // pump2 ON
        pumpSeqState = PSEQ_IDLE;
        activeStartMode = START_NONE;
        addEventLog("SEQ: pump2 ON (sequence complete, idle)");
      }
      break;

    case PSEQ_STAGGERED_PUMP2_WAIT:
      if (now - pumpSeqMs >= 5000) {
        digitalWrite(relayPins[2], HIGH); // pump2 ON
        pumpSeqState = PSEQ_STAGGERED_PUMP2_HOLD;
        pumpSeqMs = now;
        addEventLog("SEQ: pump2 ON (5s after pump1)");
      }
      break;

    case PSEQ_STAGGERED_PUMP2_HOLD:
      if (now - pumpSeqMs >= 40000) {
        digitalWrite(relayPins[2], LOW); // pump2 OFF
        pumpSeqState = PSEQ_IDLE;
        activeStartMode = START_NONE;
        addEventLog("SEQ: pump2 OFF after 40s hold (pump1 remains ON)");
      }
      break;
  }
}

void handleRoot() {
  bool authorized = isWebAuthorized();

  String wifiState = (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "DISCONNECTED";
  String wifiIP = WiFi.localIP().toString();
  String wifiGW = WiFi.gatewayIP().toString();
  String wifiMask = WiFi.subnetMask().toString();
  String wifiSSID = WiFi.SSID();
  int wifiRSSI = WiFi.RSSI();
  String wifiMAC = WiFi.macAddress();

  String roothtml;
  roothtml += "<!DOCTYPE HTML>";
  roothtml += "<html><head>";
  roothtml += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" /> ";
  roothtml += "<META HTTP-EQUIV=\"CACHE-CONTROL\" CONTENT=\"NO-CACHE\">";
  roothtml += "<meta http-equiv=\"Expires\" content=\"0\">";
  roothtml += "<title>MxIrrigation - Pump control</title></head><body>";
  roothtml += "<div align=center>";

  roothtml += "<h2>MxIrrigation</h2>";
  roothtml += "<div>FW: ";
  roothtml += FW_VERSION;
  roothtml += "</div>";
  roothtml += "<div>Now: " + htmlEscape(nowString()) + "</div>";
  roothtml += "<div>Uptime: " + htmlEscape(formatUptime(millis())) + "</div>";

  roothtml += "<hr style='max-width:900px;'>";

  roothtml += "<b>WiFi</b><br>";
  roothtml += "Status: " + htmlEscape(wifiState) + "<br>";
  roothtml += "SSID: " + htmlEscape(wifiSSID) + "<br>";
  roothtml += "RSSI: " + String(wifiRSSI) + " dBm<br>";
  roothtml += "IP: " + htmlEscape(wifiIP) + "<br>";
  roothtml += "Gateway: " + htmlEscape(wifiGW) + "<br>";
  roothtml += "Subnet: " + htmlEscape(wifiMask) + "<br>";
  roothtml += "MAC: " + htmlEscape(wifiMAC) + "<br>";

  roothtml += "<hr style='max-width:900px;'>";

  roothtml += "Stato pompe:";
  if (masterOn) roothtml += " <font color=red>ACCESE</font>";
  else          roothtml += " <font color=green>SPENTE</font>";

  roothtml += "<br><br>Pressione: raw ";
  roothtml += raw;
  roothtml += " | bar ";
  roothtml += String(valore, 2);

  roothtml += "<br><br><a href=/logs>View logs</a>";
  roothtml += "<br><a href=/pressure>View pressure (last hour)</a>";

  if (authorized) {
#if ENABLE_WEB_PASSWORD
    String pwq = String("?") + WEB_PW_PARAM + "=" + WEB_PASSWORD;
#else
    String pwq = "";
#endif
    roothtml += "<br><br><a href=/master/on" + pwq + ">Accendi entrambe</a>";
    roothtml += "<br><br><a href=/master-solar/on" + pwq + ">Accendi master solar</a>";
    roothtml += "<br><br><a href=/single" + pwq + ">Accendi singola</a>";
    roothtml += "<br><br><a href=/staggered-auto" + pwq + ">Accendi doppia sequenza 5s / spegni seconda dopo 40s</a>";
    roothtml += "<br><br><a href=/master/off" + pwq + ">SPEGNI</a>";
    roothtml += "<br><br><button onclick=\"if(confirm('Reboot controller now?')){window.location='/reboot" + pwq + "';}\">Reboot controller</button>";
  } else {
#if ENABLE_WEB_PASSWORD
    roothtml += "<br><br><font color=gray>Controls hidden. Use /?pw=**** to enable.</font>";
#endif
  }

  roothtml += pressureTripLog;
  roothtml += lowPressureTripLog;

  roothtml += "</div></body></html>";
  server.send(200, "text/html", roothtml);
}

void handleLogs() {
  String html;
  html += "<!DOCTYPE HTML><html><head>";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" /> ";
  html += "<META HTTP-EQUIV=\"CACHE-CONTROL\" CONTENT=\"NO-CACHE\">";
  html += "<meta http-equiv=\"Expires\" content=\"0\">";
  html += "<title>MxIrrigation - Logs</title></head><body>";
  html += "<div align=center>";
  html += "<h2>MxIrrigation - Event Logs (last 10 since boot)</h2>";
  html += "<div style='margin-bottom:10px;'><a href='/'>Back</a></div>";

  html += "<table border='1' cellpadding='6' cellspacing='0' style='border-collapse:collapse;text-align:left;'>";
  html += "<tr><th>#</th><th>Event</th></tr>";

  if (eventLogCount == 0) {
    html += "<tr><td colspan='2'><i>No events yet</i></td></tr>";
  } else {
    int start = (eventLogHead - eventLogCount);
    while (start < 0) start += EVENT_LOG_CAPACITY;

    for (int i = 0; i < eventLogCount; i++) {
      int idx = (start + i) % EVENT_LOG_CAPACITY;
      html += "<tr><td>";
      html += String(i + 1);
      html += "</td><td><pre style='margin:0;white-space:pre-wrap;'>";
      html += htmlEscape(eventLogs[idx]);
      html += "</pre></td></tr>";
    }
  }

  html += "</table>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handlePressure() {
  String html;
  html += "<!DOCTYPE HTML><html><head>";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" /> ";
  html += "<META HTTP-EQUIV=\"CACHE-CONTROL\" CONTENT=\"NO-CACHE\">";
  html += "<meta http-equiv=\"Expires\" content=\"0\">";
  html += "<title>MxIrrigation - Pressure</title>";
  html += "<style>body{font-family:Arial;} canvas{border:1px solid #444;}</style>";
  html += "</head><body>";
  html += "<div align=center>";
  html += "<h2>Pressure (bar) - Last hour (1 sample/sec)</h2>";
  html += "<div style='margin-bottom:10px;'><a href='/'>Back</a></div>";

  uint16_t curCb = rawToCentibar(raw);
  float curBar = curCb / 100.0f;
  html += "<div>Now: " + htmlEscape(nowString()) + " | current: " + String(curBar, 2) + " bar</div><br>";

  html += "<canvas id='c' width='1000' height='300'></canvas>";

  html += "<script>\n";
  html += "const samplesCb=[";
  if (pressureHistCount > 0) {
    int start = (pressureHistHead - pressureHistCount);
    while (start < 0) start += PRESSURE_HISTORY_SECONDS;
    for (int i = 0; i < pressureHistCount; i++) {
      int idx = (start + i) % PRESSURE_HISTORY_SECONDS;
      html += String((unsigned int)pressureHistoryCb[idx]);
      if (i != pressureHistCount - 1) html += ",";
    }
  }
  html += "];\n";

  html += R"JS(
const samples = samplesCb.map(v => v/100.0); // bar
const canvas=document.getElementById('c');
const ctx=canvas.getContext('2d');
const W=canvas.width, H=canvas.height;

function draw(){
  ctx.clearRect(0,0,W,H);

  ctx.strokeStyle='#444';
  ctx.strokeRect(0.5,0.5,W-1,H-1);

  if(samples.length<2){
    ctx.fillStyle='#666';
    ctx.fillText('Not enough samples yet', 10, 20);
    return;
  }

  let min=Number.POSITIVE_INFINITY, max=Number.NEGATIVE_INFINITY;
  for(const v of samples){ if(v<min)min=v; if(v>max)max=v; }
  if(min===max){ min-=0.01; max+=0.01; }

  const padL=60, padR=10, padT=10, padB=25;
  const plotW=W-padL-padR, plotH=H-padT-padB;

  ctx.strokeStyle='#999';
  ctx.beginPath();
  ctx.moveTo(padL, padT);
  ctx.lineTo(padL, padT+plotH);
  ctx.lineTo(padL+plotW, padT+plotH);
  ctx.stroke();

  ctx.fillStyle='#333';
  ctx.font='12px Arial';
  ctx.fillText(max.toFixed(2)+' bar', 5, padT+10);
  ctx.fillText(min.toFixed(2)+' bar', 5, padT+plotH);

  ctx.fillText('60 min ago', padL, H-5);
  ctx.fillText('now', padL+plotW-25, H-5);

  ctx.strokeStyle='#0a6';
  ctx.lineWidth=1;
  ctx.beginPath();
  for(let i=0;i<samples.length;i++){
    const x=padL + (i/(samples.length-1))*plotW;
    const y=padT + (1 - (samples[i]-min)/(max-min))*plotH;
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();
}
draw();
)JS";

  html += "\n</script>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleReboot() {
  if (!ensureAuthorized()) return;
  addEventLog("RESTART: reboot requested from web UI");
  server.send(200, "text/html", "<font color=orange size=5>rebooting...</font>");
  delay(250);
  ESP.restart();
}

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.begin(115200);
  while (!Serial) delay(1);
  Serial.println("\n            Generator Admin by MxSolutions.it");
  Serial.println("                  www.mxsolutions.it");
  Serial.println("                 All Rights Reserved");
  Serial.print("                 Versione firmware: ");
  Serial.println(FW_VERSION);
  Serial.println("------------------------------------------------------------");

  bootMs = millis();
  lastPingMs = 0;
  pingFailCount = 0;
  lastPingTriggeredRestartMs = 0;

  pressureTimer = millis() + 1000;
  pinMode(analogPin, INPUT);

  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }

  for (int i = 0; i < PRESSURE_HISTORY_SECONDS; i++) pressureHistoryCb[i] = 0;
  pressureHistHead = 0;
  pressureHistCount = 0;

  setupWiFi();
  configureLocalTime();

  addEventLog("BOOT: firmware " + String(FW_VERSION) + " started");

  server.on("/", handleRoot);
  server.on("/logs", handleLogs);
  server.on("/pressure", handlePressure);

  server.on("/master/on", HTTP_GET, []() {
    if (!ensureAuthorized()) return;
    queueStartMode(START_MASTER_BOTH, "master/on");
    server.send(200, "text/html", "<font color=red size=5>master starting...</font>");
  });

  server.on("/master-solar/on", HTTP_GET, []() {
    if (!ensureAuthorized()) return;
    queueStartMode(START_MASTER_SOLAR_BOTH, "master-solar/on");
    server.send(200, "text/html", "<font color=red size=5>master solar starting...</font>");
  });

  server.on("/single", HTTP_GET, []() {
    if (!ensureAuthorized()) return;
    queueStartMode(START_SINGLE_ONLY, "single");
    server.send(200, "text/html", "<font color=red size=5>single starting...</font>");
  });

  server.on("/staggered-auto", HTTP_GET, []() {
    if (!ensureAuthorized()) return;
    queueStartMode(START_STAGGERED_AUTO_OFF, "staggered-auto");
    server.send(200, "text/html", "<font color=red size=5>staggered auto sequence starting...</font>");
  });

  server.on("/master/off", HTTP_GET, []() {
    if (!ensureAuthorized()) return;
    pendingStop = true;
    pendingStartMode = START_NONE;
    server.send(200, "text/html", "<font color=green size=5>stopping...</font>");
  });

  server.on("/reboot", HTTP_GET, handleReboot);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  ensureWiFiConnected();
  ensureTimeConfigured();
  runPumpSequencer();
  monitorPingAndRebootWhenPumpsOff();

  if (millis() >= pressureTimer) {
    pressureTimer = millis() + 1000;

    int r1 = analogRead(analogPin);
    int r2 = analogRead(analogPin);
    int r3 = analogRead(analogPin);
    raw = median3(r1, r2, r3);

    uint16_t cb = rawToCentibar(raw);
    addPressureSampleCentibar(cb);

    Serial.print("Raw: ");
    Serial.println(raw);

    valore = cb / 100.0f;

    checkHighPressureTrip();
    checkLowPressureTripWhilePumpsOn();
  }

  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long ledMs = 0;
    if (millis() - ledMs >= 1000) {
      ledMs = millis();
      digitalWrite(ledPin, !digitalRead(ledPin));
    }
  } else {
    digitalWrite(ledPin, HIGH);
  }
}
