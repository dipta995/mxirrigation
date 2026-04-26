/*
  MxIrrigation by MxSolutions.it
  Author: Nicola Deboni, Mx Solutions
  Firmware version: 1.1.2

  Description
  -----------
  ESP32-based irrigation pump controller with:
  - Web UI (status + control links; optional password to show controls)
  - Non-blocking pump start/stop sequencing (timers instead of delays)
  - Pressure monitoring (high-pressure trip and low-pressure trip)
  - Network watchdog via ICMP ping (reboot after consecutive failures when pumps are off)
  - NTP time for timestamped alert logs displayed on the web root page

  Function overview
  -----------------
  setup()
    - Initializes serial, GPIO pins, relays (OFF), WiFi (static IP), NTP time, and HTTP routes.

  loop()
    - Handles HTTP clients, runs the pump sequencer, runs ping monitor when pumps are off,
      reads pressure every second, and performs safety trips.

  nowString()
    - Returns current date/time as "YYYY-MM-DD HH:MM:SS" from NTP.
      If NTP not set, returns an uptime-based string.

  clearPressureAlertLogs()
    - Clears alert HTML strings and resets pressure timing windows.
      Called when pumps are started again so old alerts disappear from the UI.

  isWebAuthorized()
    - If password feature enabled, checks query string parameter `pw` against WEB_PASSWORD.
      If disabled, always returns true.

  checkHighPressureTrip()
    - Safety trip: if raw pressure remains above RAW_LIMIT continuously for RAW_OVER_LIMIT_MS,
      shuts down pumps and logs an alert to the web UI. Resets timing if pressure drops back.

  checkLowPressureTripWhilePumpsOn()
    - Safety trip: while pumps are ON (masterOn==true), if raw pressure remains below
      RAW_MIN_LIMIT continuously for RAW_UNDER_LIMIT_MS, shuts down pumps and logs an alert.

  monitorPingAndRebootWhenPumpsOff()
    - Network watchdog: starts 60s after boot; while pumps are OFF, pings 192.168.5.40 every
      20s. After 4 consecutive failures, reboots the ESP32.

  runPumpSequencer()
    - Non-blocking relay sequencing:
        /master/on:       relay4 ON -> wait 5s -> master ON -> wait 2s -> pump1 ON -> wait 4s -> pump2 ON
        /master-solar/on: master ON -> wait 2s -> pump1 ON -> wait 4s -> pump2 ON
        /single:          master ON -> wait 2s -> pump1 ON only
        /master/off:      ALL relays OFF immediately

  handleRoot()
    - Builds and serves the main HTML page with current status, pressure, and any alert logs.
      If not authorized, hides control links.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPping.h>
#include <time.h>

const char* FW_VERSION = "1.1.2";

const char *ssid = "WMPSERVICE";
const char *password = "motocross";

IPAddress staticIP(192, 168, 5, 43);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

const int relayPins[] = {21, 19, 18, 5};
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int ledPin = 25;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 30000;
bool counterFlag = false;

// pressure sensor
int analogPin = 36;
int raw = 0;
float valore = 0;
unsigned long pressureTimer;

// status
bool masterOn = false;

// ---- Web UI password (query string) ----
#define ENABLE_WEB_PASSWORD 1
const char* WEB_PASSWORD = "1234";   // <-- change this
const char* WEB_PW_PARAM = "pw";     // URL: /?pw=1234

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

// ---- Non-blocking ping/reboot monitor ----
const unsigned long PING_START_DELAY_MS = 60000;   // start 60s after boot
const unsigned long PING_PERIOD_MS      = 20000;   // every 20s when pumps off
const int PING_FAIL_REBOOT_COUNT        = 4;

unsigned long bootMs = 0;
unsigned long lastPingMs = 0;
int pingFailCount = 0;

// ---- Non-blocking pump sequencer ----
enum PumpSequenceState {
  PSEQ_IDLE = 0,
  PSEQ_RELAY4_WAIT5S,
  PSEQ_START_WAIT2S,
  PSEQ_START_PUMP1_ON_WAIT4S,
  PSEQ_STOP_WAIT500MS
};

PumpSequenceState pumpSeqState = PSEQ_IDLE;
unsigned long pumpSeqMs = 0;

bool pendingMasterStart = false;
bool pendingMasterSolarStart = false;
bool pendingSingleStart = false;
bool pendingStop = false;

bool startBothPumps = false;
bool useRelay4 = false;

// Forward declarations
static int median3(int a, int b, int c);
String nowString();
void clearPressureAlertLogs();
bool isWebAuthorized();
void checkHighPressureTrip();
void checkLowPressureTripWhilePumpsOn();
void monitorPingAndRebootWhenPumpsOff();
void runPumpSequencer();
void handleRoot();

static int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b; // median
}

String nowString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "NTP not set (uptime " + String(millis() / 1000) + "s)";
  }
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
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

void checkHighPressureTrip() {
  if (raw > RAW_LIMIT) {
    if (rawOverStartMs == 0) {
      rawOverStartMs = millis();
      return;
    }

    if (millis() - rawOverStartMs >= RAW_OVER_LIMIT_MS) {
      for (int i = 0; i < numRelays; i++) digitalWrite(relayPins[i], LOW);
      masterOn = false;

      rawOverStartMs = 0;

      pressureTripLog  = "<div style='margin-top:20px;padding:10px;border:2px solid red;color:red;'>";
      pressureTripLog += "<b>ALERT: PUMPS SHUTOFF - HIGH PRESSURE</b><br>";
      pressureTripLog += "Time: " + nowString() + "<br>";
      pressureTripLog += "Raw: " + String(raw) + " (limit " + String(RAW_LIMIT) + ")<br>";
      pressureTripLog += "</div>";

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
      for (int i = 0; i < numRelays; i++) digitalWrite(relayPins[i], LOW);
      masterOn = false;

      rawUnderStartMs = 0;

      lowPressureTripLog  = "<div style='margin-top:20px;padding:10px;border:2px solid orange;color:orange;'>";
      lowPressureTripLog += "<b>ALERT: PUMPS SHUTOFF - LOW PRESSURE</b><br>";
      lowPressureTripLog += "Time: " + nowString() + "<br>";
      lowPressureTripLog += "Raw: " + String(raw) + " (min " + String(RAW_MIN_LIMIT) + ")<br>";
      lowPressureTripLog += "</div>";

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
    Serial.println("RESTART: ping failed 4 times (pumps off)");
    ESP.restart();
  }
}

void runPumpSequencer() {
  unsigned long now = millis();

  // Stop request has priority
  if (pendingStop && pumpSeqState == PSEQ_IDLE) {
    for (int i = 0; i < numRelays; i++) digitalWrite(relayPins[i], LOW);
    masterOn = false;

    pumpSeqState = PSEQ_IDLE;
    pendingStop = false;

    startBothPumps = false;
    useRelay4 = false;
  }

  // Start requests
  if (pendingMasterStart && pumpSeqState == PSEQ_IDLE) {
    useRelay4 = true;
    startBothPumps = true;

    digitalWrite(relayPins[3], HIGH); // relay 4 ON first
    pumpSeqState = PSEQ_RELAY4_WAIT5S;
    pumpSeqMs = now;

    pendingMasterStart = false;
  }

  if (pendingMasterSolarStart && pumpSeqState == PSEQ_IDLE) {
    useRelay4 = false;
    startBothPumps = true;

    digitalWrite(relayPins[3], LOW);  // ensure relay4 OFF
    digitalWrite(relayPins[0], HIGH); // master ON
    masterOn = true;

    pumpSeqState = PSEQ_START_WAIT2S;
    pumpSeqMs = now;

    pendingMasterSolarStart = false;
  }

  if (pendingSingleStart && pumpSeqState == PSEQ_IDLE) {
    useRelay4 = false;
    startBothPumps = false;

    digitalWrite(relayPins[3], LOW);  // ensure relay4 OFF
    digitalWrite(relayPins[0], HIGH); // master ON
    masterOn = true;

    pumpSeqState = PSEQ_START_WAIT2S;
    pumpSeqMs = now;

    pendingSingleStart = false;
  }

  // State progression
  switch (pumpSeqState) {
    case PSEQ_IDLE:
      break;

    case PSEQ_RELAY4_WAIT5S:
      if (now - pumpSeqMs >= 5000) {
        digitalWrite(relayPins[0], HIGH); // master ON after 5s pause
        masterOn = true;

        pumpSeqState = PSEQ_START_WAIT2S;
        pumpSeqMs = now;
      }
      break;

    case PSEQ_START_WAIT2S:
      if (now - pumpSeqMs >= 2000) {
        digitalWrite(relayPins[1], HIGH); // pump1

        if (startBothPumps) {
          pumpSeqState = PSEQ_START_PUMP1_ON_WAIT4S;
          pumpSeqMs = now;
        } else {
          digitalWrite(relayPins[2], LOW); // ensure pump2 stays off
          pumpSeqState = PSEQ_IDLE;
        }
      }
      break;

    case PSEQ_START_PUMP1_ON_WAIT4S:
      if (now - pumpSeqMs >= 4000) {
        digitalWrite(relayPins[2], HIGH); // pump2
        pumpSeqState = PSEQ_IDLE;
      }
      break;

    case PSEQ_STOP_WAIT500MS:
      pumpSeqState = PSEQ_IDLE;
      break;
  }
}

void handleRoot() {
  bool authorized = isWebAuthorized();

  String roothtml;
  roothtml += "<!DOCTYPE HTML>";
  roothtml += "<html><head>";
  roothtml += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" /> ";
  roothtml += "<META HTTP-EQUIV=\"CACHE-CONTROL\" CONTENT=\"NO-CACHE\">";
  roothtml += "<meta http-equiv=\"Expires\" content=\"0\">";
  roothtml += "<title>MxIrrigation - Pump control</title></head><body>";
  roothtml += "<div align=center>Stato pompe:";
  if (masterOn) roothtml += " <font color=red>ACCESE</font>";
  else          roothtml += "<font color=green>SPENTE</font>";

  roothtml += "<br><br>Pressione: raw ";
  roothtml += raw;
  roothtml += " | bar ";
  roothtml += valore;

  if (authorized) {
#if ENABLE_WEB_PASSWORD
    String pwq = String("?") + WEB_PW_PARAM + "=" + WEB_PASSWORD;
#else
    String pwq = "";
#endif
    roothtml += "<br><br><a href=/master/on" + pwq + ">Accendi entrambe</a>";
    roothtml += "<br><br><a href=/master-solar/on" + pwq + ">Accendi master solar</a>";
    roothtml += "<br><br><a href=/single" + pwq + ">Accendi singola</a>";
    roothtml += "<br><br><a href=/master/off" + pwq + ">SPEGNI</a>";
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

  pressureTimer = millis() + 1000;
  pinMode(analogPin, INPUT);

  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
  }

  for (int i = 0; i < numRelays; i++) {
    digitalWrite(relayPins[i], LOW);
  }

  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (counterFlag == false) {
      startMillis = millis();
      counterFlag = true;
    } else {
      currentMillis = millis();
      if (currentMillis - startMillis >= period) {
        Serial.println("RESTART: wifi connect timeout");
        ESP.restart();
      }
    }
  }

  digitalWrite(ledPin, HIGH);
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  server.on("/", handleRoot);

  server.on("/master/on", HTTP_GET, []() {
    clearPressureAlertLogs();
    pendingMasterStart = true;
    pendingMasterSolarStart = false;
    pendingSingleStart = false;
    pendingStop = false;
    server.send(200, "text/html", "<font color=red size=5>master starting...</font>");
  });

  server.on("/master-solar/on", HTTP_GET, []() {
    clearPressureAlertLogs();
    pendingMasterSolarStart = true;
    pendingMasterStart = false;
    pendingSingleStart = false;
    pendingStop = false;
    server.send(200, "text/html", "<font color=red size=5>master solar starting...</font>");
  });

  server.on("/single", HTTP_GET, []() {
    clearPressureAlertLogs();
    pendingSingleStart = true;
    pendingMasterStart = false;
    pendingMasterSolarStart = false;
    pendingStop = false;
    server.send(200, "text/html", "<font color=red size=5>single starting...</font>");
  });

  server.on("/master/off", HTTP_GET, []() {
    pendingStop = true;
    pendingMasterStart = false;
    pendingMasterSolarStart = false;
    pendingSingleStart = false;
    server.send(200, "text/html", "<font color=green size=5>stopping...</font>");
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  runPumpSequencer();
  monitorPingAndRebootWhenPumpsOff();

  if (millis() >= pressureTimer) {
    pressureTimer = millis() + 1000;

    int r1 = analogRead(analogPin);
    int r2 = analogRead(analogPin);
    int r3 = analogRead(analogPin);
    raw = median3(r1, r2, r3);

    Serial.print("Raw: ");
    Serial.println(raw);

    valore = map(raw, 0, 2095, 0, 10);

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
