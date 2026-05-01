/*
  MxIrrigation by MxSolutions.it
  Author: Nicola Deboni, Mx Solutions
  Firmware version: 1.1.4

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
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPping.h>
#include <time.h>

const char* FW_VERSION = "1.1.4";

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
float valore = 0; // current pressure in bar (but see mapping in loop)
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
void addEventLog(const String& msg);
void addPressureSampleCentibar(uint16_t centibar);
uint16_t rawToCentibar(int rawValue);
void clearPressureAlertLogs();
bool isWebAuthorized();
void checkHighPressureTrip();
void checkLowPressureTripWhilePumpsOn();
void monitorPingAndRebootWhenPumpsOff();
void runPumpSequencer();
void handleRoot();
void handleLogs();
void handlePressure();

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

// Convert raw ADC to centibar (bar*100).
// Current project mapping (kept consistent with previous map(raw, 0, 2095, 0, 10)):
//   raw=0   -> 0.00 bar
//   raw=2095-> 10.00 bar
uint16_t rawToCentibar(int rawValue) {
  if (rawValue <= 0) return 0;
  if (rawValue >= 2095) return 1000; // 10.00 bar
  // centibar = raw * (1000 / 2095)
  // Use integer math with rounding:
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

      addEventLog("ALERT: High pressure trip, pumps shut off (raw=" + String(raw) + ")");

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

      addEventLog("ALERT: Low pressure trip, pumps shut off (raw=" + String(raw) + ")");

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
    addEventLog("RESTART: ping failed " + String(PING_FAIL_REBOOT_COUNT) + " times (pumps off)");
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

    addEventLog("PUMPS OFF (stop requested)");
  }

  // Start requests
  if (pendingMasterStart && pumpSeqState == PSEQ_IDLE) {
    useRelay4 = true;
    startBothPumps = true;

    digitalWrite(relayPins[3], HIGH); // relay 4 ON first
    pumpSeqState = PSEQ_RELAY4_WAIT5S;
    pumpSeqMs = now;

    pendingMasterStart = false;

    addEventLog("START: master/on requested (relay4 ON, sequencing both pumps)");
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

    addEventLog("START: master-solar/on requested (master ON, sequencing both pumps)");
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

    addEventLog("START: single requested (master ON, pump1 only)");
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

        addEventLog("SEQ: master ON (after relay4 wait 5s)");
      }
      break;

    case PSEQ_START_WAIT2S:
      if (now - pumpSeqMs >= 2000) {
        digitalWrite(relayPins[1], HIGH); // pump1
        addEventLog("SEQ: pump1 ON");

        if (startBothPumps) {
          pumpSeqState = PSEQ_START_PUMP1_ON_WAIT4S;
          pumpSeqMs = now;
        } else {
          digitalWrite(relayPins[2], LOW); // ensure pump2 stays off
          pumpSeqState = PSEQ_IDLE;
          addEventLog("SEQ: single mode complete (pump2 OFF, idle)");
        }
      }
      break;

    case PSEQ_START_PUMP1_ON_WAIT4S:
      if (now - pumpSeqMs >= 4000) {
        digitalWrite(relayPins[2], HIGH); // pump2
        pumpSeqState = PSEQ_IDLE;
        addEventLog("SEQ: pump2 ON (sequence complete, idle)");
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
      html += eventLogs[idx];
      html += "</pre></td></tr>";
    }
  }

  html += "</table>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handlePressure() {
  // Convert centibar -> bar in JS with /100.0
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
  html += "<div>Now: " + nowString() + " | current: " + String(curBar, 2) + " bar</div><br>";

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

  const padL=50, padR=10, padT=10, padB=25;
  const plotW=W-padL-padR, plotH=H-padT-padB;

  // axes
  ctx.strokeStyle='#999';
  ctx.beginPath();
  ctx.moveTo(padL, padT);
  ctx.lineTo(padL, padT+plotH);
  ctx.lineTo(padL+plotW, padT+plotH);
  ctx.stroke();

  // y labels
  ctx.fillStyle='#333';
  ctx.font='12px Arial';
  ctx.fillText(max.toFixed(2)+' bar', 5, padT+10);
  ctx.fillText(min.toFixed(2)+' bar', 5, padT+plotH);

  // x labels
  ctx.fillText('60 min ago', padL, H-5);
  ctx.fillText('now', padL+plotW-25, H-5);

  // polyline
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

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.begin(115200);
  while (!Serial) delay(1);
  Serial.println("\n           MxIrrigation by MxSolutions.it");
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

  for (int i = 0; i < PRESSURE_HISTORY_SECONDS; i++) pressureHistoryCb[i] = 0;
  pressureHistHead = 0;
  pressureHistCount = 0;

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
        addEventLog("RESTART: wifi connect timeout");
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

  addEventLog("BOOT: firmware " + String(FW_VERSION) + " started, IP " + WiFi.localIP().toString());

  server.on("/", handleRoot);
  server.on("/logs", handleLogs);
  server.on("/pressure", handlePressure);

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

    uint16_t cb = rawToCentibar(raw);
    addPressureSampleCentibar(cb);

    Serial.print("Raw: ");
    Serial.println(raw);

    // Keep existing "bar" display behavior but with float (better than integer map)
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
