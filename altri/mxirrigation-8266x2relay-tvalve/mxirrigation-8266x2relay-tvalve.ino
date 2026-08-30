/*
  Same behavior as before, but relay control is done by sending hex commands on Serial.

  Relay module protocol (given):
    Relay1 ON  : A0 01 01 A2
    Relay1 OFF : A0 01 00 A1
    Relay2 ON  : A0 02 01 A3
    Relay2 OFF : A0 02 00 A2

  IMPORTANT:
    - Set SERIAL_BAUD to whatever your relay-board expects (commonly 9600).
    - Wire ESP8266 UART TX -> relay-board RX (and common GND).
    - If you need RX too, wire RX accordingly; this sketch only transmits.
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266Ping.h>

// ----------------- USER CONFIG -----------------
const char* WIFI_SSID     = "WMPHOME";
const char* WIFI_PASSWORD = "motocross";

IPAddress localIP(192, 168, 5, 48);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(192, 168, 5, 1);
IPAddress dns2(8, 8, 8, 8);

const uint32_t RELAY_PULSE_MS = 15000UL;
const uint32_t PING_PERIOD_MS = 30000UL;

const uint32_t SERIAL_BAUD = 9600; // change if your relay board uses a different baud

// ----------------- RELAY COMMANDS -----------------
// Hex command to send to serial for close relay
byte relON[]  = {0xA0, 0x01, 0x01, 0xA2};
// Hex command to send to serial for open relay
byte relOFF[] = {0xA0, 0x01, 0x00, 0xA1};

// Hex command to send to serial for close relay
byte rel2ON[]  = {0xA0, 0x02, 0x01, 0xA3};
// Hex command to send to serial for open relay
byte rel2OFF[] = {0xA0, 0x02, 0x00, 0xA2};

// ----------------- STATE -----------------
ESP8266WebServer server(80);

enum NodeStatus : uint8_t { STATUS_OFF = 0, STATUS_ON = 1 };
volatile NodeStatus nodeStatus = STATUS_OFF;

// Which relay is currently being pulsed (if any)
enum PulsedRelay : uint8_t { PULSE_NONE = 0, PULSE_R1 = 1, PULSE_R2 = 2 };

struct RelayPulse {
  bool active = false;
  PulsedRelay which = PULSE_NONE;
  uint32_t endAtMs = 0;
} pulse;

uint32_t lastPingAtMs = 0;
uint8_t consecutivePingFails = 0;

// ----------------- SERIAL RELAY HELPERS -----------------
void sendCmd(const byte* cmd, size_t len) {
  Serial.write(cmd, len);
  Serial.flush(); // ensure bytes are pushed out quickly
}

void relay1On()  { sendCmd(relON,  sizeof(relON)); }
void relay1Off() { sendCmd(relOFF, sizeof(relOFF)); }

void relay2On()  { sendCmd(rel2ON,  sizeof(rel2ON)); }
void relay2Off() { sendCmd(rel2OFF, sizeof(rel2OFF)); }

void allRelaysOff() {
  // Enforce "never both ON": proactively open both
  relay1Off();
  relay2Off();
}

void startRelayPulse(PulsedRelay which, uint32_t durationMs) {
  // Stop any existing pulse and ensure both are OFF first (never both ON)
  pulse.active = false;
  pulse.which = PULSE_NONE;
  allRelaysOff();

  // Turn ON only requested relay
  if (which == PULSE_R1) relay1On();
  else if (which == PULSE_R2) relay2On();

  pulse.active = true;
  pulse.which = which;
  pulse.endAtMs = millis() + durationMs;
}

void serviceRelayPulse() {
  if (!pulse.active) return;

  if ((int32_t)(millis() - pulse.endAtMs) >= 0) {
    // Turn OFF whichever relay we turned ON
    if (pulse.which == PULSE_R1) relay1Off();
    else if (pulse.which == PULSE_R2) relay2Off();

    pulse.active = false;
    pulse.which = PULSE_NONE;
    // status stays as requested (ON/OFF)
  }
}

// ----------------- WEB HELPERS -----------------
String htmlEscape(const String& s) {
  String o;
  o.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': o += F("&amp;"); break;
      case '<': o += F("&lt;"); break;
      case '>': o += F("&gt;"); break;
      case '"': o += F("&quot;"); break;
      case '\'': o += F("&#39;"); break;
      default: o += c; break;
    }
  }
  return o;
}

String statusToString(NodeStatus st) {
  return (st == STATUS_ON) ? F("ON") : F("OFF");
}

void handleRoot() {
  String page;
  page.reserve(1200);
  page += F("<!doctype html><html><head><meta charset='utf-8'>");
  page += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  page += F("<title>ESP8266 Relays</title></head><body>");
  page += F("<h2>ESP8266 Relay Controller</h2>");

  page += F("<p><b>Status:</b> ");
  page += htmlEscape(statusToString(nodeStatus));
  page += F("</p>");

  page += F("<ul>");
  page += F("<li><a href='/on'>ON</a> (Relay 1 for 15s)</li>");
  page += F("<li><a href='/off'>OFF</a> (Relay 2 for 15s)</li>");
  page += F("</ul>");

  page += F("<hr>");
  page += F("<p><b>IP:</b> ");
  page += htmlEscape(WiFi.localIP().toString());
  page += F("<br><b>Gateway:</b> ");
  page += htmlEscape(gateway.toString());
  page += F("<br><b>WiFi RSSI:</b> ");
  page += String(WiFi.RSSI());
  page += F(" dBm</p>");

  page += F("</body></html>");

  server.send(200, "text/html; charset=utf-8", page);
}

void handleOn() {
  nodeStatus = STATUS_ON;
  startRelayPulse(PULSE_R1, RELAY_PULSE_MS);
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "OK");
}

void handleOff() {
  nodeStatus = STATUS_OFF;
  startRelayPulse(PULSE_R2, RELAY_PULSE_MS);
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "OK");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ----------------- PING + REBOOT -----------------
void servicePing() {
  uint32_t now = millis();
  if (lastPingAtMs != 0 && (uint32_t)(now - lastPingAtMs) < PING_PERIOD_MS) return;
  lastPingAtMs = now;

  bool ok = (WiFi.status() == WL_CONNECTED) && Ping.ping(gateway, 1);

  if (ok) {
    consecutivePingFails = 0;
  } else {
    consecutivePingFails++;
    if (consecutivePingFails >= 4) {
      allRelaysOff();
      delay(50);
      ESP.restart();
    }
  }
}

// ----------------- WIFI -----------------
void connectWiFiStatic() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);

  // Configure static IP BEFORE begin()
  WiFi.config(localIP, gateway, subnet, dns1, dns2);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - start) < 15000UL) {
    delay(250);
    yield();
  }
}

// ----------------- SETUP/LOOP -----------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(20);

  // Ensure a known state at boot: open both, then "boot action"
  allRelaysOff();

  // On boot: turn relay 2 on for 15s and set status OFF
  nodeStatus = STATUS_OFF;
  startRelayPulse(PULSE_R2, RELAY_PULSE_MS);

  connectWiFiStatic();

  server.on("/", handleRoot);
  server.on("/on", handleOn);
  server.on("/off", handleOff);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop() {
  server.handleClient();
  serviceRelayPulse();
  servicePing();
  yield();
}
