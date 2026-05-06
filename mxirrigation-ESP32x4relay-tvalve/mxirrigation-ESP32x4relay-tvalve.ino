/*
  MxIrrigation T Valve Controller
  author MxSolutions.it
  version 0.4

  ESP32 + 4 relays (only relay 1 and relay 2 used)
  - Relay pins: {21, 19, 18, 5} ACTIVE HIGH
  - Never allow both working relays ON at the same time:
      /on  -> Relay1 ON for 15s, node status = ON
      /off -> Relay2 ON for 15s, node status = OFF
  - On boot: pulse Relay2 ON for 15s, status = OFF

  Network:
  - Static IP: 192.168.5.48
  - Gateway : 192.168.5.1
  - WiFi stays connected with modem-sleep enabled (lower power, near-instant response)

  Watchdog:
  - Ping gateway every 60s ONLY when status == OFF
  - If 4 consecutive ping failures -> reboot

  Power saving changes (v0.4):
  - Disable Bluetooth
  - Reduce CPU frequency to 80 MHz
  - Enable WiFi sleep (modem-sleep)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPping.h>

// For btStop()
#include "esp_bt.h"

// ----------------- USER CONFIG -----------------
const char* WIFI_SSID     = "WMPHOUSE";
const char* WIFI_PASSWORD = "motocross";

IPAddress localIP(192, 168, 5, 48);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(192, 168, 5, 1);

const uint32_t RELAY_PULSE_MS = 15000UL;

// Ping every minute only when status is OFF
const uint32_t PING_PERIOD_MS = 60000UL;
const uint8_t  MAX_PING_FAILS = 4;

// Relay pins (4 relays present; only first two used)
const int relayPins[] = {21, 19, 18, 5};
static_assert(sizeof(relayPins) / sizeof(relayPins[0]) == 4, "relayPins must have 4 entries");

// Active HIGH relays
const uint8_t RELAY_ON  = HIGH;
const uint8_t RELAY_OFF = LOW;

// ----------------- STATE -----------------
WebServer server(80);

enum NodeStatus : uint8_t { STATUS_OFF = 0, STATUS_ON = 1 };
volatile NodeStatus nodeStatus = STATUS_OFF;

enum PulsedRelay : uint8_t { PULSE_NONE = 0, PULSE_R1 = 1, PULSE_R2 = 2 };

struct RelayPulse {
  bool active = false;
  PulsedRelay which = PULSE_NONE;
  uint32_t endAtMs = 0;
} pulse;

uint32_t lastPingAtMs = 0;
uint8_t consecutivePingFails = 0;

// ----------------- RELAY HELPERS -----------------
void allRelaysOff() {
  for (int i = 0; i < 4; i++) digitalWrite(relayPins[i], RELAY_OFF);
}

void relay1On()  { digitalWrite(relayPins[0], RELAY_ON);  }
void relay1Off() { digitalWrite(relayPins[0], RELAY_OFF); }
void relay2On()  { digitalWrite(relayPins[1], RELAY_ON);  }
void relay2Off() { digitalWrite(relayPins[1], RELAY_OFF); }

void startRelayPulse(PulsedRelay which, uint32_t durationMs) {
  // Enforce never-both-on: OFF everything first
  pulse.active = false;
  pulse.which = PULSE_NONE;
  allRelaysOff();

  if (which == PULSE_R1) relay1On();
  else if (which == PULSE_R2) relay2On();

  pulse.active = true;
  pulse.which = which;
  pulse.endAtMs = millis() + durationMs;
}

void serviceRelayPulse() {
  if (!pulse.active) return;

  if ((int32_t)(millis() - pulse.endAtMs) >= 0) {
    if (pulse.which == PULSE_R1) relay1Off();
    else if (pulse.which == PULSE_R2) relay2Off();

    pulse.active = false;
    pulse.which = PULSE_NONE;
  }
}

// ----------------- WEB HELPERS -----------------
String statusToString(NodeStatus st) {
  return (st == STATUS_ON) ? F("ON") : F("OFF");
}

void handleRoot() {
  String page;
  page.reserve(1000);

  page += F("<!doctype html><html><head><meta charset='utf-8'>");
  page += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  page += F("<title>MxIrrigation</title></head><body>");
  page += F("<h2>MxIrrigation T Valve Controller</h2>");

  page += F("<p><b>Status:</b> ");
  page += statusToString(nodeStatus);
  page += F("</p>");

  page += F("<ul>");
  page += F("<li><a href='/on'>ON</a> (Relay 1 for 15s)</li>");
  page += F("<li><a href='/off'>OFF</a> (Relay 2 for 15s)</li>");
  page += F("</ul>");

  page += F("<hr><p><b>IP:</b> ");
  page += WiFi.localIP().toString();
  page += F("<br><b>Gateway:</b> ");
  page += gateway.toString();
  page += F("<br><b>WiFi RSSI:</b> ");
  page += String(WiFi.RSSI());
  page += F(" dBm</p>");

  page += F("</body></html>");

  server.send(200, "text/html; charset=utf-8", page);
}

void handleOn() {
  nodeStatus = STATUS_ON;
  consecutivePingFails = 0; // optional: reset watchdog failures on activity
  startRelayPulse(PULSE_R1, RELAY_PULSE_MS);
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "OK");
}

void handleOff() {
  nodeStatus = STATUS_OFF;
  consecutivePingFails = 0;
  startRelayPulse(PULSE_R2, RELAY_PULSE_MS);
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "OK");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ----------------- PING + REBOOT -----------------
void servicePing() {
  // Only ping every minute when the node is OFF (per requirement)
  if (nodeStatus != STATUS_OFF) return;

  uint32_t now = millis();
  if (lastPingAtMs != 0 && (uint32_t)(now - lastPingAtMs) < PING_PERIOD_MS) return;
  lastPingAtMs = now;

  bool ok = (WiFi.status() == WL_CONNECTED) && Ping.ping(gateway, 1);

  if (ok) {
    consecutivePingFails = 0;
  } else {
    consecutivePingFails++;
    if (consecutivePingFails >= MAX_PING_FAILS) {
      allRelaysOff();
      delay(50);
      ESP.restart();
    }
  }
}

// ----------------- WIFI + POWER -----------------
void applyPowerSavings() {
  // 1) Disable Bluetooth stack (saves power even if unused)
  btStop();
  esp_bt_controller_disable();

  // 2) Reduce CPU frequency (near-instant web response still OK for this workload)
  setCpuFrequencyMhz(80);

  // 3) Enable WiFi modem-sleep while connected
  WiFi.setSleep(true);
}

void connectWiFiStatic() {
  WiFi.mode(WIFI_STA);

  // Static IP before begin()
  WiFi.config(localIP, gateway, subnet, dns1);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Bounded connect wait (keeps boot deterministic)
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - start) < 15000UL) {
    delay(250);
  }
}

// ----------------- SETUP/LOOP -----------------
void setup() {
  // GPIO init
  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], RELAY_OFF);
  }

  // Boot behavior
  nodeStatus = STATUS_OFF;
  startRelayPulse(PULSE_R2, RELAY_PULSE_MS);

  connectWiFiStatic();
  applyPowerSavings();

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
  delay(1);
}
