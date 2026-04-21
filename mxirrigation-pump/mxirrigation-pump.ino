#include <WiFi.h>
#include <WebServer.h>
#include <ESPping.h>
#include <time.h>

const char* FW_VERSION = "1100";

const char *ssid = "WMPSERVICE";
const char *password = "motocross";

IPAddress staticIP(192, 168, 5, 43); // static IP address
IPAddress gateway(192, 168, 5, 1);    // network's gateway address
IPAddress subnet(255, 255, 255, 0);   // network's subnet mask

WebServer server(80);

const int relayPins[] = {21, 19, 18, 5};
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int ledPin = 25;
short int pingCounter = 0;
int offlineCounter = 0;
bool online = false;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
unsigned long lastMsg = 0;
const unsigned long period = 30000;  //the value is a number of milliseconds
bool counterFlag = false;

// pressure sensor
int analogPin = 36;
int raw = 0;
int Vin = 3.3;
float Vout = 0;
float R1 = 100;
float R2 = 0;
float buffer = 0;
float valore = 0;
unsigned int long pressureTimer;

bool pressureAlarm = false;

bool masterOn = false;
bool minPressure = false;
bool maxPressure = false;
bool valveStatus = false;
unsigned long enableTime;

#define VALVE1_A 23
#define VALVE1_B 22
#define closedPin 27
#define openPin 26

// ---- High/Low pressure shutdown settings/state ----
const int RAW_LIMIT = 1280;
const unsigned long RAW_OVER_LIMIT_MS = 15000;

const int RAW_MIN_LIMIT = 300;
const unsigned long RAW_UNDER_LIMIT_MS = 60000;

unsigned long rawOverStartMs = 0;   // 0 = not currently over limit
unsigned long rawUnderStartMs = 0;  // 0 = not currently under limit

// Alert logs shown on root page
String pressureTripLog = "";
String lowPressureTripLog = "";

String nowString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    // fallback: uptime in seconds
    return "NTP not set (uptime " + String(millis() / 1000) + "s)";
  }
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}

void clearPressureAlertLogs() {
  pressureTripLog = "";
  lowPressureTripLog = "";

  // reset timers so checks start fresh
  rawOverStartMs = 0;
  rawUnderStartMs = 0;
}

// High pressure trip: raw > 2000 for >= 15 seconds -> shutdown, log.
// Non-latching: clears its timer after shutdown, does NOT restart pumps.
void checkHighPressureTrip() {
  if (raw > RAW_LIMIT) {
    if (rawOverStartMs == 0) {
      rawOverStartMs = millis();
      return;
    }

    if (millis() - rawOverStartMs >= RAW_OVER_LIMIT_MS) {
      offPumps();
      disablePumps();

      // Clear latch/timer immediately (re-arm) but do NOT turn pumps back on
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

// Low pressure trip: while masterOn==true, raw < 500 for >= 60 seconds -> shutdown, log.
// Non-latching: clears its timer after shutdown, does NOT restart pumps.
void checkLowPressureTripWhilePumpsOn() {
  // Only monitor when pumps are enabled/on
  if (masterOn == false) {
    rawUnderStartMs = 0; // reset when pumps are off
    return;
  }

  if (raw < RAW_MIN_LIMIT) {
    if (rawUnderStartMs == 0) {
      rawUnderStartMs = millis();
      return;
    }

    if (millis() - rawUnderStartMs >= RAW_UNDER_LIMIT_MS) {
      offPumps();
      disablePumps();

      // Clear latch/timer immediately (re-arm) but do NOT restart pumps
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

  pressureTimer = millis() + 1000;
  Serial.print("millis: ");
  Serial.println(millis());
  pinMode(analogPin, INPUT);

  // pin per valvola
  pinMode(closedPin, INPUT);
  pinMode(openPin, INPUT);

  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
  }

  //Turn off all relays
  for (int i = 0; i < numRelays; i++) {
    digitalWrite(relayPins[i], LOW);
  }

  WiFi.begin(ssid, password);
  WiFi.config(staticIP, gateway, subnet);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    // inserire reboot qua
    if (counterFlag == false) {
      startMillis = millis();  //initial start time
      counterFlag = true;
    } else {
      currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
      if (currentMillis - startMillis >= period)  //test whether the period has elapsed
      {
        Serial.println("RESTART: wifi connect timeout"); 
        ESP.restart();
      }
    }

  }
  digitalWrite(ledPin, HIGH);
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Time sync (NTP) for timestamped shutdown log
  const char* TZ_ITALIA = "CET-1CEST,M3.5.0/2,M10.5.0/3"; 
  const char* ntpServer = "pool.ntp.org";
  // Configurazione orario
  configTzTime(TZ_ITALIA, ntpServer);
  
  struct tm timeinfo;
  if(getLocalTime(&timeinfo)){
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  }
  //configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  server.on("/", handleRoot);

  server.on("/master/on", HTTP_GET, []() {
    clearPressureAlertLogs();
    enablePumps();
    delay(2000);
    onPumps();
    server.send(200, "text/html", "<font color=red size=5>master turned on</font>");
  });

  server.on("/single", HTTP_GET, []() {
    clearPressureAlertLogs();
    enablePumps();
    delay(2000);
    onPumpOne();
    server.send(200, "text/html", "<font color=red size=5>single turned on</font>");
  });

  server.on("/master/off", HTTP_GET, []() {
    offPumps();
    disablePumps();
    server.send(200, "text/html", "<font color=green size=5>master turned off</font>");
  });

  server.begin();
  Serial.println("HTTP server started");
  Serial.print("millis: ");
  Serial.println(millis());
  Serial.print("timer: ");
  Serial.println(pressureTimer);
  pinMode(VALVE1_A, OUTPUT);
  pinMode(VALVE1_B, OUTPUT);
  Serial.println("ok ready");
  /*
    closeValve();
    delay(3000);
    openValve();
    closeValve();
    delay(3000);
    openValve();
  */
}

void loop() {
  server.handleClient();
  //  WiFi connection check
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED every 2 second
    //delay(1000);
  }
  
  // se le pompe son spente facciamo check connessione con un ping
  if (masterOn == false ) {
    // pinghiamo ogni 10 secondi
    unsigned long now = millis();
    if (now - lastMsg > 10000) {
      lastMsg = now;
      // check if we are connected to main
      bool ret = Ping.ping("192.168.5.1");
      if (ret == false ) {
        online = false;

      } else if (ret == true) {
        online = true;
      }
    } // end timer

    // se non pinghiamo, controlliamo quante volte e poi tiriamo un reset
    if (online == false) {
      offlineCounter += 1;
    } else if (online == true ) {
      offlineCounter = 0;
    }
    if (offlineCounter >= 4 ) {
     // Serial.println("RESTART: ping"); 
    //  ESP.restart();
    }
  }

  // leggiamo pressione acqua ogni secondo
  if (pressureTimer <= millis()) {
    pressureTimer = millis() + 1000;

    raw = analogRead(analogPin);
    raw = analogRead(analogPin);
    raw = analogRead(analogPin);
    Serial.print("Raw: ");
    Serial.println(raw);
    valore = map(raw, 0, 2095, 0, 10);

    // Pressure safety checks
    checkHighPressureTrip();
    checkLowPressureTripWhilePumpsOn();
  }

}

void enablePumps() {
  digitalWrite(relayPins[0], HIGH);
  masterOn = true;
}

void disablePumps() {
  digitalWrite(relayPins[0], LOW);
  masterOn = false;
}

void onPumps() {
  if (masterOn == true ) {
    digitalWrite(relayPins[1], HIGH);
    delay(4000);
    digitalWrite(relayPins[2], HIGH);
  }
}

void onPumpOne() {
  if (masterOn == true ) {
    digitalWrite(relayPins[1], HIGH);

  }
}

void offPumps() {
  digitalWrite(relayPins[1], LOW);
  delay(500);
  digitalWrite(relayPins[2], LOW);

}

// spurgo aria
void closeValve() {
  digitalWrite(VALVE1_A, HIGH);
  digitalWrite(VALVE1_B, LOW);
  if (analogRead(closedPin) == LOW ) {
    Serial.println("valvola chiusa");
    digitalWrite(VALVE1_A, LOW);
    digitalWrite(VALVE1_B, LOW);
  }
}

void openValve() {
  digitalWrite(VALVE1_A, LOW);
  digitalWrite(VALVE1_B, HIGH);
  // if(analogRead(openPin) == LOW ){
  //    Serial.println("valvola aperta");
  //     digitalWrite( VALVE1_A, LOW );
  //  digitalWrite( VALVE1_B, LOW );
  // }
}

void handleRoot() {
  String roothtml;
  roothtml += "<!DOCTYPE HTML>";
  roothtml += "<html><head>";
  roothtml += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" /> ";
  roothtml += "<META HTTP-EQUIV=\"CACHE-CONTROL\" CONTENT=\"NO-CACHE\">";
  roothtml += "<meta http-equiv=\"Expires\" content=\"0\">";
  roothtml += "<title>MxIrrigation - Pump control</title></head><body>";
  roothtml += "<div align=center>Stato pompe:";
  if (masterOn == true) {
    roothtml += " <font color=red>ACCESE</font>";
  } else {
    roothtml += "<font color=green>SPENTE</font>";
  }
  roothtml += "<br><br>Pressione: raw ";
  roothtml += raw;
  roothtml += " | bar ";
  roothtml += valore;
  roothtml += "<br><br><a href=/master/on>Accendi entrambe</a>";
  roothtml += "<br><br><a href=/single>Accendi singola</a>";
  roothtml += "<br><br><a href=/master/off>SPEGNI</a>";

  // Alerts (cleared when pumps are started again)
  roothtml += pressureTripLog;
  roothtml += lowPressureTripLog;

  roothtml += "</div></body></html>";
  server.send(200, "text/html", roothtml);
}
