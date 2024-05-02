#include <WiFi.h>
#include <WebServer.h>

const char *ssid = "WAFNAMOTOPARK";  
const char *password = "motocross";

/*
IPAddress staticIP(192, 168, 5, 41); // static IP address
IPAddress gateway(192, 168, 5, 1);    // network's gateway address
IPAddress subnet(255, 255, 255, 0);   // network's subnet mask
*/
WebServer server(80);

const int relayPins[] = {21, 19, 18, 5};
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int ledPin = 25; 

void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  for (int i=0; i< numRelays ;i++) {
  pinMode(relayPins[i], OUTPUT);
  }

   //Turn off all relays
    for (int i=0; i< numRelays ;i++) {
      digitalWrite(relayPins[i], LOW);

  }
    digitalWrite(ledPin, HIGH);
  
   WiFi.begin(ssid, password);
//  WiFi.config(staticIP, gateway, subnet); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/plain", "Hello from ESP32!");
  });

 
  server.on("/master/on", HTTP_GET, []() {
    enablePumps();
    delay(2000);
    onPumps();
    server.send(200, "text/plain", "master turned on");
  });

  server.on("/master/off", HTTP_GET, []() {
    digitalWrite(relayPins[1], LOW);
    delay(500);
    digitalWrite(relayPins[2], LOW);
    server.send(200, "text/plain", "master turned off");
  });



  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
   //  WiFi connection check
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED every 2 second
    delay(1000);
  }
}

void enablePumps(){
     digitalWrite(relayPins[0], HIGH);
}

void disablePumps(){
  digitalWrite(relayPins[0], LOW);
}

void onPumps(){
     digitalWrite(relayPins[1], HIGH);
    delay(4000);
    digitalWrite(relayPins[2], HIGH);
}
