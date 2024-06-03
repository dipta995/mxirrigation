#include <WiFi.h>
#include <WebServer.h>

const char *ssid = "CABIANCATRACK";  
const char *password = "cabiancamotocross";


IPAddress staticIP(192, 168, 1, 9); // static IP address
IPAddress gateway(192, 168, 1, 1);    // network's gateway address
IPAddress subnet(255, 255, 255, 0);   // network's subnet mask

WebServer server(80);

const int relayPins[] = {21, 19, 18, 5};
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int ledPin = 25; 

// voltage sensor
int analogPin = 36;
int raw = 0;
int Vin = 3.3;
float Vout = 0;
float R1 = 100;
float R2 = 0;
float buffer = 0;
int valore = 0;
unsigned int long pressureTimer;

unsigned long int masterOffTimer;

bool masterOn = false;
bool keyOn = false;

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
  WiFi.config(staticIP, gateway, subnet); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/plain", "Hello from mxirrigation-diesel!");
  });

 
  server.on("/master/on", HTTP_GET, []() {
 //       digitalWrite(relayPins[0], HIGH);
 //    masterOn = true;
    server.send(200, "text/plain", "master turned on");
        Serial.println("master turned on");
  });

  server.on("/master/off", HTTP_GET, []() {
    digitalWrite(relayPins[0], HIGH);
     masterOn = false;
     masterOffTimer = millis() + 10000;
    server.send(200, "text/plain", "master turned off");
    Serial.println("master turned off");
  });



  server.begin();
  Serial.println("HTTP server started");

  
    masterOn = true;
}


void loop() {
  server.handleClient();
   //  WiFi connection check
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED every 2 second
    
  }
/*
  if(masterOn == false){
    if(masterOffTimer >= millis()){
      digitalWrite(relayPins[0], LOW);
    }
  }
  */
  /*
  // leggiamo voltaggio ogni secondo
  if(pressureTimer <= millis()){
    pressureTimer = millis() + 1000;
  
      raw = analogRead(analogPin);
       Serial.print("Raw: ");
    Serial.println(raw);
    valore = raw;
    if(raw >= 2500){
      // chiave inserita, faccio cose
      if(keyOn == false){
        keyOn = true;
      }
      
    }
  }
  */
}
