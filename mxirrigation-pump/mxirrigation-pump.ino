#include <WiFi.h>
#include <WebServer.h>

const char* FW_VERSION = "1000";

const char *ssid = "WAFNAMOTOPARK";  
const char *password = "motocross";


IPAddress staticIP(192, 168, 5, 43); // static IP address
IPAddress gateway(192, 168, 5, 1);    // network's gateway address
IPAddress subnet(255, 255, 255, 0);   // network's subnet mask

WebServer server(80);

const int relayPins[] = {21, 19, 18, 5};
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int ledPin = 25; 

// pressure sensor
int analogPin = 36;
int raw = 0;
int Vin = 3.3;
float Vout = 0;
float R1 = 100;
float R2 = 0;
float buffer = 0;
int valore = 0;
unsigned int long pressureTimer;

bool masterOn = false;
bool minPressure = false;
bool maxPressure = false;
bool valveStatus = false;
unsigned long enableTime;

#define VALVE1_A 23
#define VALVE1_B 22
#define closedPin 27
#define openPin 26

void setup() {
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

  server.on("/", handleRoot);

 
  server.on("/master/on", HTTP_GET, []() {
    enablePumps();
    delay(2000);
    onPumps();
    server.send(200, "text/html", "<font color=green size=5>master turned on</font>");
  });

  server.on("/single", HTTP_GET, []() {
    enablePumps();
    delay(2000);
    onPumpOne();
    server.send(200, "text/html", "<font color=green size=5>single turned on</font>");
  });

  server.on("/master/off", HTTP_GET, []() {
    offPumps();
    disablePumps();
    server.send(200, "text/plain", "<font color=green size=5>master turned off</font>");
  });



  server.begin();
  Serial.println("HTTP server started");
           Serial.print("millis: ");
    Serial.println(millis());
               Serial.print("timer: ");
    Serial.println(pressureTimer);
     pinMode( VALVE1_A, OUTPUT );
  pinMode( VALVE1_B, OUTPUT );
closeValve();
delay(3000);
openValve();
closeValve();
delay(3000);
openValve();
}

void loop() {
  server.handleClient();
   //  WiFi connection check
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED every 2 second
    delay(1000);
  }


  // leggiamo pressione acqua ogni secondo
  if(pressureTimer <= millis()){
    pressureTimer = millis() + 1000;
  
      raw = analogRead(analogPin);
       Serial.print("Raw: ");
    Serial.println(raw);
    valore = raw;
  }
  
}

void enablePumps(){
     digitalWrite(relayPins[0], HIGH);
     masterOn = true;
}

void disablePumps(){
  digitalWrite(relayPins[0], LOW);
  masterOn = false;
}

void onPumps(){
  if(masterOn == true ){
     digitalWrite(relayPins[1], HIGH);
    delay(4000);
    digitalWrite(relayPins[2], HIGH);
  }
}

void onPumpOne(){
  if(masterOn == true ){
     digitalWrite(relayPins[1], HIGH);

  }
}

void offPumps(){
 digitalWrite(relayPins[1], LOW);
    delay(500);
    digitalWrite(relayPins[2], LOW);

}

// spurgo aria
void closeValve(){
   digitalWrite( VALVE1_A, HIGH );
  digitalWrite( VALVE1_B, LOW );
 if(analogRead(closedPin) == LOW ){
    Serial.println("valvola chiusa");
     digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, LOW );
 }
}

void openValve(){
   digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, HIGH );
// if(analogRead(openPin) == LOW ){
//    Serial.println("valvola aperta");    
//     digitalWrite( VALVE1_A, LOW );
//  digitalWrite( VALVE1_B, LOW );
// }
   

}



void handleRoot(){
  String roothtml;
roothtml +="<!DOCTYPE HTML>";
roothtml +="<html><head>";
roothtml +="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" /> ";
roothtml += "<META HTTP-EQUIV=\"CACHE-CONTROL\" CONTENT=\"NO-CACHE\">";
roothtml += "<meta http-equiv=\"Expires\" content=\"0\">";
roothtml +="<title>MxIrrigation - Pump control</title></head><body>";
roothtml +="<div align=center>Stato pompe:";
if(masterOn == true){
  roothtml +=" <font color=red>ACCESE</font>";
}else{
  roothtml +="<font color=green>SPENTE</font>";
}
roothtml +="<br><br>Pressione: raw ";
roothtml += raw;
roothtml +=" | bar ";
roothtml += valore;
roothtml +="<br><br><a href=/master/on>Accendi entrambe</a>";
roothtml +="<br><br><a href=/single>Accendi singola</a>";
roothtml +="<br><br><a href=/master/off>SPEGNI</a>";
roothtml +="</div></body></html>";
  server.send(200, "text/html", roothtml);
}
