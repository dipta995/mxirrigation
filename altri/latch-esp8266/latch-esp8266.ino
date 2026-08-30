#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

bool debug = false;

ESP8266WebServer server(80);

//const char *ssid = "WAFNAMOTOPARK";  
//const char *password = "motocross";

/*
IPAddress staticIP(192, 168, 5, 42); // static IP address
IPAddress gateway(192, 168, 5, 1);    // network's gateway address
IPAddress subnet(255, 255, 255, 0);   // network's subnet mask
*/

#define VALVE1_A D0
#define VALVE1_B D1
#define VALVE2_A D2
#define VALVE2_B D3
#define VALVE3_A D8
#define VALVE3_B D5
#define VALVE4_A D6
#define VALVE4_B D7





//int valveNum = 0;

void setup() {
  Serial.begin(115200);
 pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on
Serial.println("hello...");
delay(2000);
  pinMode( VALVE1_A, OUTPUT );
  pinMode( VALVE1_B, OUTPUT );
   digitalWrite( VALVE1_A, HIGH );
  digitalWrite( VALVE1_B, LOW );
  delay(350);
  digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, LOW );

 pinMode( VALVE2_A, OUTPUT );
  pinMode( VALVE2_B, OUTPUT );
   digitalWrite( VALVE2_A, HIGH );
  digitalWrite( VALVE2_B, LOW );
  delay(350);
  digitalWrite( VALVE2_A, LOW );
  digitalWrite( VALVE2_B, LOW );
   pinMode( VALVE3_A, OUTPUT );
  pinMode( VALVE3_B, OUTPUT );
   digitalWrite( VALVE3_A, HIGH );
  digitalWrite( VALVE3_B, LOW );
  delay(350);
  digitalWrite( VALVE3_A, LOW );
  digitalWrite( VALVE3_B, LOW );
   pinMode( VALVE4_A, OUTPUT );
  pinMode( VALVE4_B, OUTPUT );
   digitalWrite( VALVE4_A, HIGH );
  digitalWrite( VALVE4_B, LOW );
  delay(350);
  digitalWrite( VALVE4_A, LOW );
  digitalWrite( VALVE4_B, LOW );


 //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;

    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
    // wm.resetSettings();

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    wm.setSTAStaticIPConfig(IPAddress(192,168,5,41), IPAddress(192,168,5,1), IPAddress(255,255,255,0));
    wm.setShowStaticFields(true);
    wm.setShowDnsFields(true);
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect("ESP-VALVE-SERVER","password"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }

  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


 // server.on("/", handleRoot);
  
  

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/plain", "Hello from ESP8266!");
  });

for (int i = 0; i < 5; i++) {
  int valveNum = i+1;
  
  server.on("/"+String(valveNum) + "/on", HTTP_GET, [i,valveNum]() {

    if(valveNum == 1){
   Serial.println("apro");
   digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, HIGH );
  delay(350);
   Serial.println("chiudo");
   digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, LOW );
    }else    if(valveNum == 2){
   Serial.println("apro");
   digitalWrite( VALVE2_A, LOW );
  digitalWrite( VALVE2_B, HIGH );
  delay(350);
   Serial.println("chiudo");
   digitalWrite( VALVE2_A, LOW );
  digitalWrite( VALVE2_B, LOW );
    }else    if(valveNum == 3){
   Serial.println("apro");
   digitalWrite( VALVE3_A, LOW );
  digitalWrite( VALVE3_B, HIGH );
  delay(350);
   Serial.println("chiudo");
   digitalWrite( VALVE3_A, LOW );
  digitalWrite( VALVE3_B, LOW );
    }else    if(valveNum == 4){
   Serial.println("apro");
   digitalWrite( VALVE4_A, LOW );
  digitalWrite( VALVE4_B, HIGH );
  delay(350);
   Serial.println("chiudo");
   digitalWrite( VALVE4_A, LOW );
  digitalWrite( VALVE4_B, LOW );
    }
    server.send(200, "text/plain", "Valvola " + String(valveNum) + " ON");
  });

  server.on("/"+String(valveNum) + "/off", HTTP_GET, [i,valveNum]() {

    if(valveNum ==1){
    Serial.println("fermo");
   digitalWrite( VALVE1_A, HIGH );
  digitalWrite( VALVE1_B, LOW );
  delay(350);
   Serial.println("chiudo");
   digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, LOW );
    }else if(valveNum ==2){
    Serial.println("fermo");
   digitalWrite( VALVE2_A, HIGH );
  digitalWrite( VALVE2_B, LOW );
  delay(350);
   Serial.println("chiudo");
   digitalWrite( VALVE2_A, LOW );
  digitalWrite( VALVE2_B, LOW );
    }else if(valveNum ==3){
    Serial.println("fermo");
   digitalWrite( VALVE3_A, HIGH );
  digitalWrite( VALVE3_B, LOW );
  delay(350);
   Serial.println("chiudo");
   digitalWrite( VALVE3_A, LOW );
  digitalWrite( VALVE3_B, LOW );
    }else if(valveNum ==4){
    Serial.println("fermo");
   digitalWrite( VALVE4_A, HIGH );
  digitalWrite( VALVE4_B, LOW );
  delay(350);
   Serial.println("chiudo");
   digitalWrite( VALVE4_A, LOW );
  digitalWrite( VALVE4_B, LOW );
    
    }
    server.send(200, "text/plain", "valve " + String(valveNum) + " turned off");
  });
}


  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
   //  WiFi connection check
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED every 2 second
    delay(1000);
  }   
}
