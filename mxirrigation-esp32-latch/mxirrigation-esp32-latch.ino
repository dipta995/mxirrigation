#include <WiFi.h>
#include <WebServer.h>
#include <ESPping.h>
#include "time.h"
#include "esp_sntp.h"

const char* FW_VERSION = "1000";

const char *ssid = "WAFNAMOTOPARK";  
const char *password = "motocross";

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)

int valveDelay = 400;

IPAddress staticIP(192, 168, 5, 42); // static IP address
IPAddress gateway(192, 168, 5, 1);    // network's gateway address
IPAddress subnet(255, 255, 255, 0);   // network's subnet mask

WebServer server(80);

#define LED_BUILTIN 22
//const int ledPin = LED_BUILTIN; 

bool online = false;
unsigned long lastMsg = 0;

#define VALVE1_A 0 //16
#define VALVE1_B 2 //17
#define VALVE2_A 18
#define VALVE2_B 19
#define VALVE3_A 21
#define VALVE3_B 22
#define VALVE4_A 23
#define VALVE4_B 25
#define VALVE5_A 26
#define VALVE5_B 27
#define VALVE6_A 16
#define VALVE6_B 17
#define VALVE7_A 32
#define VALVE7_B 33

int valveStatus[8];

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}

void setup() {

 //  pinMode(LED_BUILTIN, OUTPUT);
 // digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(115200);
  while (!Serial) delay(1);
  Serial.println("\n            Valve Latch Admin by MxSolutions.it");
  Serial.println("                  www.mxsolutions.it");
  Serial.println("                 All Rights Reserved");
    Serial.print("                 Versione firmware: ");
  Serial.println(FW_VERSION);
  Serial.println("------------------------------------------------------------");

     // set notification call-back function
//  sntp_set_time_sync_notification_cb( timeavailable );

 // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  
delay(2000);
  pinMode( VALVE1_A, OUTPUT );
  pinMode( VALVE1_B, OUTPUT );
   digitalWrite( VALVE1_A, HIGH );
  digitalWrite( VALVE1_B, LOW );
  delay(valveDelay);
  digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, LOW );

 pinMode( VALVE2_A, OUTPUT );
  pinMode( VALVE2_B, OUTPUT );
   digitalWrite( VALVE2_A, HIGH );
  digitalWrite( VALVE2_B, LOW );
  delay(valveDelay);
  digitalWrite( VALVE2_A, LOW );
  digitalWrite( VALVE2_B, LOW );
   pinMode( VALVE3_A, OUTPUT );
  pinMode( VALVE3_B, OUTPUT );
   digitalWrite( VALVE3_A, HIGH );
  digitalWrite( VALVE3_B, LOW );
  delay(valveDelay);
  digitalWrite( VALVE3_A, LOW );
  digitalWrite( VALVE3_B, LOW );
   pinMode( VALVE4_A, OUTPUT );
  pinMode( VALVE4_B, OUTPUT );
   digitalWrite( VALVE4_A, HIGH );
  digitalWrite( VALVE4_B, LOW );
  delay(valveDelay);
  digitalWrite( VALVE4_A, LOW );
  digitalWrite( VALVE4_B, LOW );
   pinMode( VALVE5_A, OUTPUT );
  pinMode( VALVE5_B, OUTPUT );
   digitalWrite( VALVE5_A, HIGH );
  digitalWrite( VALVE5_B, LOW );
  delay(valveDelay);
  digitalWrite( VALVE5_A, LOW );
  digitalWrite( VALVE5_B, LOW );
   pinMode( VALVE6_A, OUTPUT );
  pinMode( VALVE6_B, OUTPUT );
   digitalWrite( VALVE6_A, HIGH );
  digitalWrite( VALVE6_B, LOW );
  delay(valveDelay);
  digitalWrite( VALVE6_A, LOW );
  digitalWrite( VALVE6_B, LOW );
     pinMode( VALVE7_A, OUTPUT );
  pinMode( VALVE7_B, OUTPUT );
   digitalWrite( VALVE7_A, HIGH );
  digitalWrite( VALVE7_B, LOW );
  delay(valveDelay);
  digitalWrite( VALVE7_A, LOW );
  digitalWrite( VALVE7_B, LOW );
  
 for(int x=1;x<8;x++){
  valveStatus[x]=0;
 }
  
   WiFi.begin(ssid, password);
  WiFi.config(staticIP, gateway, subnet); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
//digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);

 
for (int i = 0; i < 8; i++) {
  int valveNum = i+1;
  
  server.on("/"+String(valveNum) + "/on", HTTP_GET, [i,valveNum]() {

    if(valveNum == 1){
   Serial.println("apro");
   digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, HIGH );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, LOW );
  
    }else    if(valveNum == 2){
   Serial.println("apro");
   digitalWrite( VALVE2_A, LOW );
  digitalWrite( VALVE2_B, HIGH );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE2_A, LOW );
  digitalWrite( VALVE2_B, LOW );
    }else    if(valveNum == 3){
   Serial.println("apro");
   digitalWrite( VALVE3_A, LOW );
  digitalWrite( VALVE3_B, HIGH );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE3_A, LOW );
  digitalWrite( VALVE3_B, LOW );
    }else    if(valveNum == 4){
   Serial.println("apro");
   digitalWrite( VALVE4_A, LOW );
  digitalWrite( VALVE4_B, HIGH );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE4_A, LOW );
  digitalWrite( VALVE4_B, LOW );
     }else    if(valveNum == 5){
   Serial.println("apro");
   digitalWrite( VALVE5_A, LOW );
  digitalWrite( VALVE5_B, HIGH );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE5_A, LOW );
  digitalWrite( VALVE5_B, LOW );
    }else    if(valveNum == 6){
   Serial.println("apro");
   digitalWrite( VALVE6_A, LOW );
  digitalWrite( VALVE6_B, HIGH );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE6_A, LOW );
  digitalWrite( VALVE6_B, LOW );
    }else    if(valveNum == 7){
   Serial.println("apro");
   digitalWrite( VALVE7_A, LOW );
  digitalWrite( VALVE7_B, HIGH );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE7_A, LOW );
  digitalWrite( VALVE7_B, LOW );
    }
    
    valveStatus[valveNum]=1;
    server.send(200, "text/plain", "Valvola " + String(valveNum) + " ON");
  });

  server.on("/"+String(valveNum) + "/off", HTTP_GET, [i,valveNum]() {

    if(valveNum ==1){
    Serial.println("fermo");
   digitalWrite( VALVE1_A, HIGH );
  digitalWrite( VALVE1_B, LOW );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE1_A, LOW );
  digitalWrite( VALVE1_B, LOW );
    }else if(valveNum ==2){
    Serial.println("fermo");
   digitalWrite( VALVE2_A, HIGH );
  digitalWrite( VALVE2_B, LOW );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE2_A, LOW );
  digitalWrite( VALVE2_B, LOW );
    }else if(valveNum ==3){
    Serial.println("fermo");
   digitalWrite( VALVE3_A, HIGH );
  digitalWrite( VALVE3_B, LOW );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE3_A, LOW );
  digitalWrite( VALVE3_B, LOW );
    }else if(valveNum ==4){
    Serial.println("fermo");
   digitalWrite( VALVE4_A, HIGH );
  digitalWrite( VALVE4_B, LOW );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE4_A, LOW );
  digitalWrite( VALVE4_B, LOW );
  }else if(valveNum ==5){
    Serial.println("fermo");
   digitalWrite( VALVE5_A, HIGH );
  digitalWrite( VALVE5_B, LOW );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE5_A, LOW );
  digitalWrite( VALVE5_B, LOW );
    }else if(valveNum ==6){
    Serial.println("fermo");
   digitalWrite( VALVE6_A, HIGH );
  digitalWrite( VALVE6_B, LOW );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE6_A, LOW );
  digitalWrite( VALVE6_B, LOW );
    
    }else if(valveNum ==7){
    Serial.println("fermo");
   digitalWrite( VALVE7_A, HIGH );
  digitalWrite( VALVE7_B, LOW );
  delay(valveDelay);
   Serial.println("chiudo");
   digitalWrite( VALVE7_A, LOW );
  digitalWrite( VALVE7_B, LOW );
    
    }

    
    valveStatus[valveNum]=0;
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
 //  digitalWrite(LED_BUILTIN, LOW); 
   Serial.println("WIFI ERROR");
  }

// aggiorniamo ogni 10 secondi
  unsigned long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;
 //   Serial.println("pingo ogni 10 secondi");
 // check if we are connected to main
  bool ret = Ping.ping("192.168.5.40");
  if(ret == false ){
   online = false;
    
  }else if(ret == true){
   online = true;
  }
      } // end timer
}





void handleRoot(){
  String roothtml;
/*
      time_t rawtime;
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
   return;
  }
  char timeStringBuff[15]; 
 
    strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo); 
 
  //Optional: Construct String object 
 String asString(timeStringBuff);
*/

roothtml +="<!DOCTYPE HTML>";
roothtml +="<html><head>";
roothtml +="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" /> ";
roothtml += "<META HTTP-EQUIV=\"CACHE-CONTROL\" CONTENT=\"NO-CACHE\">";
roothtml += "<meta http-equiv=\"Expires\" content=\"0\">";
roothtml +="<title>MxIrrigation - Valve Latch control</title></head><body>";
roothtml +="<div align=center>Stato valvole:<br>";
for(int y=1;y<8;y++){
  roothtml +="valvola n.";
  roothtml += y;
  roothtml +=" ";
  if(valveStatus[y]==1){
     roothtml +=" <font color=red>ON</font> | <a href=/";
     roothtml += y;
     roothtml += "/off >TURN OFF</a><br>";
}else{
  roothtml +="<font color=green>OFF</font> | <a href=/";
     roothtml += y;
     roothtml += "/on >TURN ON</a><br>";

  }
}

roothtml +="<br><br>Connection Status: ";
if(online == true){
roothtml +="<font color=green>CONNECTED</font>";
}else{
  roothtml +="<font color=red>ERROR</font>";
}
//roothtml +="<br><br><a href=/single>Accendi singola</a>";
//roothtml +="<br><br><a href=/master/off>SPEGNI</a><br><br>";
roothtml +="<br>Data: ";
//roothtml +=timeStringBuff;
roothtml +="</div></body></html>";
  server.send(200, "text/html", roothtml);
}
