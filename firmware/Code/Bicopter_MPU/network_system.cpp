//Bicopter Network System
//Written by Matthew Santos

#include "network_system.h"
#include "storage_system.h"
#include "sensor_system.h"

//Wifi Settings (ESP32)
#define WIFI_SSID         "Bicopter"      //SSID
#define WIFI_PASS         "Bicopter"      //Passwprd
#define WIFI_Hostname     "Bicopter"      //Hostname
#define WIFI_channel      1               //Wifi Channel #
#define WIFI_Hidden       false           //Hidden Wifi
#define Wifi_Period       2000            //[ms] Wifi Update Period
#define WIFI_Connections  2               //Max active connections
#define WIFI_LOCAL_IP     192,168,1,100
#define WIFI_GATEWAY      192,168,1,10
#define WIFI_SUBNET       255,255,255,0


extern Storage_System *storage;
//extern Sensor_System *sensor;

//Wifi Access Point (WIFI)
//---------------------
bool Network_System::WIFI_Init(){
  bool success = true;
  IPAddress local_IP(WIFI_LOCAL_IP);
  IPAddress gateway(WIFI_GATEWAY);
  IPAddress subnet(WIFI_SUBNET);
  success &= WiFi.softAPConfig(local_IP, gateway, subnet);
  success &= WiFi.softAPsetHostname(WIFI_Hostname);
  success &= WiFi.softAP(WIFI_SSID,WIFI_PASS,WIFI_channel,WIFI_Hidden,WIFI_Connections);
  return success;
}
void Network_System::WIFI_Update(){
  static int last_time;
  if (millis() - last_time < Wifi_Period) return;
  connections = WiFi.softAPgetStationNum();
  last_time = millis();
}

//Network System Class 
//--------------------------
void Network_System::Init(){
  if(!WIFI_Init()) Serial.println("Error Initializing Network->WIFI");
}
void Network_System::Update(){
  WIFI_Update();
}
