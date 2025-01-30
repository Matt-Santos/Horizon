//Bicopter Network System
//Written by Matthew Santos

#include "network_system.h"

//Wifi Settings (ESP32)
#define WIFI_Connections  2               //Max active connections
#define WIFI_LOCAL_IP     192,168,1,100
#define WIFI_GATEWAY      192,168,1,10
#define WIFI_SUBNET       255,255,255,0

Network_System network;

//Public Functions
//---------------------
void Network_System::Update(){
  WIFI_Update();
}
void Network_System::Init(){
  if(!WIFI_Init()) Serial.println("Error Initializing Network->WIFI");
}
//Private Functions
//---------------------
bool Network_System::WIFI_Init(){
  bool success = true;
  IPAddress local_IP(WIFI_LOCAL_IP);
  IPAddress gateway(WIFI_GATEWAY);
  IPAddress subnet(WIFI_SUBNET);
  success &= WiFi.softAPConfig(local_IP, gateway, subnet);
  success &= WiFi.softAPsetHostname(storage.Network.WIFI_Hostname);
  success &= WiFi.softAP(
    storage.Network.WIFI_SSID,
    storage.Network.WIFI_PASS,
    storage.Network.WIFI_channel,
    storage.Network.WIFI_Hidden,
    WIFI_Connections);
  return success;
}
void Network_System::WIFI_Update(){
  static int last_time;
  if (millis() - last_time < storage.Network.Wifi_Period) return;
  connections = WiFi.softAPgetStationNum();
  last_time = millis();
}
