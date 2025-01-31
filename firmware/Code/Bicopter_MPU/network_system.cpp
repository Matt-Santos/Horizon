//Bicopter Network System
//Written by Matthew Santos

#include "network_system.h"

//Wifi Settings (ESP32)
#define WIFI_Connections  2               //Max active connections
#define WIFI_LOCAL_IP     192,168,1,100
#define WIFI_GATEWAY      192,168,1,10
#define WIFI_SUBNET       255,255,255,0

Network_System* Network_System::instance = nullptr;

//Wifi Access Point (WIFI)
//---------------------
bool Network_System::WIFI_Init(){
  bool success = true;
  IPAddress local_IP(WIFI_LOCAL_IP);
  IPAddress gateway(WIFI_GATEWAY);
  IPAddress subnet(WIFI_SUBNET);
  success &= WiFi.softAPConfig(local_IP, gateway, subnet);
  success &= WiFi.softAPsetHostname(Storage_System::get()->Network.WIFI_Hostname);
  success &= WiFi.softAP(
    Storage_System::get()->Network.WIFI_SSID,
    Storage_System::get()->Network.WIFI_PASS,
    Storage_System::get()->Network.WIFI_channel,
    Storage_System::get()->Network.WIFI_Hidden,
    WIFI_Connections);
  return success;
}
void Network_System::WIFI_Update(){
  static int last_time;
  if (millis() - last_time < Storage_System::get()->Network.Wifi_Period) return;
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
Network_System* Network_System::get() {
  if (instance == nullptr) instance = new Network_System;
    return instance;
}