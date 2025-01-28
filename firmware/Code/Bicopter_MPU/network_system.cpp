//Bicopter Network System
//Written by Matthew Santos

#include "network_system.h"

Network_System network;

//Public Functions
//---------------------
void Network_System::Update(){
  static int last_time;
  if (millis() - last_time < Wifi_Period) return;
  connections = WiFi.softAPgetStationNum();
  last_time = millis();
}
void Network_System::Init(){
  bool success = true;
  IPAddress local_IP(WIFI_LOCAL_IP);
  IPAddress gateway(WIFI_GATEWAY);
  IPAddress subnet(WIFI_SUBNET);
  success &= WiFi.softAPConfig(local_IP, gateway, subnet);
  success &= WiFi.softAPsetHostname(WIFI_Hostname);
  success &= WiFi.softAP(WIFI_SSID,WIFI_PASS,WIFI_channel,WIFI_Hidden,WIFI_Connections);
  if (!success){Serial.println("Error Initializing Network->Wifi");}
}
