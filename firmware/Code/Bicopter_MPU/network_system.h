//Bicopter Network System
//Written by Matthew Santos
#ifndef NETWORK_SYSTEM_H
#define NETWORK_SYSTEM_H

#include <WiFi.h>
#include <WiFiAP.h>

//Wifi Settings (ESP32)
#define WIFI_SSID         "Bicopter"
#define WIFI_PASS         "Bicopter"
#define WIFI_Hostname     "Bicopter"
#define WIFI_channel      1
#define WIFI_Hidden       false
#define WIFI_Connections  2               //Max active connections
#define WIFI_LOCAL_IP     192,168,1,100
#define WIFI_GATEWAY      192,168,1,10
#define WIFI_SUBNET       255,255,255,0
#define Wifi_Period       2000            //[ms] Wifi Update Period

class Network_System {
  public:
    uint8_t connections = 0;  //Tracks # of active connects

    //Public Functions
    //---------------------
    void Update();
    void Init();
};

extern Network_System network;

#endif