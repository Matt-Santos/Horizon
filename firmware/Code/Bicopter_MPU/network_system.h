//Bicopter Network System
//Written by Matthew Santos
#ifndef NETWORK_SYSTEM_H
#define NETWORK_SYSTEM_H

#include <WiFi.h>
#include <WiFiAP.h>
#include "storage_system.h"
#include "sensor_system.h"

class Network_System {
  public:
    uint8_t connections = 0;  //Tracks # of active connects
    //Public Functions
    //---------------------
    void Update();
    void Init();
  private:
    //Private Functions
    //---------------------
    bool WIFI_Init();
    void WIFI_Update();
};

extern Network_System network;

#endif