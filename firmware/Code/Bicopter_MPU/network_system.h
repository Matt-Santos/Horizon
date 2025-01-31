//Bicopter Network System
//Written by Matthew Santos

/* Author Notes
- this system includes these submodules
  - Wifi Access Point (WIFI)
- the WIFI module configures the wifi network
  - other systems may use it for communication
*/

#ifndef NETWORK_SYSTEM_H
#define NETWORK_SYSTEM_H

#include <WiFi.h>
#include <WiFiAP.h>
#include "storage_system.h"
#include "sensor_system.h"

class Network_System {

  //Wifi Access Point (WIFI)
  //---------------------
  public:
    //WIFI Data
    uint8_t connections = 0;  //Tracks # of active connects
    //WIFI Functions
  private:
    bool WIFI_Init();
    void WIFI_Update();

  //Network System Class 
  //--------------------------
  public:
    void Init();
    void Update();
    static Network_System* get();  //Access Singleton Network Class
  private:
    Network_System(){};
    static Network_System* instance;

};

#endif