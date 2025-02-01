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

};

#endif