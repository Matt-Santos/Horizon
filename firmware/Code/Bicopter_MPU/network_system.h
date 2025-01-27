//Bicopter Network
//Written by Matthew Santos

//#include <WiFi.h>
#include <WiFiAP.h>
//

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
    uint8_t connections = 0;

    //Public Functions
    //---------------------
    void Update(){
      static int last_time;
      if (millis() - last_time < Wifi_Period) return;
      connections = WiFi.softAPgetStationNum();
      last_time = millis();
    }
    void Init(){
      bool success = true;
      IPAddress local_IP(WIFI_LOCAL_IP);
      IPAddress gateway(WIFI_GATEWAY);
      IPAddress subnet(WIFI_SUBNET);
      success &= WiFi.softAPConfig(local_IP, gateway, subnet);
      success &= WiFi.softAPsetHostname(WIFI_Hostname);
      success &= WiFi.softAP(WIFI_SSID,WIFI_PASS,WIFI_channel,WIFI_Hidden,WIFI_Connections);
      if (!success){Serial.println("Error Initializing Network->Wifi");}
    }
};