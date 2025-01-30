//Bicopter Storage System
//Written by Matthew Santos
#ifndef STORAGE_SYSTEM_H
#define STORAGE_SYSTEM_H

#include <Preferences.h>

class Storage_System {
  public:
    //NVS Namespaces
    struct storage_storage {
    } Storage;
    struct storage_sensor{
      uint32_t I2C_Freq = 400000;     //[Hz] I2C Clock Frequency
      uint16_t IMU_Cal_Samples = 100; //Number of Samples used in Calibration
      int BAT_Period = 1;             //[ms] Time between Battery Measurements
      int CAM_Size = 0;               // 96x96
      //int CAM_Size = 1;               // 160x120
      //int CAM_Size = 2;               // 128x128
      //int CAM_Size = 3;               // 176x144
      //int CAM_Size = 4;               // 240x176
      //int CAM_Size = 5;               // 240x240
      //int CAM_Size = 6;               // 320x240
      //int CAM_Size = 7;               // 320x320
      //int CAM_Size = 8;               // 400x296
      //int CAM_Size = 9;               // 480x320
      //int CAM_Size = 10;              // 640x480
      //int CAM_Size = 11;              // 800x600
    } Sensor;
    struct storage_network {
      String WIFI_SSID = "Bicopter";          //SSID
      String WIFI_PASS = "Bicopter";          //Passwprd
      const char* WIFI_Hostname = "Bicopter"; //Hostname
      int WIFI_channel = 1;                   //Wifi Channel #
      bool WIFI_Hidden = false;               //Hidden Wifi
      int Wifi_Period = 2000;                 //[ms] Wifi Update Period
    } Network;
    struct storage_comms{
    } Comms;
    struct storage_flight{
    } Flight;
    Preferences prefs;
    //SDCard Varriables
    uint8_t SDCard_type = 0;
    uint64_t SDCard_size = 0;

    //Public Functions (returns 1 on success)
    //---------------------
    void Init();
    template <typename T_Storage>
    bool NVS_write(const char *name,const char *key,T_Storage &data);  //Save Namespace to NVS
    template <typename T_Storage>
    bool NVS_read (const char *name,const char *key,T_Storage &data);  //Load Namespace from NVS
    bool NVS_writeAll();
    bool SDCARD_createDir(const char *path);
    bool SDCARD_removeDir(const char *path);
    bool SDCARD_writeFile(const char *path, const char *data, const char* access_type);
    bool SDCARD_deleteFile(const char *path);
    static Storage_System& getInstance();  //Access Singleton Storage Class
  private:
    //Private Functions
    //---------------------
    bool NVS_Init();
    bool SDCARD_Init();
    //Singleton Implementation
    //---------------------
    static Storage_System* storage; //Private Class
    Storage_System(){}; //Blank Constructor
    
};
#endif