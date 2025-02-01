//Bicopter Storage System
//Written by Matthew Santos

/* Author Notes
- this system includes these submodules
  - Non Volitile Storage (NVS)
  - SDCard Storage (Removable Flash)
- the NVS module handles all configurable parameters
  - parameters stored in NVS are loaded into ram upon calling Init()
  - other systems access or modify these parameters through public varriables
  - other systems may save or reload from NVS using associated public functions
  - to maintain organization, varriables outside of this class should be system specific (ie: internal to another class)
- the SDCard module permits reading and writing
  - other systems may use it for logging information or as temporary storage
*/

#ifndef STORAGE_SYSTEM_H
#define STORAGE_SYSTEM_H

#include <Preferences.h>
#include "FS.h"
#include "SD_MMC.h"

class Storage_System {

  //Non Volitile Storage (NVS)
  //--------------------------
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
    //NVS Functions
    bool NVS_readAll();
    bool NVS_writeAll();
    template <typename T_Storage>
    bool NVS_read (const char *name,const char *key,T_Storage &data);  //Load Namespace from NVS
    template <typename T_Storage>
    bool NVS_write(const char *name,const char *key,T_Storage &data);  //Save Namespace to NVS
    
  //SDCard (Flash) Storage
  //--------------------------
  public:
    //SD Card Varriables
    uint8_t SDCard_type;
    uint64_t SDCard_size;
    //SDCard Functions
    bool SDCARD_Init();
    bool SDCARD_createDir(const char *path);
    bool SDCARD_removeDir(const char *path);
    bool SDCARD_writeFile(const char *path, const char *data, const char* access_type);
    bool SDCARD_deleteFile(const char *path);

  //Storage System Class 
  //--------------------------
  public:
    void Init();                    //Storage Class Initializer
};
#endif