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
#include <MAVLink_common.h>
#include "FS.h"
#include "SD_MMC.h"

typedef struct _NVS_Param {
  char param_id[16];            //Parameter ID (Text Label)
  mavlink_param_union_t value;  //Parameter Value and Type
} NVS_Param_t;

class Storage_System {

  //Non Volitile Storage (NVS)
  //--------------------------
  public:
    NVS_Param_t param[4] = {
      {"TestparmA",{.param_float = 1.44,.type = MAVLINK_TYPE_FLOAT}},  //0
      {"TestparmB",{.param_float = 2.44,.type = MAVLINK_TYPE_FLOAT}},  //1
      {"TestparmC",{.param_float = 3.44,.type = MAVLINK_TYPE_FLOAT}},  //2
      {"TestparmD",{.param_float = 4.44,.type = MAVLINK_TYPE_FLOAT}},  //3
    };
    bool NVS_readAll();
    bool NVS_writeAll();
    template <typename T_Storage>
    bool NVS_read (const char *name,const char *key,T_Storage &data);  //Load Namespace from NVS
    template <typename T_Storage>
    bool NVS_write(const char *name,const char *key,T_Storage &data);  //Save Namespace to NVS
    
  //SDCard (Flash) Storage
  //--------------------------
  public:
    uint8_t SDCard_type;
    uint64_t SDCard_size;
    bool SDCARD_createDir(const char *path);
    bool SDCARD_removeDir(const char *path);
    bool SDCARD_writeFile(const char *path, const char *data, const char* access_type);
    bool SDCARD_deleteFile(const char *path);
  private:
    bool SDCARD_Init();

  //Storage System Class 
  //--------------------------
  public:
    void Init();  //Storage Class Initializer
};
#endif