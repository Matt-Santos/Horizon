//Bicopter Storage System
//Written by Matthew Santos

#include "storage_system.h"

//NVS Settings
#define NAMESPACE "param"

//SDCARD Settings
#define SDCard_CLK 14
#define SDCard_CMD 15
#define SDCard_DAT 2

Preferences prefs;

//Non Volitile Storage (NVS)
//--------------------------
bool Storage_System::NVS_readAll() {
  bool success = true;
  success &= NVS_read(NAMESPACE, "Param", param);
  return success;
}
bool Storage_System::NVS_writeAll() {
  bool success = true;
  success &= NVS_write(NAMESPACE, "Param", param);
  return success;
}
template<typename T_Storage>
bool Storage_System::NVS_read(const char *name, const char *key, T_Storage &data) {
  bool success = true;
  success &= prefs.begin(name, true, "nvs");
  if (success && prefs.getBytesLength(key) == sizeof(T_Storage)) {
    uint8_t buffer[sizeof(T_Storage)];
    memcpy(&data, buffer, sizeof(T_Storage));
  }
  prefs.end();
  return success;
}
template<typename T_Storage>
bool Storage_System::NVS_write(const char *name, const char *key, T_Storage &data) {
  bool success = true;
  success &= prefs.begin(name, false, "nvs");
  if (success) {
    uint8_t buffer[sizeof(T_Storage)];
    memcpy(buffer, &data, sizeof(T_Storage));
    success &= (prefs.putBytes(key, buffer, sizeof(T_Storage)) > 0);
  }
  prefs.end();
  return success;
}

//SDCard (Flash) Storage
//--------------------------
bool Storage_System::SDCARD_Init() {
  bool success = true;
  success &= SD_MMC.setPins(SDCard_CLK, SDCard_CMD, SDCard_DAT);
  success &= SD_MMC.begin("/sdcard", true);
  //Get Card Info
  SDCard_type = SD_MMC.cardType();
  SDCard_size = SD_MMC.cardSize() / (1024 * 1024);
  return success;
}
bool Storage_System::SDCARD_createDir(const char *path) {
  return SD_MMC.mkdir(path);
}
bool Storage_System::SDCARD_removeDir(const char *path) {
  return SD_MMC.rmdir(path);
}
bool Storage_System::SDCARD_writeFile(const char *path, const char *data, const char *access_type) {
  File file = SD_MMC.open(path, access_type);  //access_type is either 'w','a' write or append
  if (!file) {
    bool success = file.print(data);
    file.close();
    return success;
  }
  return false;
}
bool Storage_System::SDCARD_deleteFile(const char *path) {
  return SD_MMC.remove(path);
}

//Storage System Class 
//--------------------------
void Storage_System::Init() {
  if (!NVS_readAll()) Serial.println("Error Initializing Storage->NVS");
  if (!SDCARD_Init()) Serial.println("Error Initializing Storage->SDCARD");
}
