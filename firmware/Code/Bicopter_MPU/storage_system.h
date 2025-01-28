//Bicopter Storage System
//Written by Matthew Santos
#ifndef STORAGE_SYSTEM_H
#define STORAGE_SYSTEM_H

#include <Preferences.h>

class Storage_System {
  public:
    //Real Sensors
    bool debug = false;
    //Camera Resolution
    //arm_delay

    //Public Functions
    //---------------------
    void Init();
    void Set(key,ptr);
    void Get(key,ptr);
  private:
    //Private Functions
    //---------------------
    void 

};

extern Storage_System storage;

#endif