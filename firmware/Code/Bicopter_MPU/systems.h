//Bicopter System
//Written by Matthew Santos

/* Author Notes
- 
*/

#include "storage_system.h"
#include "sensor_system.h"
#include "network_system.h"
#include "comms_system.h"
#include "flight_system.h"

class Systems {
  public:
    struct Systems {
      Storage_System storage;
      Sensor_System   sensor;
      Network_System network;
      Comms_System     comms;
      Flight_System   flight;
    } systems;
    void Init(){
      storage.Init();
    };
}
