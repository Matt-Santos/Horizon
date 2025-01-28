// Bicopter Firmware
// Written by Matthew Santos

//Libraries
#include "storage_system.h" //needs to be written still
#include "sensor_system.h"  //fix camera
#include "network_system.h" //done
#include "comms_system.h"   //functional but room to improve
#include "flight_system.h"  //needs work

/* Todo
-write the storage system class to access persistant namespaces
  -move most defines into namespaces (be sure to check namespace limits)
-move the inferred sensor information into the flight class instead?
-Configure Camera to connect to GCS in Sensors
-write the flight class
  -Fix PWM Negative Value issue in motor output calculation
  -Reduce Upper and lower limits of PWM Value to ensure ESC can read

-Add arming Protical which starts by giving ESC minimal throttle (to calibrate) in Comms
-Implement Flgiht Control PID system
-add Additional MAVLINK Features in comms

-Note ESP32 Has two processors and could benifit from multithreading (flight calculations)
*/

//Startup
void setup() {
  Serial.begin(9600,SERIAL_8N1);
  storage.Init();
  sensor.Init();
  network.Init();
  comms.Init();
  flight.Init();
}
//Main Program Loop
void loop() {
  sensor.Update();
  network.Update();
  comms.Update();
  flight.Update();
}
