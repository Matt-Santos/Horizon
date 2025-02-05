// Bicopter Firmware
// Written by Matthew Santos

//Libraries
#include "storage_system.h"
#include "sensor_system.h"
#include "network_system.h"
#include "comms_system.h"
#include "flight_system.h"

/* Todo

-storage class
  -verify NVS and SDCard functions

-sensor class
  -Finish Gyro Filter in Sensor System and improve sensor stability and orientation
  -Add GPS system
  -Add pressure sensor
  
-network class
  -configure the camera webserver and setup vlc to perform local GCS conversion/encoding of video feed to qGCS

-comms class
  -arming ability (starts by giving ESC minimal throttle (to calibrate) in Comms)
  -change control modes
  -complete FTP Protocal and Metadata delivery

-flight class
  -add esc calibration procedure
  -Fix PWM Negative Value issue in motor output calculation
  -Reduce Upper and lower limits of PWM Value to ensure ESC can read
  -Implement stabilization PID system with storaged values

-Note ESP32 Has two processors and could benifit from multithreading (flight calculations)
*/

//System Modules (Global Objects)
Storage_System *storage = new Storage_System();
Sensor_System *sensor = new Sensor_System();
Network_System *network = new Network_System();
Comms_System *comms = new Comms_System();
Flight_System *flight = new Flight_System();

//Startup
void setup() {
  flight->Fast_Init();
  Serial.begin(9600,SERIAL_8N1);
  storage->Init();
  sensor->Init();
  network->Init();
  comms->Init();
}

//Main Program Loop
void loop() {
  sensor->Update();
  network->Update();
  comms->Update();
  flight->Update();
}
