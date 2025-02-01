// Bicopter Firmware
// Written by Matthew Santos

//Libraries
#include "storage_system.h"
#include "sensor_system.h"
#include "network_system.h"
#include "comms_system.h"
#include "flight_system.h"

/* Todo
-configure the camera webserver with controls into network subsystem
-setup vlc to perform local GCS conversion/encoding of video feed to qGCS
-add complementary filter to the sensor system and improve sensor stability
-write the flight class
  -Fix PWM Negative Value issue in motor output calculation
  -Reduce Upper and lower limits of PWM Value to ensure ESC can read
  -Implement Flgiht Control PID system with storaged values
-add Additional MAVLINK Features in comms
  -ability to read/write params in storage NVS
  -arming ability (starts by giving ESC minimal throttle (to calibrate) in Comms)
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
  Serial.begin(9600,SERIAL_8N1);
  storage->Init();
  sensor->Init();
  network->Init();
  comms->Init();
  flight->Init();
}

//Main Program Loop
void loop() {
  sensor->Update();
  network->Update();
  comms->Update();
  flight->Update();
}
