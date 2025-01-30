// Bicopter Firmware
// Written by Matthew Santos

//Libraries
#include "storage_system.h" //needs to be written still
#include "sensor_system.h"  //fix camera
#include "network_system.h" //done
#include "comms_system.h"   //functional but room to improve
#include "flight_system.h"  //needs work

//Global Objects
Storage_System& storage = Storage_System::getInstance();
Sensor_Class&    sensor = Sensor_Class::getInstance();
Network_System& network = Network_System::getInstance();
Comms_System&     comms = Comms_System::getInstance();
Flight_System&   flight = Flight_System::getInstance();

/* Todo
-move most defines into namespaces (be sure to check namespace limits)
-configure the camera webserver with controls
-setup vlc to perform local GCS conversion/encoding of video feed to qGCS
-add complementary filter to the sensor system
-write the flight class
  -Fix PWM Negative Value issue in motor output calculation
  -Reduce Upper and lower limits of PWM Value to ensure ESC can read
  -Implement Flgiht Control PID system with storaged values
-add Additional MAVLINK Features in comms
  -ability to read/write params in storage NVS
  -arming ability (starts by giving ESC minimal throttle (to calibrate) in Comms)

-Note ESP32 Has two processors and could benifit from multithreading (flight calculations)
*/

//Startup
void setup() {
  Serial.begin(9600,SERIAL_8N1);Serial.println("Start Init");
  storage.Init();Serial.println("Storage Init");
  sensor.Init();Serial.println("Sensor Init");
  //network.Init();Serial.println("Network Init");
  //comms.Init();Serial.println("Comms Init");
  //flight.Init();Serial.println("Flight Init");
  Serial.println("Init Complete");
}
//Main Program Loop
void loop() {
  sensor.Update();
  //network.Update();
  //comms.Update();
  //flight.Update();
}
