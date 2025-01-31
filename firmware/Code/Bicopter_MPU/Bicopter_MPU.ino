// Bicopter Firmware
// Written by Matthew Santos

//Libraries
#include "systems.h"

/* Todo
-move most defines into namespaces (be sure to check namespace limits)
-configure the camera webserver with controls
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

//Startup
void setup() {
  Serial.begin(9600,SERIAL_8N1);Serial.println("Start Init");
  storage->Init();Serial.println("Storage Init");
  Serial.printf("BatPeriod: %d \n",storage->Sensor.BAT_Period);
  sensor->Init();Serial.println("Sensor Init");
  //network.Init();Serial.println("Network Init");
  //comms.Init();Serial.println("Comms Init");
  //flight.Init();Serial.println("Flight Init");
  Serial.println("Init Complete");
}
//Main Program Loop
void loop() {
  //sensor.Update();
  //network.Update();
  //comms.Update();
  //flight.Update();
}
