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
  -verify NVS
  -add additional SDCard functions and test

-sensor class
  -Perfect IMU Offset Calibration, orientation/frame directionality and Filtering
  -Increase GPS SampleRate and optimize messages
  -Use pressure Sensor for altitude complementary filter
  -Calibrate the  Battery Sensor (and send to comms)
  -improve camera system? (or just leave it to the network stack)
  
-network class
  -configure the camera webserver and setup vlc to perform local GCS conversion/encoding of video feed to qGCS

-comms class
  -arming ability (starts by giving ESC minimal throttle (to calibrate) in Comms)
  -change control modes
  -complete FTP Protocal and Metadata delivery

-flight class
  -Implement Pitch Motor Calibration
  -Perfect ESC Motor Calibration
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
  Serial.begin(9600,SERIAL_8N1);
  while (!Serial){}
  Serial.println("Starting Up");
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

  
  // //Check Loop delay
  // static unsigned long last;
  // Serial.printf("Loop Delay = %d [us]\n",micros()-last);
  // last = micros();
}
