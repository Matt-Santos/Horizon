// Bicopter Firmware
// Written by Matthew Santos

//Libraries
//#include "params.h"   //needs to be written still
#include "network_system.h"  //wifi library issues?
#include "comms_system.h"   //inprogress
#include "sensor_system.h"  //needs camera fixes
#include "flight_system.h"  //needs alot of work


//Global Varriables
Persist_System persist;
Network_System network;
Comms_System   comms;
Sensor_System  sensors;
Flight_System  flight;

/* Todo
-paramaterization and varriable orgainization
-Configure Camera to connect to GCS
-Fix PWM Negative Value issue in calculation
-Reduce Upper limit to the PWM Value to ensure ESC can read
-Add arming Protical which starts by giving ESC minimal throttle (to calibrate)
-Implement Flgiht Control System
-Additional MAVLINK Features

-Note ESP32 Has two processors and could benifit from multithreading (flight calculations)
*/

//Startup
void setup() {
  initialize();
  network.Init();
  comms.Init();
  sensors.Init();
  flight.Init();
}

//Main Program Loop
void loop() {
  network.Update();
  comms.Update();
  sensors.Update();
  flight.Update();
}

//Support Functions
//------------------------
void initialize(void){
  //Setup Pins
  pinMode(EXTRA_PIN,INPUT);
  //Configure Serial Communications
  Serial.begin(9600,SERIAL_8N1);
  Serial.setTimeout(500);
}


