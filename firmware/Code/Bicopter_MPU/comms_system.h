// Bicopter Communications
// Written by Matthew Santos

/* Author Notes
- this system includes these submodules
  - Mavlink Protocol (MAVLINK)
- the MAVLINK module handles communication with GCS
*/

#ifndef COMMS_SYSTEM_H
#define COMMS_SYSTEM_H

#include <WiFiUdp.h>
#include <MAVLink_common.h>
#include "time.h"
#include "storage_system.h"
#include "sensor_system.h"

class Comms_System {

  //Mavlink Protocol (MAVLINK)
  //--------------------------
  public:
    //MAVLINK Varriables
    mavlink_heartbeat_t MAVLINK_heartbeat;
    mavlink_sys_status_t MAVLINK_sys_status;
    mavlink_attitude_t MAVLINK_attitude;
    mavlink_param_value_t MAVLINK_param_value;
    mavlink_autopilot_version_t MAVLINK_autopilot_version;
    mavlink_manual_control_t MAVLINK_manual_control;
    mavlink_mission_count_t MAVLINK_mission_count;
    mavlink_command_ack_t MAVLINK_command_ack;
  private:
    WiFiUDP udp;
    //MAVLINK Functions
    bool MAVLINK_Init();                          //Initialize MAVLINK Interface
    void MAVLINK_Write(mavlink_message_t msg);    //Transmit MAVLINK Messages
    void MAVLINK_Read();                          //Recieve MAVLINK Messages
    bool MAVLINK_Process(mavlink_message_t msg);  //Process MAVLINK Messages
    void MAVLINK_Streams();                       //MAVLINK Send Data Periodicly (Streaming Information)
    bool Stream_Check(int ID,int interval);       //Tracks Stream Message Timing to Ensure they are sent at specific rates

  //Comms System Class 
  //--------------------------
  public:
    void Init();
    void Update();
    static Comms_System* get();  //Access Singleton Comms Class
  private:
    Comms_System(){};
    static Comms_System* instance;
    
};

#endif