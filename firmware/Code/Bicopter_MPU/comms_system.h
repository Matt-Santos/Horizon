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

class Comms_System {

  //Mavlink Protocol (MAVLINK)
  //--------------------------
  public:
    //MAVLINK Varriables
    mavlink_heartbeat_t MAVLINK_heartbeat;  //0
    mavlink_sys_status_t MAVLINK_sys_status;  //1
    mavlink_system_time_t MAVLINK_system_time;  //2
    mavlink_param_value_t MAVLINK_param_value;  //22
    mavlink_param_set_t MAVLINK_param_set;  //23
    mavlink_gps_raw_int_t MAVLINK_gps_raw;  //24
    mavlink_attitude_t MAVLINK_attitude;  //30
    mavlink_global_position_int_t MAVLINK_global_position_int;  //33
    mavlink_file_transfer_protocol_t MAVLINK_ftp; //110
    mavlink_autopilot_version_t MAVLINK_autopilot_version;  //148
    mavlink_manual_control_t MAVLINK_manual_control;
    mavlink_mission_count_t MAVLINK_mission_count;
    mavlink_command_ack_t MAVLINK_command_ack;  //77
    mavlink_altitude_t MAVLINK_altitude;  //141
    mavlink_statustext_t MAVLINK_statusText;  //253
    mavlink_gimbal_manager_information_t MAVLINK_gimbalManagerInfo; //280
    mavlink_protocol_version_t MAVLINK_protocolVersion; //300
    mavlink_component_information_t MAVLINK_componentInfo;  //395
    mavlink_component_metadata_t MAVLINK_componentMetaData; //397

    //437 (avaliable modes)

    //Comms States
    uint8_t MAV_state = MAV_STATE_BOOT;
    
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
    
};

#endif