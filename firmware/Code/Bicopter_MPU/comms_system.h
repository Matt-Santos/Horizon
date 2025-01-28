// Bicopter Communications
// Written by Matthew Santos
#ifndef COMMS_SYSTEM_H
#define COMMS_SYSTEM_H

#include <WiFiUdp.h>
#include <MAVLink_common.h>
#include "time.h"
#include "sensor_system.h"

//Configurations for MAVLINK (Move to configs.h)
#define MAVLINK_SYSTEM_ID             1
#define MAVLINK_UDP_PORT              14550

#define MAX_LENGTH_SENDS              3   //Maximum Number of SEND MESSAGES

#define HeartBeat_Interval            1000
#define SystemStatus_Interval         10000
#define ATTITUDE_Interval             100

#define HeartBeat_Chan                1
#define SystemStatus_Chan             1
#define ATTITUDE_Chan                 1
#define ARM_Chan                      1
#define CMDLong_Chan                  1
#define Data_ParamValue_Chan          1
#define MISSONCount_Chan              1
#define ManualControl_Chan            1

class Comms_System {
  public:
    //Interface Varriables
    WiFiUDP udp;
    //MAVLink Generic Varriables

    //MAVLink Stream Varriables
    mavlink_heartbeat_t Data_Heartbeat = {
      .type = MAV_TYPE_TRICOPTER,
      .autopilot = MAV_AUTOPILOT_GENERIC,
      .base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED,
      .system_status = MAV_STATE_BOOT  //this changes
    };
    mavlink_sys_status_t Data_SystemStatus = {
      .onboard_control_sensors_present = 0b00000010000000001000000000000011,
      .onboard_control_sensors_enabled = 0b00000010000000001000000000000011,
      .onboard_control_sensors_health  = 0b00000010000000001000000000000011,
      .voltage_battery = 0,   //this changes
      .current_battery = 0,   //this changes
      .battery_remaining = 0  //this changes
    };
    mavlink_attitude_t Data_Attitude;
    //MAVLink Response Varriables
    mavlink_param_value_t Data_ParamValue = { //these are just a test for now
      .param_value = 1,
      .param_count = 1,
      .param_index = 0,
      .param_id = "Test",
      .param_type = MAV_PARAM_TYPE_REAL32
    };
    mavlink_autopilot_version_t Data_AutoPilotVersion = {
      .capabilities = 0
      //.flight_sw_version = 2,
      //.board_version = 1,
    };
    mavlink_manual_control_t Data_ManualControl;
    mavlink_mission_count_t Data_MissionCount = {
      .count = 0,
      .target_system = MAVLINK_SYSTEM_ID,
      .target_component = MAV_COMP_ID_AUTOPILOT1
    };
    mavlink_command_ack_t Data_ACK;

    //Public Functions
    //---------------------
    void Init();
    void Update();
  private:
    //Private Functions
    //---------------------
    //Initialize MAVLINK Communications
    bool MAVLINK_Init();
    //Transmit MAVLINK Messages
    void MAVLINK_Write(mavlink_message_t msg);
    //Recieve MAVLINK Messages
    void MAVLINK_Read();
    //Tracks Stream Message Timing to Ensure they are sent at specific rates
    bool Stream_Check(int ID,int interval);
    //MAVLINK Send Data Periodicly (Streaming Information)
    void MAVLINK_Streams();
    //Process MAVLINK Messages
    bool MAVLINK_Process(mavlink_message_t msg);
};

extern Comms_System comms;

#endif