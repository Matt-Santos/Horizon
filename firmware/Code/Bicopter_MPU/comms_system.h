// Bicopter Communications
// Written by Matthew Santos

#include <WiFiUdp.h>
#include <MAVLink_common.h>
#include "time.h"

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
    void Init(){
      if(!MAVLINK_Init()) Serial.println("Error Initializing Comms->MAVLINK");
    }
    void Update(){
      MAVLINK_Streams();  //Transmit Continous MAVLINK Messages
      MAVLINK_Read();     //Recieve MAVLINK Messages and Respond
    }
  private:
  //Support Functions
  //---------------------
  //Initialize MAVLINK Communications
  bool MAVLINK_Init(){
    udp.begin(MAVLINK_UDP_PORT);
    return true;
  }
  //Transmit MAVLINK Messages
  void MAVLINK_Write(mavlink_message_t msg){
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    udp.beginPacket("255.255.255.255", MAVLINK_UDP_PORT);
    udp.write(buf, len);
    udp.endPacket();
  }
  //Recieve MAVLINK Messages
  void MAVLINK_Read(){
    int packetSize = udp.parsePacket();
    if (packetSize > 0){
      mavlink_message_t msg;
      mavlink_status_t status;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      udp.read(buf, MAVLINK_MAX_PACKET_LEN);
      for (int i = 0; i < packetSize; i++) {
        uint8_t success = mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status);
        if(success) MAVLINK_Process(msg);
      }
    }
  }
  //Tracks Stream Message Timing to Ensure they are sent at specific rates
  bool Stream_Check(int ID,int interval){
    static int last_time[MAX_LENGTH_SENDS];
    if (millis() - last_time[ID] < interval) return false;
    last_time[ID] = millis();
    return true;
  }
  //MAVLINK Send Data Periodicly (Streaming Information)
  void MAVLINK_Streams(){
    mavlink_message_t msg;
    if(Stream_Check(0,HeartBeat_Interval)){
      mavlink_msg_heartbeat_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,HeartBeat_Chan,&msg,&Data_Heartbeat);
      MAVLINK_Write(msg);
    }
    if(Stream_Check(1,SystemStatus_Interval)){
      Data_SystemStatus.voltage_battery = BAT_Level;
      Data_SystemStatus.current_battery = -1;
      Data_SystemStatus.battery_remaining = -1;
      mavlink_msg_sys_status_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,SystemStatus_Chan,&msg,&Data_SystemStatus);
      MAVLINK_Write(msg);
    }
    if(Stream_Check(2,ATTITUDE_Interval)){
      Data_Attitude.time_boot_ms = millis();  //Overflows in 49 days
      Data_Attitude.roll = (float) IMU_Data.w[0];
      Data_Attitude.pitch = (float) IMU_Data.w[1];
      Data_Attitude.yaw = (float) IMU_Data.w[2];
      Data_Attitude.rollspeed = (float) IMU_Data.w_dot[0];
      Data_Attitude.pitchspeed = (float) IMU_Data.w_dot[1];
      Data_Attitude.yawspeed = (float) IMU_Data.w_dot[2];
      mavlink_msg_attitude_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,ATTITUDE_Chan,&msg,&Data_Attitude);
      MAVLINK_Write(msg);
    }
  }
  //Process MAVLINK Messages
  bool MAVLINK_Process(mavlink_message_t msg){
    static mavlink_command_long_t cmd;
    switch (msg.msgid) {
      //Basic Messages
      case MAVLINK_MSG_ID_HEARTBEAT:  //0
        break;
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //21
        mavlink_msg_param_value_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,Data_ParamValue_Chan,&msg,&Data_ParamValue);
        MAVLINK_Write(msg);
        break;
      case MAVLINK_MSG_ID_COMMAND_LONG: //76
        mavlink_msg_command_long_decode(&msg,&cmd);
        switch (cmd.command){
          case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            mavlink_msg_autopilot_version_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,CMDLong_Chan,&msg,&Data_AutoPilotVersion);
            MAVLINK_Write(msg);
            break;
          case MAV_CMD_COMPONENT_ARM_DISARM:
            Data_ACK.command = MAV_CMD_COMPONENT_ARM_DISARM;
            Data_ACK.result = MAV_RESULT_ACCEPTED;
            mavlink_msg_command_ack_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,ARM_Chan,&msg,&Data_ACK);
            MAVLINK_Write(msg);
            break;
        }
        break;
      case MAVLINK_MSG_ID_MANUAL_CONTROL: //69
        mavlink_msg_manual_control_decode(&msg, &Data_ManualControl);
        L = (512.0*Data_ManualControl.z)/125.0-(512.0*Data_ManualControl.y)/625.0;
        R = (512*Data_ManualControl.z)/125.0+(512.0*Data_ManualControl.y)/625.0;
        L_PWM = (unsigned int) constrain(L,0,4096);
        R_PWM = (unsigned int) constrain(R,0,4096);
        if(DEBUG_MODE){
          Serial.printf("MANUAL_CONTROL: ");
          Serial.printf("x= %d ",Data_ManualControl.x);
          Serial.printf("y= %d ",Data_ManualControl.y);
          Serial.printf("z= %d ",Data_ManualControl.z);
          Serial.printf("r= %d ",Data_ManualControl.r);
          Serial.printf("L_PWM= %d ",L_PWM);
          Serial.printf("R_PWM= %d \n",R_PWM);
        }
        break;
      //Camera Control
        // case MAV_CMD_VIDEO_START_CAPTURE:
        // case MAV_CMD_VIDEO_STOP_CAPTURE:
      //Navigation
      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: //43
        mavlink_msg_mission_count_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,MISSONCount_Chan,&msg,&Data_MissionCount);
        MAVLINK_Write(msg);
        break;
      default:
        if(DEBUG_MODE){
          Serial.print("Recieved Unknown Message: ");
          Serial.println(msg.msgid);
        }
        break;
    }
    return true;
  }
};












