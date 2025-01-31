// Bicopter Communications
// Written by Matthew Santos

#include <comms_system.h>

//MAVLINK Settings (MAVLINK)
#define MAV_SYSTEM_ID                 1
#define MAV_COMP_ID                   1
#define MAV_CHAN_ID                   1
#define MAVLINK_UDP_PORT              14550

//Moved to Storage Class (after confirming parameter NVS saving)
#define HeartBeat_Interval            1000
#define SystemStatus_Interval         10000
#define ATTITUDE_Interval             100
//Make this the size of above storage section
#define MAX_LENGTH_SENDS              3   //Maximum Number of SEND MESSAGES

Comms_System* Comms_System::instance = nullptr;

//Mavlink Protocol (MAVLINK)
//--------------------------
bool Comms_System::MAVLINK_Init(){
  udp.begin(MAVLINK_UDP_PORT);
  //Initialize Varriables
   MAVLINK_heartbeat = {
    .type = MAV_TYPE_TRICOPTER,
    .autopilot = MAV_AUTOPILOT_GENERIC,
    .base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED,
    .system_status = MAV_STATE_BOOT  //this changes
  };
  MAVLINK_sys_status = {
    .onboard_control_sensors_present = 0b00000010000000001000000000000011,
    .onboard_control_sensors_enabled = 0b00000010000000001000000000000011,
    .onboard_control_sensors_health  = 0b00000010000000001000000000000011,
    .voltage_battery = 0,   //this changes
    .current_battery = 0,   //this changes
    .battery_remaining = 0  //this changes
  };
  MAVLINK_param_value = { //these are just a test for now
    .param_value = 1,
    .param_count = 1,
    .param_index = 0,
    .param_id = "Test",
    .param_type = MAV_PARAM_TYPE_REAL32
  };
  MAVLINK_autopilot_version = {
    .capabilities = 0
    //.flight_sw_version = 2,
    //.board_version = 1,
  };
  MAVLINK_manual_control;
  MAVLINK_mission_count = {
    .count = 0,
    .target_system = MAV_SYSTEM_ID,
    .target_component = MAV_COMP_ID
  };
  return true;
}
void Comms_System::MAVLINK_Write(mavlink_message_t msg){
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  udp.beginPacket("255.255.255.255", MAVLINK_UDP_PORT);
  udp.write(buf, len);
  udp.endPacket();
}
void Comms_System::MAVLINK_Read(){
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
bool Comms_System::MAVLINK_Process(mavlink_message_t msg){
  mavlink_command_long_t cmd;
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:  //0
      break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //21
      mavlink_msg_param_value_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_param_value);
      MAVLINK_Write(msg);
      break;
    case MAVLINK_MSG_ID_COMMAND_LONG: //76
      mavlink_msg_command_long_decode(&msg,&cmd);
      switch (cmd.command){
        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
          mavlink_msg_autopilot_version_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_autopilot_version);
          MAVLINK_Write(msg);
          break;
        case MAV_CMD_COMPONENT_ARM_DISARM:
          MAVLINK_command_ack.command = MAV_CMD_COMPONENT_ARM_DISARM;
          MAVLINK_command_ack.result = MAV_RESULT_ACCEPTED;
          mavlink_msg_command_ack_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_command_ack);
          MAVLINK_Write(msg);
          break;
      }
      break;
    case MAVLINK_MSG_ID_MANUAL_CONTROL: //69
      mavlink_msg_manual_control_decode(&msg, &MAVLINK_manual_control);
      break;
    //Camera Control
      // case MAV_CMD_VIDEO_START_CAPTURE:
      // case MAV_CMD_VIDEO_STOP_CAPTURE:
    //Navigation
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: //43
      mavlink_msg_mission_count_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_mission_count);
      MAVLINK_Write(msg);
      break;
    default:
        Serial.print("Recieved Unexpected MAVLink Message: ");
        Serial.println(msg.msgid);
      break;
  }
  return true;
}
void Comms_System::MAVLINK_Streams(){
  mavlink_message_t msg;
  if(Stream_Check(0,HeartBeat_Interval)){
    mavlink_msg_heartbeat_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_heartbeat);
    MAVLINK_Write(msg);
  }
  if(Stream_Check(1,SystemStatus_Interval)){
    MAVLINK_sys_status.voltage_battery = Sensor_System::get()->BAT_Level;
    MAVLINK_sys_status.current_battery = -1;
    MAVLINK_sys_status.battery_remaining = -1;
    mavlink_msg_sys_status_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_sys_status);
    MAVLINK_Write(msg);
  }
  if(Stream_Check(2,ATTITUDE_Interval)){
    MAVLINK_attitude.time_boot_ms = millis();  //Overflows in 49 days
    MAVLINK_attitude.roll = (float) Sensor_System::get()->w[0];
    MAVLINK_attitude.pitch = (float) Sensor_System::get()->w[1];
    MAVLINK_attitude.yaw = (float) Sensor_System::get()->w[2];
    MAVLINK_attitude.rollspeed = (float) Sensor_System::get()->w_dot[0];
    MAVLINK_attitude.pitchspeed = (float) Sensor_System::get()->w_dot[1];
    MAVLINK_attitude.yawspeed = (float) Sensor_System::get()->w_dot[2];
    mavlink_msg_attitude_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_attitude);
    MAVLINK_Write(msg);
  }
}
bool Comms_System::Stream_Check(int ID,int interval){
  static int last_time[MAX_LENGTH_SENDS];
  if (millis() - last_time[ID] < interval) return false;
  last_time[ID] = millis();
  return true;
}

//Comms System Class 
//--------------------------
void Comms_System::Init(){
  if(!MAVLINK_Init()) Serial.println("Error Initializing Comms->MAVLINK");
}
void Comms_System::Update(){
  MAVLINK_Streams();  //Transmit Continous MAVLINK Messages
  MAVLINK_Read();     //Recieve MAVLINK Messages and Respond
}
Comms_System* Comms_System::get() {
  if (instance == nullptr) instance = new Comms_System;
    return instance;
}