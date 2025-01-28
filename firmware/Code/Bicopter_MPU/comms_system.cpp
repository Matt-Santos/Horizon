// Bicopter Communications
// Written by Matthew Santos

#include <comms_system.h>

Comms_System comms;

//Public Functions
//---------------------
void Comms_System::Init(){
  if(!MAVLINK_Init()) Serial.println("Error Initializing Comms->MAVLINK");
}
void Comms_System::Update(){
  MAVLINK_Streams();  //Transmit Continous MAVLINK Messages
  MAVLINK_Read();     //Recieve MAVLINK Messages and Respond
}
//Private Functions
//---------------------
//Initialize MAVLINK Communications
bool Comms_System::MAVLINK_Init(){
  udp.begin(MAVLINK_UDP_PORT);
  return true;
}
//Transmit MAVLINK Messages
void Comms_System::MAVLINK_Write(mavlink_message_t msg){
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  udp.beginPacket("255.255.255.255", MAVLINK_UDP_PORT);
  udp.write(buf, len);
  udp.endPacket();
}
//Recieve MAVLINK Messages
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
//Tracks Stream Message Timing to Ensure they are sent at specific rates
bool Comms_System::Stream_Check(int ID,int interval){
  static int last_time[MAX_LENGTH_SENDS];
  if (millis() - last_time[ID] < interval) return false;
  last_time[ID] = millis();
  return true;
}
//MAVLINK Send Data Periodicly (Streaming Information)
void Comms_System::MAVLINK_Streams(){
  mavlink_message_t msg;
  if(Stream_Check(0,HeartBeat_Interval)){
    mavlink_msg_heartbeat_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,HeartBeat_Chan,&msg,&Data_Heartbeat);
    MAVLINK_Write(msg);
  }
  if(Stream_Check(1,SystemStatus_Interval)){
    Data_SystemStatus.voltage_battery = sensor.BAT_Level;
    Data_SystemStatus.current_battery = -1;
    Data_SystemStatus.battery_remaining = -1;
    mavlink_msg_sys_status_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,SystemStatus_Chan,&msg,&Data_SystemStatus);
    MAVLINK_Write(msg);
  }
  if(Stream_Check(2,ATTITUDE_Interval)){
    Data_Attitude.time_boot_ms = millis();  //Overflows in 49 days
    Data_Attitude.roll = (float) sensor.w[0];
    Data_Attitude.pitch = (float) sensor.w[1];
    Data_Attitude.yaw = (float) sensor.w[2];
    Data_Attitude.rollspeed = (float) sensor.w_dot[0];
    Data_Attitude.pitchspeed = (float) sensor.w_dot[1];
    Data_Attitude.yawspeed = (float) sensor.w_dot[2];
    mavlink_msg_attitude_encode_chan(MAVLINK_SYSTEM_ID,MAV_COMP_ID_AUTOPILOT1,ATTITUDE_Chan,&msg,&Data_Attitude);
    MAVLINK_Write(msg);
  }
}
//Process MAVLINK Messages
bool Comms_System::MAVLINK_Process(mavlink_message_t msg){
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
        Serial.print("Recieved Unexpected MAVLink Message: ");
        Serial.println(msg.msgid);
      break;
  }
  return true;
}