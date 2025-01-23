// Bicopter Communications
// Written by Matthew Santos

//Features to Implement
//-------------------------
/*
Heartbeat
Manual Control
Camera Protocol
Ping Protocol
Battery Protocol

*/



//Transmit MAVLINK Messages
bool MAVLINK_Write(mavlink_message_t msg){
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  return true;  //fix to be error code
}

//Recieve MAVLINK Messages
bool MAVLINK_Read(){
  //Serial.read();
  //if(length of read = 0 ) return false;

  //Read Data
  //Process CMD 


}

//Process MAVLINK Messages
bool MAVLINK_Process(mavlink_message_t msg){

  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      //We got a heartbeat back from station...
      break;
    case MAVLINK_MSG_ID_PROTOCOL_VERSION:
    case MAVLINK_MSG_ID_SYS_STATUS:
    case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
    case MAVLINK_MSG_ID_:
    case MAVLINK_MSG_ID_:
    case MAVLINK_MSG_ID_:
    case MAVLINK_MSG_ID_:
    case MAVLINK_MSG_ID_:
    case MAVLINK_MSG_ID_:
    case MAVLINK_MSG_ID_:




    case MAVLINK_MSG_ID_MANUAL_CONTROL:
      mavlink_manual_control_t manualControl;
      mavlink_msg_manual_control_decode(&msg, &manualControl);
      break;
    default:
      Serial.print("Recieved Message: ");
      Serial.println(msg.msgid);
      break;

  }
  
  

  //Heartbeat (Periodic Send)
  mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_STANDBY);
uint16_t length =  mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)

}

//Configurations for MAVLINK (move to config.h later)
#define MAVLINK_SYSTEM_ID 1
