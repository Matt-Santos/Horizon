// Bicopter Communications
// Written by Matthew Santos

#include <comms_system.h>
#include "storage_system.h"
#include "sensor_system.h"
#include "flight_system.h"

//MAVLINK Settings (MAVLINK)
#define MAV_SYSTEM_ID                 1
#define MAV_COMP_ID                   1
#define MAV_CHAN_ID                   MAVLINK_COMM_0
#define MAVLINK_UDP_PORT              14550

//Moved to Storage Class (after confirming parameter NVS saving)
#define HeartBeat_Interval            1000    //[ms]
#define SystemStatus_Interval         10000   //[ms]
#define ATTITUDE_Interval             100     //[ms]
#define ALTITUDE_Interval             100     //[ms]
#define GPS_POSITION_Interval         1000    //[ms]
#define GPS_RAW_Interval              1000    //[ms]

//Make this the size of above storage section
#define MAX_LENGTH_SENDS              20   //Maximum Number of SEND MESSAGES

extern Storage_System *storage;
extern Sensor_System *sensor;
extern Flight_System *flight;

//Mavlink Protocol (MAVLINK)
//--------------------------
bool Comms_System::MAVLINK_Init(){
  udp.begin(MAVLINK_UDP_PORT);
  //Initialize Varriables
   MAVLINK_heartbeat = {
    .type = MAV_TYPE_TRICOPTER,
    .autopilot = MAV_AUTOPILOT_GENERIC,
    .base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
    .system_status = MAV_state
  };
  MAVLINK_sys_status = {
    .onboard_control_sensors_present = 0b00000010000000001000000000101011,
    .onboard_control_sensors_enabled = 0b00000010000000001000000000101011,
    .onboard_control_sensors_health  = 0b00000010000000001000000000101011,
    .voltage_battery = 0,
    .current_battery = -1,
    .battery_remaining = -1
  };
  MAVLINK_autopilot_version = {
    .capabilities = 0,
    .uid = 847373744262144613694,
    .flight_sw_version = 1
  };
  MAVLINK_protocolVersion = {
    .version = 201,
    .min_version = 200
  };
  MAVLINK_componentMetaData = {
    .file_crc = 2241, //metadata file CRC32
    .uri = "ftp_resource_here"
  };
  MAVLINK_mission_count = {
    .count = 0,
    .target_system = MAV_SYSTEM_ID,
    .target_component = MAV_COMP_ID
  };
  // MAVLINK_gimbalManagerInfo = {
  //   .cap_flags = 0b0000000100100100,
  //   .roll_min = -PI,
  //   .roll_max = PI,
  //   .pitch_min = -PI,
  //   .pitch_max = PI,
  //   .yaw_min = -2*PI,
  //   .yaw_max = 2*PI
  // };
  MAVLINK_param_value.param_count = sizeof(storage->param)/sizeof(storage->param[0]);
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
  mavlink_message_t cmd_response_msg;
  switch (msg.msgid) {
    //General Information
    case MAVLINK_MSG_ID_HEARTBEAT:  //0 (not used atm, GCS sends on initial connection)
      break;
    case MAVLINK_MSG_ID_SYSTEM_TIME:  //2 (GCS sends unix time but not used atm)
      mavlink_msg_system_time_decode(&msg,&MAVLINK_system_time);
      //Serial.println(MAVLINK_system_time.time_unix_usec);
      break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: //20
      //Recieve Parameter Index
      MAVLINK_param_value.param_index = mavlink_msg_param_request_read_get_param_index(&msg);
      if (MAVLINK_param_value.param_index == -1){ //Request by ID instead of index
        mavlink_msg_param_request_read_get_param_id(&msg,MAVLINK_param_value.param_id);
        for (uint16_t i=0;i<MAVLINK_param_value.param_count;i++){
          if (MAVLINK_param_value.param_id == storage->param[i].param_id){
            MAVLINK_param_value.param_index = i;
            break;
          }
        }
      }
      //Send Parameter by Index
      MAVLINK_param_value.param_value = storage->param[MAVLINK_param_value.param_index].value.param_float;
      memcpy(MAVLINK_param_value.param_id,storage->param[MAVLINK_param_value.param_index].param_id,16);
      MAVLINK_param_value.param_type = storage->param[MAVLINK_param_value.param_index].value.type;
      mavlink_msg_param_value_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_param_value);
      MAVLINK_Write(msg);
      break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //21
      for (uint16_t i=0;i<MAVLINK_param_value.param_count;i++){
        MAVLINK_param_value.param_value = storage->param[i].value.param_float;
        memcpy(MAVLINK_param_value.param_id,storage->param[i].param_id,16);
        MAVLINK_param_value.param_type = storage->param[i].value.type;
        MAVLINK_param_value.param_index = i;
        mavlink_msg_param_value_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_param_value);
        MAVLINK_Write(msg);
      }
      break;
    case MAVLINK_MSG_ID_PARAM_SET:  //23
      mavlink_msg_param_set_decode(&msg,&MAVLINK_param_set);
      for (uint16_t i=0;i<MAVLINK_param_value.param_count;i++){
        if (strcmp(MAVLINK_param_value.param_id,storage->param[i].param_id) == 0){
          MAVLINK_param_value.param_index = i;
          break;
        }
      }
      //Update Newly Set Parameter
      storage->param[MAVLINK_param_value.param_index].value.param_float = MAVLINK_param_set.param_value;
      memcpy(storage->param[MAVLINK_param_value.param_index].param_id,MAVLINK_param_set.param_id,16);
      storage->param[MAVLINK_param_value.param_index].value.type = MAVLINK_param_set.param_type;
      //Reply with new Value
      MAVLINK_param_value.param_value = storage->param[MAVLINK_param_value.param_index].value.param_float;
      memcpy(MAVLINK_param_value.param_id,storage->param[MAVLINK_param_value.param_index].param_id,16);
      MAVLINK_param_value.param_type = storage->param[MAVLINK_param_value.param_index].value.type;
      mavlink_msg_param_value_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_param_value);
      MAVLINK_Write(msg);
      break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: //43 (sends dummy info atm)
      mavlink_msg_mission_count_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_mission_count);
      MAVLINK_Write(msg);
      break;
    //case MAVLINK_MSG_ID_MISSION_ACK: //47 (todo)
    //case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:  //48 (todo)
    case MAVLINK_MSG_ID_COMMAND_LONG: //76 (wip)
      mavlink_msg_command_long_decode(&msg,&cmd);
      cmd_response_msg.seq = 255;
      switch (cmd.command){
        case MAV_CMD_REQUEST_MESSAGE:  //512 (needs mftp implementation and target metadata source information)
          switch ((int) cmd.param1){
            case MAVLINK_MSG_ID_AUTOPILOT_VERSION:  //148
              mavlink_msg_autopilot_version_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&cmd_response_msg,&MAVLINK_autopilot_version);
              break;
            case MAVLINK_MSG_ID_PROTOCOL_VERSION: //300
              mavlink_msg_protocol_version_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&cmd_response_msg,&MAVLINK_protocolVersion);
              break;
            case MAVLINK_MSG_ID_COMPONENT_METADATA:  //397 (need to define file_crc and uri)
              MAVLINK_componentMetaData.time_boot_ms = millis();
              strcpy(MAVLINK_componentMetaData.uri,"mftp://system_metadata.xml");
              MAVLINK_componentMetaData.file_crc = 0x44;
              mavlink_msg_component_metadata_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&cmd_response_msg,&MAVLINK_componentMetaData);
              break;
            case MAVLINK_MSG_ID_COMPONENT_INFORMATION:  //395 (need to define file_crc and uri)
              MAVLINK_componentInfo.time_boot_ms = millis();
              strcpy(MAVLINK_componentMetaData.uri,"mftp://system_info.xml");
              MAVLINK_componentMetaData.file_crc = 0x42;
              mavlink_msg_component_information_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&cmd_response_msg,&MAVLINK_componentInfo);
              break;
            case MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION:  // 280 (update initialization gyro limits)
              MAVLINK_gimbalManagerInfo.time_boot_ms = millis();
              mavlink_msg_gimbal_manager_information_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&cmd_response_msg,&MAVLINK_gimbalManagerInfo);
              break;
            case MAVLINK_MSG_ID_CAMERA_INFORMATION: //259 (todo)
              break;
            default:
              Serial.print("Recieved Unexpected MAVLink CMD Request: ");
              Serial.println((int) cmd.param1);
              break;
          }
          MAVLINK_command_ack.result = MAV_RESULT_ACCEPTED;
          break;
        case MAV_CMD_COMPONENT_ARM_DISARM:  //400
          flight->ARMED = cmd.param1;
          //Send Warning Text
          memcpy(MAVLINK_statusText.text,"System State Changed",50);
          MAVLINK_statusText.severity = 3;
          mavlink_msg_statustext_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&cmd_response_msg,&MAVLINK_statusText);
          MAVLINK_command_ack.result = MAV_RESULT_ACCEPTED;
          break;
        case MAV_CMD_DO_SET_MODE: //176
          // flight->flight_mode = cmd.param1;
          MAVLINK_command_ack.result = MAV_RESULT_ACCEPTED;
          break;
        default:
          Serial.print("Recieved Unexpected MAVLink CMD: ");
          Serial.println(cmd.command);
          MAVLINK_command_ack.result = MAV_RESULT_UNSUPPORTED;
          break;
      }
      //Send ACK Response
      MAVLINK_command_ack.command = cmd.command;
      MAVLINK_command_ack.target_system = cmd.target_system;
      MAVLINK_command_ack.target_component = cmd.target_component;
      mavlink_msg_command_ack_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_command_ack);
      MAVLINK_Write(msg);
      //Send Response Data (if applicable)
      if (cmd_response_msg.seq != 255) MAVLINK_Write(cmd_response_msg);
      break;
    case MAVLINK_MSG_ID_MANUAL_CONTROL: //69
      mavlink_msg_manual_control_decode(&msg, &MAVLINK_manual_control);
      break;
    //case MAVLINK_MSG_ID_MANUAL_SETPOINT:  //81 (todo)
    //case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:  //82 (todo)
    //case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: //86 (todo)
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: //110 (WIP more functions to add)
      mavlink_msg_file_transfer_protocol_decode(&msg,&MAVLINK_ftp);
      if (MAVLINK_ftp.target_system == MAV_SYSTEM_ID){
        MAVLINK_ftp.payload[5] = MAVLINK_ftp.payload[3];  //Load Recieved OpCode
        switch(MAVLINK_ftp.payload[3]){ //opcode
          case MAV_FTP_OPCODE_OPENFILERO:
            if (storage->SDCARD_createDir((const char*) &MAVLINK_ftp.payload[12])) {
              MAVLINK_ftp.payload[3] = MAV_FTP_OPCODE_ACK;
            }
            else{
              MAVLINK_ftp.payload[3] = MAV_FTP_OPCODE_NAK;
              MAVLINK_ftp.payload[12] = MAV_FTP_ERR_FILEEXISTS;
            }
            MAVLINK_ftp.payload[4] = 0; //payload size
            break;
          // case MAV_FTP_OPCODE_OPENFILEWO:
          // case MAV_FTP_OPCODE_READFILE:
          // case MAV_FTP_OPCODE_CREATEFILE:
          // case MAV_FTP_OPCODE_WRITEFILE:
          // case MAV_FTP_OPCODE_LISTDIRECTORY:
          case MAV_FTP_OPCODE_CREATEDIRECTORY:  //done
            if (storage->SDCARD_createDir((const char*) &MAVLINK_ftp.payload[12])) {
              MAVLINK_ftp.payload[3] = MAV_FTP_OPCODE_ACK;
            }
            else{
              MAVLINK_ftp.payload[3] = MAV_FTP_OPCODE_NAK;
              MAVLINK_ftp.payload[12] = MAV_FTP_ERR_FILEEXISTS;
            }
            MAVLINK_ftp.payload[4] = 0; //payload size
            break;
          // case MAV_FTP_OPCODE_RENAME:
          // case MAV_FTP_OPCODE_CALCFILECRC:
          // case MAV_FTP_OPCODE_BURSTREADFILE:
          // case MAV_FTP_OPCODE_RESETSESSION:
          // case MAV_FTP_OPCODE_TERMINATESESSION:
          case MAV_FTP_OPCODE_NONE: //Send ACK
            MAVLINK_ftp.payload[3] = MAV_FTP_OPCODE_ACK;
            MAVLINK_ftp.payload[4] = 0;  //payload size
            break;
          default:
            Serial.print("Recieved Unexpected MAVLink FTP Opcode: ");
            Serial.println(MAVLINK_ftp.payload[3]);
            break;
        }
      }
      //Send Response
      mavlink_msg_file_transfer_protocol_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_ftp);
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
  if(Stream_Check(0,HeartBeat_Interval)){ //Heartbeat
    mavlink_msg_heartbeat_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_heartbeat);
    MAVLINK_Write(msg);
  }
  if(Stream_Check(1,SystemStatus_Interval)){  //System Status
    MAVLINK_sys_status.voltage_battery = sensor->BAT_Level;
    mavlink_msg_sys_status_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_sys_status);
    MAVLINK_Write(msg);
  }
  if(Stream_Check(2,ATTITUDE_Interval)){  //Attitude
    MAVLINK_attitude.time_boot_ms = millis();
    MAVLINK_attitude.roll = (float) sensor->w[0];
    MAVLINK_attitude.pitch = (float) sensor->w[1];
    MAVLINK_attitude.yaw = (float) sensor->w[2];
    MAVLINK_attitude.rollspeed = (float) sensor->w_dot[0];
    MAVLINK_attitude.pitchspeed = (float) sensor->w_dot[1];
    MAVLINK_attitude.yawspeed = (float) sensor->w_dot[2];
    mavlink_msg_attitude_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_attitude);
    MAVLINK_Write(msg);
  }
  if(Stream_Check(3,ALTITUDE_Interval)){  //Altitude
    MAVLINK_altitude.time_usec = micros();
    MAVLINK_altitude.altitude_monotonic = 0;
    MAVLINK_altitude.altitude_amsl = 0;
    MAVLINK_altitude.altitude_local = 0;
    MAVLINK_altitude.altitude_relative = 0;
    MAVLINK_altitude.altitude_terrain = -1000;
    MAVLINK_altitude.bottom_clearance = -1;
    mavlink_msg_altitude_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_altitude);
    MAVLINK_Write(msg);
  }
  // if(Stream_Check(4,GPS_POSITION_Interval)){  //Global Position Estimate
  //   MAVLINK_global_position_int.time_boot_ms = millis();
  //   MAVLINK_global_position_int.alt = sensor->altitude;             //[mm] MSL Altitude
  //   MAVLINK_global_position_int.lat = sensor->latitude;             //[degE7] Latitude
  //   MAVLINK_global_position_int.lon = sensor->longitude;            //[degE7] Longitude
  //   // MAVLINK_global_position_int.relative_alt = ;  //[mm] Altitude above Home
  //   // MAVLINK_global_position_int.hdg = sensor->gps.course()/100.0;   //[deg] Vehicle heading
  //   // MAVLINK_global_position_int.vx = sensor->x_dot[0]*100.0;  //[cm/s] Ground speed along North Axis
  //   // MAVLINK_global_position_int.vy = sensor->x_dot[1]*100.0;  //[cm/s] Ground speed along East Axis
  //   // MAVLINK_global_position_int.vz = sensor->x_dot[2]*100.0;  //[cm/s] Ground speed along Altitude (positive down)
  //   mavlink_msg_global_position_int_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_global_position_int);
  //   MAVLINK_Write(msg);
  // }
  if(Stream_Check(5,GPS_RAW_Interval)){  //GPS Raw Sensor Data
    MAVLINK_gps_raw.time_usec = micros();
    MAVLINK_gps_raw.lat = sensor->latitude;               //[degE7] Latitude
    MAVLINK_gps_raw.lon = sensor->longitude;              //[degE7] Latitude
    MAVLINK_gps_raw.alt = sensor->gps.altitude()*10;      //[mm] MSL Altitude
    MAVLINK_gps_raw.eph = sensor->gps.hdop();             //[] Horizontal Dilution of Position
    MAVLINK_gps_raw.epv = UINT16_MAX;                     //[] Vertical Dilution of Position
    MAVLINK_gps_raw.vel = sensor->gps.f_speed_mps()*100;  //[cm/s] 
    MAVLINK_gps_raw.cog = sensor->gps.course();           //[deg*100] Course (direction of movement)
    MAVLINK_gps_raw.fix_type = GPS_FIX_TYPE_3D_FIX;
    MAVLINK_gps_raw.satellites_visible = sensor->gps.satellites();

    MAVLINK_gps_raw.h_acc = 100;
    MAVLINK_gps_raw.v_acc = 100;
    MAVLINK_gps_raw.vel_acc = 100;
    MAVLINK_gps_raw.hdg_acc = 100;

    // MAVLINK_gps_raw.yaw = sensor->gps.
    mavlink_msg_gps_raw_int_encode_chan(MAV_SYSTEM_ID,MAV_COMP_ID,MAV_CHAN_ID,&msg,&MAVLINK_gps_raw);
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
