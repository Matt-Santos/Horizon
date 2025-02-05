//Bicopter Flight System
//Written by Matthew Santos

#include "flight_system.h"
#include "storage_system.h"
#include "sensor_system.h"
#include "comms_system.h"

//Motor Settings (MOTOR)
#define PWM_Frequency     400   //[Hz]
#define Motor_PWM_L       32    //Left Motor GPIO Pin
#define Motor_PWM_R       33    //Right Motor GPIO Pin
#define PWM_MAX           4096-25
#define PWM_MIN           0

extern Storage_System *storage;
extern Sensor_System *sensor;
extern Comms_System *comms;

//Motor Control (MOTOR)
//---------------------
bool Flight_System::MOTOR_Init(){
  //Initialize Motors
  bool success = true;
  pinMode(Motor_PWM_L,OUTPUT);
  pinMode(Motor_PWM_R,OUTPUT);
  success &= ledcSetClockSource((ledc_clk_cfg_t) 4); //APB clock
  success &= ledcAttach(Motor_PWM_L, PWM_Frequency, 12);
  success &= ledcAttach(Motor_PWM_R, PWM_Frequency, 12);
  // success &= ledcWrite(Motor_PWM_L,0);
  // success &= ledcWrite(Motor_PWM_R,0);
  success &= ledcOutputInvert(Motor_PWM_L,true);
  success &= ledcOutputInvert(Motor_PWM_R,true);
  MOTOR_ESC_Calibrate();
  return success;
}
void Flight_System::MOTOR_ESC_Calibrate(){  //Must be run at startup
  //Throttle Range Calibration Procedure
  ledcWrite(Motor_PWM_L,PWM_MAX); //Update Left Motor
  ledcWrite(Motor_PWM_R,PWM_MAX); //Update Right Motor
  delay(3000);  //Wait for two beeps
  ledcWrite(Motor_PWM_L,PWM_MIN); //Update Left Motor
  ledcWrite(Motor_PWM_R,PWM_MIN); //Update Right Motor
  delay(3000);
}

//Stabalize (STABILIZE)
//---------------------


//Flight System Class 
//--------------------------
void Flight_System::Fast_Init(){
  MOTOR_Init();
}
void Flight_System::Init(){
  if(!MOTOR_Init()) Serial.println("Error Initializing Flight->Motor");
}
void Flight_System::Update(){
  if (ARMED){
    float L = (512.0*comms->MAVLINK_manual_control.z)/125.0-(512.0*comms->MAVLINK_manual_control.y)/625.0;
    float R = (512.0*comms->MAVLINK_manual_control.z)/125.0+(512.0*comms->MAVLINK_manual_control.y)/625.0;
    L_PWM = (unsigned int) constrain(L,PWM_MIN,PWM_MAX);
    R_PWM = (unsigned int) constrain(R,PWM_MIN,PWM_MAX);
  }
  
  //Update Motor PWM
  ledcWrite(Motor_PWM_L,L_PWM); //Update Left Motor
  ledcWrite(Motor_PWM_R,R_PWM); //Update Right Motor

  //Debug Print Code
  Serial.print("x_in:");
  Serial.print(comms->MAVLINK_manual_control.x);Serial.print(",");
  Serial.print("y_in:");
  Serial.print(comms->MAVLINK_manual_control.y);Serial.print(",");
  Serial.print("z_in:");
  Serial.print(comms->MAVLINK_manual_control.z);Serial.print(",");
  Serial.print("L:");
  Serial.print(L_PWM);Serial.print(",");
  Serial.print("R:");
  Serial.println(R_PWM);
}
