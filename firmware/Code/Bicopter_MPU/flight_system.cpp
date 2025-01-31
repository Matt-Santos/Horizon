//Bicopter Flight System
//Written by Matthew Santos

#include "flight_system.h"

//Motor Settings (MOTOR)
#define PWM_Frequency     10000 //[Hz]
#define Motor_PWM_L       3     //Left Motor GPIO Pin
#define Motor_PWM_R       33    //Right Motor GPIO Pin
#define Motor_PWM_P       32    //Pitch Motor GPIO Pin

Flight_System* Flight_System::instance = nullptr;

//Motor Control (MOTOR)
//---------------------
bool Flight_System::MOTOR_Init(){
  //Initialize Motors
  pinMode(Motor_PWM_L,OUTPUT);
  pinMode(Motor_PWM_R,OUTPUT);
  pinMode(Motor_PWM_P,OUTPUT);
  bool success = true;
  success &= ledcSetClockSource((ledc_clk_cfg_t) 4); //APB clock
  success &= ledcAttach(Motor_PWM_L, PWM_Frequency, 12);
  success &= ledcAttach(Motor_PWM_R, PWM_Frequency, 12);
  success &= ledcAttach(Motor_PWM_P, PWM_Frequency, 12);
  success &= ledcOutputInvert(Motor_PWM_L,true);
  success &= ledcOutputInvert(Motor_PWM_R,true);
  success &= ledcOutputInvert(Motor_PWM_P,true);
  success &= ledcWrite(Motor_PWM_L,0);
  success &= ledcWrite(Motor_PWM_R,0);
  success &= ledcWrite(Motor_PWM_P,0);
  return success;
}

//Flight System Class 
//--------------------------
void Flight_System::Init(){
  if(!MOTOR_Init()) Serial.println("Error Initializing Flight->Motor");
}
void Flight_System::Update(){
  float L = (512.0*Comms_System::get()->MAVLINK_manual_control.z)/125.0-(512.0*Comms_System::get()->MAVLINK_manual_control.y)/625.0;
  float R = (512*Comms_System::get()->MAVLINK_manual_control.z)/125.0+(512.0*Comms_System::get()->MAVLINK_manual_control.y)/625.0;
  L_PWM = (unsigned int) constrain(L,0,4096);
  R_PWM = (unsigned int) constrain(R,0,4096);
  Serial.printf("MANUAL_CONTROL: ");
  Serial.printf("x= %d ",Comms_System::get()->MAVLINK_manual_control.x);
  Serial.printf("y= %d ",Comms_System::get()->MAVLINK_manual_control.y);
  Serial.printf("z= %d ",Comms_System::get()->MAVLINK_manual_control.z);
  Serial.printf("r= %d ",Comms_System::get()->MAVLINK_manual_control.r);
  Serial.printf("L_PWM= %d ",L_PWM);
  Serial.printf("R_PWM= %d \n",R_PWM);
  ledcWrite(Motor_PWM_L,L_PWM); //Update Left Motor
  ledcWrite(Motor_PWM_R,R_PWM); //Update Right Motor
}
Flight_System* Flight_System::get() {
  if (instance == nullptr) instance = new Flight_System;
    return instance;
}