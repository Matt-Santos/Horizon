//Bicopter Flight System
//Written by Matthew Santos

#include "flight_system.h"
#include "storage_system.h"
#include "sensor_system.h"
#include "comms_system.h"

//Debug Flags
// #define MOTOR_Debug
// #define MODE_Debug

//I2C Interface
#define I2C_SDA         26      //I2C Data Pin
#define I2C_SCL         27      //I2C Clock Pin
#define I2C_Freq        400000  //[Hz] I2C Clock Frequency

//Motor Settings (MOTOR)
#define PCA9685_Addrs     0x40  //PCA9685 I2C Address
#define Motor_OE_Pin      32    //PCA9685 Output Enable Pin
#define PWM_Frequency     1   //[Hz] PWM Output Frequency
#define L_Motor_Chan      0     //Left  Motor Output Channel
#define R_Motor_Chan      1     //Right Motor Output Channel
#define P_Motor_ChanE     2     //Pitch Motor Enable
#define P_Motor_ChanA1    4     //Pitch Motor Output Channel A1
#define P_Motor_ChanA2    5     //Pitch Motor Output Channel A2
#define P_Motor_ChanB1    7     //Pitch Motor Output Channel B1
#define P_Motor_ChanB2    6     //Pitch Motor Output Channel B2
#define P_Motor_Tol       5     //[%] Pitch Motor Control Tolerene
#define P_Motor_Count     1000  //Number of Pulses to travel Full Pitch Motor Range
#define PWM_MIN           5     //Minimum PWM Output
#define PWM_MAX           95    //Maximum PWM Output


extern Storage_System *storage;
extern Sensor_System *sensor;
extern Comms_System *comms;

//Motor Control (MOTOR)
//---------------------
bool Flight_System::MOTOR_Init(){
  bool success = true;
  //Configure I2C Interface
  pinMode(I2C_SDA,INPUT_PULLUP);
  pinMode(I2C_SCL,INPUT_PULLUP);
  success &= Wire.begin(I2C_SDA,I2C_SCL,I2C_Freq);
  //Setup Motor PWM Driver
  motor.setupSingleDevice(Wire,PCA9685_Addrs);
  motor.setupOutputEnablePin(Motor_OE_Pin);
  motor.setToFrequency(PWM_Frequency);
  motor.disableOutputs(Motor_OE_Pin);
  motor.setOutputsNotInverted();
  //Set Disabled Mode
  motor.setOutputsLowWhenDisabled();
  MOTOR_ESC_Calibrate();
  MOTOR_Pitch_Calibrate();
  return success;
}
void Flight_System::MOTOR_ESC_Calibrate(){
  //Calibrate ESC Throttle Max
  motor.enableOutputs(Motor_OE_Pin);
  motor.setChannelDutyCycle(L_Motor_Chan,PWM_MAX);
  motor.setChannelDutyCycle(R_Motor_Chan,PWM_MAX);
  delay(3000);  //Wait for two beeps
  //Calibrate ESC Throttle Min
  motor.setChannelDutyCycle(L_Motor_Chan,PWM_MIN);
  motor.setChannelDutyCycle(R_Motor_Chan,PWM_MIN);
  //Shutdown ESC
  delay(3000);
  motor.disableOutputs(Motor_OE_Pin);
  motor.setChannelDutyCycle(L_Motor_Chan,0.0);
  motor.setChannelDutyCycle(R_Motor_Chan,0.0);
}
void Flight_System::MOTOR_Pitch_Calibrate(){
  //Force Pitch Motor to Zero Point
  Serial.println("Starting");  
  motor.setChannelDutyCycle(P_Motor_ChanA1,37.5,0);
  motor.setChannelDutyCycle(P_Motor_ChanA2,37.5,50);
  motor.setChannelDutyCycle(P_Motor_ChanB1,37.5,0);
  motor.setChannelDutyCycle(P_Motor_ChanB2,37.5,50);
  motor.setChannelDutyCycle(P_Motor_ChanE,100,0);
  // delay(1000*P_Motor_Count/PWM_Frequency);  //Hold Until Limit Reached
  delay(20000);
  Serial.println("Ending");
  //Set Pitch Motor to Disabled Forward Direction (Initial State)
  motor.setChannelDutyCycle(P_Motor_ChanE,0.0,0.0);
  // motor.setChannelDutyCycle(P_Motor_ChanA1,0,0);
  // motor.setChannelDutyCycle(P_Motor_ChanB1,0,0);
  // motor.setChannelDutyCycle(P_Motor_ChanA2,50,25);
  // motor.setChannelDutyCycle(P_Motor_ChanB2,50,0);
  //Set Initial Target Pitch Position to Midpoint
  P_Motor = 50.0;
}
void Flight_System::MOTOR_Pitch_Control(float target){
  static float position = 50;     //[%] Motor Position as % of Range
  static bool forwards = true;    //Motor Movement Direction (true=forwards)
  static bool enabled = false;    //Motor State (true=enabled)
  static long last;               //[us] Time between Function Calls

  // //Forward Movement
  // if (position < target - P_Motor_Tol){
  //   if(!forwards){
  //     motor.setChannelDutyCycle(P_Motor_ChanA,50.0,0.0);
  //     motor.setChannelDutyCycle(P_Motor_ChanB,50.0,25.0);
  //     forwards = true;
  //   }
  //   if(!enabled){
  //     motor.setChannelDutyCycle(P_Motor_ChanE,100.0,0.0);
  //     enabled = true;
  //   }
  //   position += (micros()-last)*PWM_Frequency/P_Motor_Count*100.0;
  // }
  // //Reverse Movement
  // else if (position > target + P_Motor_Tol){
  //   if(forwards){
  //     motor.setChannelDutyCycle(P_Motor_ChanA,50.0,25.0);
  //     motor.setChannelDutyCycle(P_Motor_ChanB,50.0,0.0);
  //     forwards = false;
  //   }
  //   if(!enabled){
  //     motor.setChannelDutyCycle(P_Motor_ChanE,100.0,0.0);
  //     enabled = true;
  //   }
  //   position -= (micros()-last)*PWM_Frequency/P_Motor_Count*100.0;
  // }
  // //Stop the Motor
  // else {
  //   motor.setChannelDutyCycle(P_Motor_ChanE,0.0,0.0);
  //   enabled = false;
  // }
  // position = constrain(position,0.0,100.0);
  // last = micros();
}
void Flight_System::MOTOR_Update(){
  //Update Armed Status
  if (ARMED)
    motor.enableOutputs(Motor_OE_Pin);
  else
    motor.disableOutputs(Motor_OE_Pin);
  //Update Motor Setpoints
  motor.setChannelDutyCycle(L_Motor_Chan,L_Motor);
  motor.setChannelDutyCycle(R_Motor_Chan,R_Motor);
  MOTOR_Pitch_Control(P_Motor);

  #ifdef MOTOR_Debug
    Serial.printf("Motors L: %f , R: %f , P: %f, ARM=%d\n",L_Motor,R_Motor,P_Motor,ARMED);
  #endif
}

//Flight Control Modes (MODE)
//--------------------------
bool Flight_System::MODE_Init(){

  return true;
}
void Flight_System::MODE_Update(){
  //Obtain Flight Orders
  switch(flight_mode){
    case MODE_MANUAL:
      L_Motor = comms->MAVLINK_manual_control.z - comms->MAVLINK_manual_control.y;
      R_Motor = comms->MAVLINK_manual_control.z + comms->MAVLINK_manual_control.y;
      P_Motor = comms->MAVLINK_manual_control.x;
      break;
    case MODE_HOLD_ALT:
      break;
    case MODE_STABILIZE:
      break;
    case MODE_AUTO:
      break;
    case MODE_CIRCLE:
      break;
    case MODE_DRIFT:
      break;
    case MODE_GUIDED:
      break;
    case MODE_LAND:
      break;
    case MODE_TAKEOFF:
      break;
    case MODE_HOLD_POS:
      break;
  }
  //Set Control Limits
  L_Motor = constrain(L_Motor,PWM_MIN,PWM_MAX);
  R_Motor = constrain(R_Motor,PWM_MIN,PWM_MAX);
  P_Motor = constrain(P_Motor,PWM_MIN,PWM_MAX);

  #ifdef MODE_Debug
    Serial.print("todo");
  #endif
}

//Flight System Class 
//--------------------------
void Flight_System::Init(){
  if(!MOTOR_Init()) Serial.println("Error Initializing Flight->Motor");
  if(!MODE_Init())  Serial.println("Error Initializing Flight->Mode");
}
void Flight_System::Update(){
  MOTOR_Update();
  MODE_Update();
}
