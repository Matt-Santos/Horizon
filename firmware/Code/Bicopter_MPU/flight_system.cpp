//Bicopter Flight System
//Written by Matthew Santos

#include "flight_system.h"
#include "sensor_system.h"
#include "comms_system.h"

//Debug Flags
#define MOTOR_Debug
#define MODE_Debug

//I2C Interface
#define I2C_SDA         26      //I2C Data Pin
#define I2C_SCL         27      //I2C Clock Pin
#define I2C_Freq        400000  //[Hz] I2C Clock Frequency

//Motor Settings (MOTOR)
#define PCA9685_Addrs     0x40  //PCA9685 I2C Address
#define Motor_OE_Pin      32    //PCA9685 Output Enable Pin
#define PWM_Frequency     50    //[Hz] PWM Output Frequency
#define L_Motor_Chan      0     //Left  Motor Output Channel
#define R_Motor_Chan      1     //Right Motor Output Channel
#define P_Motor_ChanE     2     //Pitch Motor Enable
#define P_Motor_ChanA1    4     //Pitch Motor Output Channel A1
#define P_Motor_ChanA2    5     //Pitch Motor Output Channel A2
#define P_Motor_ChanB1    7     //Pitch Motor Output Channel B1
#define P_Motor_ChanB2    6     //Pitch Motor Output Channel B2
#define P_Motor_Tol       1     //[%] Pitch Motor Control Tolerene
#define P_Motor_Count     500   //Number of Pulses to travel Full Pitch Motor Range
#define PWM_MIN           5     //Minimum PWM Output
#define PWM_MAX           10    //Maximum PWM Output
#define P_Motor_MIN       5     //Minimum Pitch Motor Position
#define P_Motor_MAX       95    //Maximum Pitch Motor Position

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
  motor.setOutputsLowWhenDisabled();
  motor.disableOutputs(Motor_OE_Pin);
  motor.setOutputsInverted();
  // motor.setOutputsNotInverted();
  //Set Pitch Motor to Initially Disabled
  motor.setChannelDutyCycle(P_Motor_ChanE,0);
  //Calibrate Motors
  MOTOR_ESC_Calibrate();
  MOTOR_Pitch_Calibrate();
  return success;
}
void Flight_System::MOTOR_ESC_Calibrate(){
  //Calibrate ESC Throttle Max
  motor.setChannelDutyCycle(L_Motor_Chan,PWM_MAX);
  motor.setChannelDutyCycle(R_Motor_Chan,PWM_MAX);
  motor.enableOutputs(Motor_OE_Pin);
  delay(4000);  //Wait for two beeps
  //Calibrate ESC Throttle Min
  motor.setChannelDutyCycle(L_Motor_Chan,PWM_MIN);
  motor.setChannelDutyCycle(R_Motor_Chan,PWM_MIN);
  //Shutdown ESC
  delay(3000);
}
void Flight_System::MOTOR_Pitch_Calibrate(){
  //Force Pitch Motor to Zero Point
  motor.setChannelDutyCycle(P_Motor_ChanE,100);
  int count = 0;
  while(count<P_Motor_Count){
    count += MOTOR_Pitch_Control(100 + P_Motor_Tol);
  }
  //Set Initial Pitch Position to Midpoint
  count = 0;
  while(count<(P_Motor_Count/2)){
    count += MOTOR_Pitch_Control(50);
  }
  //Disable Pitch Motor
  motor.setChannelDutyCycle(P_Motor_ChanE,0);
}
bool Flight_System::MOTOR_Pitch_Control(float target){
  static float position = 50;     //[%] Motor Position as % of Range
  static unsigned long last;      //[us] Time between Function Calls
  static bool enabled = false;
  static uint8_t index = 0;

  //Limit Servo RotationSpeed
  if(micros()-last < 10000) return false;

  //Update Direction
  bool forwards = position < target;

  //Enable Control
  if(position < target - P_Motor_Tol || position > target + P_Motor_Tol){
    if(!enabled){
      motor.setChannelDutyCycle(P_Motor_ChanE,100);
      enabled = true;
    }
    //Update Position
    position -= 100.0/P_Motor_Count*(1-2*forwards);
    position = constrain(position,0.0,100.0);
  }
  else{
    motor.setChannelDutyCycle(P_Motor_ChanE,0);
    enabled = false;
  }

  //Update State
  index = (index+1) % 8;
  switch(index){
    case 0:
      if(forwards){
        motor.setChannelDutyCycle(P_Motor_ChanB2,0);
      }
      else{
        motor.setChannelDutyCycle(P_Motor_ChanB1,0);
      }
      break;
    case 1:
      if(forwards){
        motor.setChannelDutyCycle(P_Motor_ChanB1,100);
      }
      else{
        motor.setChannelDutyCycle(P_Motor_ChanB2,100);
      }
      break;
    case 2:
      motor.setChannelDutyCycle(P_Motor_ChanA1,0);
      break;
    case 3:
      motor.setChannelDutyCycle(P_Motor_ChanA2,100);
      break;
    case 4:
      if(forwards){
        motor.setChannelDutyCycle(P_Motor_ChanB1,0);
      }
      else{
        motor.setChannelDutyCycle(P_Motor_ChanB2,0);
      }
      break;
    case 5:
      if(forwards){
        motor.setChannelDutyCycle(P_Motor_ChanB2,100);
      }
      else{
        motor.setChannelDutyCycle(P_Motor_ChanB1,100);
      }
      break;
    case 6:
      motor.setChannelDutyCycle(P_Motor_ChanA2,0);
      break;
    case 7:
      motor.setChannelDutyCycle(P_Motor_ChanA1,100);
      break;
  }
  #ifdef MOTOR_Debug
    Serial.printf("Pitch_Position:%f\n",position);
  #endif
  last = micros();
  return true;
}
void Flight_System::MOTOR_Update(){
  if (ARMED){
    //Update Motor Positions
    motor.setChannelDutyCycle(L_Motor_Chan,L_Motor);
    motor.setChannelDutyCycle(R_Motor_Chan,R_Motor);
    MOTOR_Pitch_Control(P_Motor);
  }
  else{
    //Set ESC Throttle to Zero
    motor.setChannelDutyCycle(L_Motor_Chan,PWM_MIN);
    motor.setChannelDutyCycle(R_Motor_Chan,PWM_MIN);
  }
  
  #ifdef MOTOR_Debug
    Serial.printf("L:%f,R:%f,P:%f,ARM:%d\n",L_Motor,R_Motor,P_Motor,ARMED);
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
      L_Motor = ((comms->MAVLINK_manual_control.z/10.0)-(comms->MAVLINK_manual_control.y/10.0))*(PWM_MAX-PWM_MIN)/100.0+PWM_MIN;
      R_Motor = ((comms->MAVLINK_manual_control.z/10.0)+(comms->MAVLINK_manual_control.y/10.0))*(PWM_MAX-PWM_MIN)/100.0+PWM_MIN;
      P_Motor = (comms->MAVLINK_manual_control.x/20.0+50);  //[0 to 100]
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
  P_Motor = constrain(P_Motor,P_Motor_MIN,P_Motor_MAX);

  #ifdef MODE_Debug
    Serial.printf("x:%d,",comms->MAVLINK_manual_control.x);
    Serial.printf("y:%d,",comms->MAVLINK_manual_control.y);
    Serial.printf("z:%d\n",comms->MAVLINK_manual_control.z);
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
