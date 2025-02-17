//Bicopter Flight System
//Written by Matthew Santos

#include "flight_system.h"
#include "storage_system.h"
#include "sensor_system.h"
#include "comms_system.h"

//Debug Flags
#define MOTOR_Debug
// #define MODE_Debug

//I2C Interface
#define I2C_SDA         26      //I2C Data Pin
#define I2C_SCL         27      //I2C Clock Pin
#define I2C_Freq        400000  //[Hz] I2C Clock Frequency

//Motor Settings (MOTOR)
#define PCA9685_Addrs     0x40  //PCA9685 I2C Address
#define Motor_OE_Pin      32    //PCA9685 Output Enable Pin
#define PWM_Frequency     400   //[Hz] PWM Output Frequency
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
  for(int i=0;i<P_Motor_Count;i++){
    while(!MOTOR_Pitch_Control(100+P_Motor_Tol)){
      // delay(1);
    }
  }
  //Set Initial Target Pitch Position to Midpoint
  P_Motor = 50.0;
}
bool Flight_System::MOTOR_Pitch_Control(float target){
  static float position = 50;     //[%] Motor Position as % of Range
  static long last;               //[us] Time between Function Calls
  static uint8_t index = 0;       //Current Servo Position State
  float udelay = 100;            //[us] Update Delay

  //Rotation Speed Control [us]
  if(micros()-last < udelay) return false;
  //Update Servo State
  switch(index){
    case 0:
      motor.setChannelDutyCycle(P_Motor_ChanA1,100);
      motor.setChannelDutyCycle(P_Motor_ChanA2,0);
      motor.setChannelDutyCycle(P_Motor_ChanB1,0);
      motor.setChannelDutyCycle(P_Motor_ChanB2,0);
      break;
    case 1:
      motor.setChannelDutyCycle(P_Motor_ChanA1,100);
      motor.setChannelDutyCycle(P_Motor_ChanA2,0);
      motor.setChannelDutyCycle(P_Motor_ChanB1,100);
      motor.setChannelDutyCycle(P_Motor_ChanB2,0);
      break;
    case 2:
      motor.setChannelDutyCycle(P_Motor_ChanA1,0);
      motor.setChannelDutyCycle(P_Motor_ChanA2,0);
      motor.setChannelDutyCycle(P_Motor_ChanB1,100);
      motor.setChannelDutyCycle(P_Motor_ChanB2,0);
      break;
    case 3:
      motor.setChannelDutyCycle(P_Motor_ChanA1,0);
      motor.setChannelDutyCycle(P_Motor_ChanA2,100);
      motor.setChannelDutyCycle(P_Motor_ChanB1,100);
      motor.setChannelDutyCycle(P_Motor_ChanB2,0);
      break;
    case 4:
      motor.setChannelDutyCycle(P_Motor_ChanA1,0);
      motor.setChannelDutyCycle(P_Motor_ChanA2,100);
      motor.setChannelDutyCycle(P_Motor_ChanB1,0);
      motor.setChannelDutyCycle(P_Motor_ChanB2,0);
      break;
    case 5:
      motor.setChannelDutyCycle(P_Motor_ChanA1,0);
      motor.setChannelDutyCycle(P_Motor_ChanA2,100);
      motor.setChannelDutyCycle(P_Motor_ChanB1,0);
      motor.setChannelDutyCycle(P_Motor_ChanB2,100);
      break;
    case 6:
      motor.setChannelDutyCycle(P_Motor_ChanA1,0);
      motor.setChannelDutyCycle(P_Motor_ChanA2,0);
      motor.setChannelDutyCycle(P_Motor_ChanB1,0);
      motor.setChannelDutyCycle(P_Motor_ChanB2,100);
      break;
    case 7:
      motor.setChannelDutyCycle(P_Motor_ChanA1,100);
      motor.setChannelDutyCycle(P_Motor_ChanA2,0);
      motor.setChannelDutyCycle(P_Motor_ChanB1,0);
      motor.setChannelDutyCycle(P_Motor_ChanB2,100);
      break;
  }
  //Pitch Direction and Enable Control
  if(position < target - P_Motor_Tol){
    index = (index+1) % 8;
    position += 100.0/P_Motor_Count;
    motor.setChannelDutyCycle(P_Motor_ChanE,100.0,0.0);
  }
  else if(position > target + P_Motor_Tol){
    index = (index-1) % 8;
    position -= 100.0/P_Motor_Count;
    motor.setChannelDutyCycle(P_Motor_ChanE,100.0,0.0);
  }
  else{
    motor.setChannelDutyCycle(P_Motor_ChanE,0.0,0.0);
  }
  //Update Function Values
  position = constrain(position,0.0,100.0);
  last = micros();

  Serial.printf("position:%f\n",position);
  return true;
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
      L_Motor = (comms->MAVLINK_manual_control.z/10.0+100) - (comms->MAVLINK_manual_control.y/10.0+100);
      R_Motor = (comms->MAVLINK_manual_control.z/10.0+100) + (comms->MAVLINK_manual_control.y/10.0+100);
      P_Motor = (comms->MAVLINK_manual_control.x/20.0+50);
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
    Serial.printf("x:%f,",comms->MAVLINK_manual_control.x);
    Serial.printf("y:%f,",comms->MAVLINK_manual_control.y);
    Serial.printf("z:%f\n",comms->MAVLINK_manual_control.z);
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
