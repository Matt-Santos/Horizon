//Bicopter Flight System
//Written by Matthew Santos

#include "flight_system.h"
#include "storage_system.h"
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
#define PWM_Frequency     400   //[Hz] PWM Output Frequency
#define L_Motor_Chan      0     //Left  Motor Output Channel
#define R_Motor_Chan      1     //Right Motor Output Channel
#define P_Motor_Chan      2     //Pitch Motor Output Channel
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
  // motor.setOutputsToTotemPole();
  // motor.setOutputsToOpenDrain();
  // motor.setOutputsHighWhenDisabled();
  // motor.setOutputsHighImpedanceWhenDisabled();
  MOTOR_ESC_Calibrate();
  MOTOR_Pitch_Calibrate();
  return success;
}
void Flight_System::MOTOR_ESC_Calibrate(){
  //Throttle Range Calibration Procedure
  motor.setChannelDutyCycle(L_Motor_Chan,95.0);
  motor.setChannelDutyCycle(R_Motor_Chan,95.0);
  delay(3000);  //Wait for two beeps
  motor.setChannelDutyCycle(L_Motor_Chan,5.0);
  motor.setChannelDutyCycle(R_Motor_Chan,5.0);
  delay(3000);
}
void Flight_System::MOTOR_Pitch_Calibrate(){
  //Set Speed to Minimal

  //Mark Zero Endpoint

  //Mark 100% Endpoint

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
  motor.setChannelDutyCycle(P_Motor_Chan,P_Motor);

  #ifdef MOTOR_Debug
    Serial.printf("Motors L: %f , R: %f , P: %f, ARM=%d\n",L_Motor,R_Motor,P_Motor,ARMED);
  #endif
}

//Flight Control Modes (MODE)
//--------------------------
bool MODE_Init(){

  return true;
}
void MODE_Update(){
  //Obtain Flight Orders
  switch(flight_mode){
    case MODE_MANUAL:
      L_Motor = (512.0*comms->MAVLINK_manual_control.z)/125.0-(512.0*comms->MAVLINK_manual_control.y)/625.0;;
      R_Motor = (512.0*comms->MAVLINK_manual_control.z)/125.0+(512.0*comms->MAVLINK_manual_control.y)/625.0;
      // P_Motor = ;
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

  #ifdef MODE_Debug
    // Serial.print("x_in:");
    // Serial.print(comms->MAVLINK_manual_control.x);Serial.print(",");
    // Serial.print("y_in:");
    // Serial.print(comms->MAVLINK_manual_control.y);Serial.print(",");
    // Serial.print("z_in:");
    // Serial.print(comms->MAVLINK_manual_control.z);Serial.print(",");
    // Serial.print("L:");
    // Serial.print(L_Motor);Serial.print(",");
    // Serial.print("R:");
    // Serial.println(R_Motor);
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
