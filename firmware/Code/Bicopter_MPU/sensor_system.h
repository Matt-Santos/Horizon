//Bicopter Sensor System
//Written by Matthew Santos

/* Author Notes
- this system includes these submodules
  - Inertia Measurement Unit (IMU) using the MPU6050
  - Battery Monitor (BAT) using the builtin ADC
  - Camera Driver (CAM) using the OV2640
- IMU module is interrupt controlled and maintains attitude information
- BAT module provides periodic update of battery voltage
- CAM module is a driver that other systems can use to access the camera
*/

#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

#include "Arduino.h"
#include <Wire.h>
#include "esp_camera.h"

class Sensor_System {

  //Inertia Measurement Unit (IMU)
  //--------------------------
  public:
    //IMU Sensor Data
    float x[3]      = {0,0,0};        //[m] Relative Position
    float x_dot[3]  = {0,0,0};        //[m/s] Linear Velocity
    float x_ddot[3] = {0,0,0};        //[m/s^2] Linear Acceleration
    float w[3]      = {0,0,0};        //[rad] Angular Position
    float w_dot[3]  = {0,0,0};        //[rad/s] Angular Velocity
    float T         = 0;              //[C] Temperature
    //IMU Calibration Values
    float w_dot_offset[3] = {0,0,0};  //[rad/s] Angular Acceleration Offset
    float x_ddot_offset[3] = {0,0,0}; //[m/s^2] Linear Acceleration Offset
    //IMU Interrupt Flag
    static bool IMU_ISR_Flag;
    //IMU Functions
    void IMU_Calibrate(); //Calibrates IMU offsets
  private:
    bool IMU_Init();
    void IMU_Filter();    //Complementary Filter for Gyro Drift Control of pitch and roll
    void IMU_Update();
    static void IRAM_ATTR IMU_INTERRUPT();
  
  //Battery Monitor (BAT)
  //--------------------------
  public:
    //BAT Data
    uint16_t BAT_Level = 0;           //[] Battery Voltage
  private:
    //BAT Functions
    bool BAT_Init();
    void BAT_Update();
  
  //Camera Driver (BAT)
  //--------------------------
  public:
    //CAM Data
    camera_fb_t *fb;                  //Camera Frame Buffer
    //CAM Functions
    //---------------------
    void CAM_GetFrame();  //Get Camera Frame
    void CAM_FreeFrame(); //Free Camera Frame Memory
  private:
    bool CAM_Init();

  //Sensor System Class 
  //--------------------------
  public:
    void Init();                  //Sensor Class Initializer
    void Update();                //Update all Sensors
    
};

#endif