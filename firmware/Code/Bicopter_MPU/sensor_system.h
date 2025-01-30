//Bicopter Sensor System
//Written by Matthew Santos
#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

//#include "Arduino.h"
#include <Wire.h>
#include "esp_camera.h"
#include "storage_system.h"

class Sensor_Class {
  public:
    Storage_System& storage = Storage_System::getInstance();
    //Attitude Data
    float x[3]      = {0,0,0};        //[m] Relative Position
    float x_dot[3]  = {0,0,0};        //[m/s] Linear Velocity
    float x_ddot[3] = {0,0,0};        //[m/s^2] Linear Acceleration
    float w[3]      = {0,0,0};        //[rad] Angular Position
    float w_dot[3]  = {0,0,0};        //[rad/s] Angular Velocity
    //Auxillary Data
    uint16_t BAT_Level = 0;           //[] Battery Voltage
    float T = 0;                      //[C] Temperature
    //Calibration Values
    float w_dot_offset[3] = {0,0,0};  //[rad/s] Angular Acceleration Offset
    float x_ddot_offset[3] = {0,0,0}; //[m/s^2] Acceleration Offset
    //Camera Data
    camera_fb_t *fb;                  //Camera Frame Buffer
    //Support Varriables
    static bool IMU_ISR_Flag;

    //Public Functions
    //---------------------
    void Update();
    void Init();
    void IMU_Calibrate(); //Calibrates IMU offsets
    void CAM_GetFrame();  //Get Camera Frame
    void CAM_FreeFrame(); //Free Camera Frame Memory
  private:
    //Private Functions
    //---------------------
    static void IRAM_ATTR IMU_INTERRUPT();
    bool IMU_Init();
    bool BAT_Init();
    bool CAM_Init();
    void IMU_Update();
    void BAT_Update();
    void IMU_Filter();  //Complementary Filter for Gyro Drift Control of pitch and roll
};

extern Sensor_Class sensor;

#endif