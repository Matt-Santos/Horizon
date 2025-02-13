//Bicopter Sensor System
//Written by Matthew Santos

/* Author Notes
- this system includes these submodules
  - Inertia Measurement Unit (IMU) using the MPU6050
  - Global Position System (GPS)
  - Barometric Pressure Sensor (BAR)
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
#include <MPU6050.h>
#include <TinyGPS.h>
#include <BMP180I2C.h>
#include "esp_camera.h"

class Sensor_System {

  //Inertia Measurement Unit (IMU)
  //--------------------------
  public:
    float x_dot[3] = {0,0,0};         //[m/s] Linear Velocity (inferred)
    float x_ddot[3] = {0,0,0};        //[m/s^2] Linear Acceleration (measured)
    float w[3] = {0,0,0};             //[rad] Angular Position (inferred)
    float w_dot[3]  = {0,0,0};        //[rad/s] Angular Velocity (measured)
    float x_ddot_offset[3] = {0,0,0}; //[m/s^2] Linear Acceleration Offset
    float w_dot_offset[3] = {0,0,0};  //[rad/s] Angular Velocity Offset

    // float ferraris_offsets = {0};
    // bool Ferraris_Calibration();      //Perform Manual Orientation Calibration

  private:
    MPU6050 imu;
    bool IMU_Init();
    void IMU_Filter();                //Various stability improvments
    void IMU_Update();
  
  //Global Position System (GPS)
  //--------------------------
  public:
    int32_t latitude = 0;             //[degE7] Latitude
    int32_t longitude = 0;            //[degE7] Longitude
    int32_t altitude = 0;             //[mm]  Altitude (MSL)
    TinyGPS gps;
  private:
    bool GPS_Init();
    void GPS_Update();

  //Barometric Pressure Sensor (BAR)
  //--------------------------
  public:
    float temp = 0;                   //[C] Ambient Temperature 
    float pressure = 0;               //[Pa] Local Pressure
  private:
    BMP180I2C bar = BMP180I2C(0x77);
    bool BAR_Init();
    void BAR_Update();

  //Battery Monitor (BAT)
  //--------------------------
  public:
    uint16_t BAT_Level = 0;           //[] Battery Voltage
  private:
    bool BAT_Init();
    void BAT_Update();
  
  //Camera Driver (BAT)
  //--------------------------
  public:
    camera_fb_t *fb;                  //Camera Frame Buffer
    void CAM_GetFrame();              //Get Camera Frame
    void CAM_FreeFrame();             //Free Camera Frame Memory
  private:
    bool CAM_Init();

  //Sensor System Class 
  //--------------------------
  public:
    void Init();                      //Sensor Class Initializer
    void Update();                    //Update all Sensors
    
};

#endif