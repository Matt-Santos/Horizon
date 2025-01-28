//Bicopter Sensor System
//Written by Matthew Santos
#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

#include "Arduino.h"
#include <Wire.h>
#include "esp_camera.h"

//IMU Settings (MPU6050)
#define MPU6050_ADDR    0x68    //I2C Address
#define IMU_I2C_Freq    400000  //I2C Frequency
#define IMU_Cal_Samples 100     //Number of Samples used for calibration
#define IMU_SDA         26      //I2C Data Pin
#define IMU_SCL         27      //I2C Clock Pin
#define IMU_INT         1       //Interrupt Pin

//BATTERY Settings (ADC)
#define BAT_Period      1       //[ms] Time between Measurements
#define BAT_SENSE       13      //GPIO Battery Pin

//Camera Settings (OV2640)
#define CAM_Size FRAMESIZE_QVGA //320x240
// #define CAM_Size FRAMESIZE_CIF   //352x288
// #define CAM_Size FRAMESIZE_VGA   //640x480
// #define CAM_Size FRAMESIZE_HD    //960x540
// #define CAM_Size FRAMESIZE_XGA   //1024x768
// #define CAM_Size FRAMESIZE_SXGA  //1280x1024
#define CAMERA_MODEL_WROVER_KIT
#define CAM_Y2          4
#define CAM_Y3          5
#define CAM_Y4          18
#define CAM_Y5          19
#define CAM_Y6          36
#define CAM_Y7          39
#define CAM_Y8          34
#define CAM_Y9          35
#define CAM_XCLK        21
#define CAM_PCLK        22
#define CAM_HREF        23
#define CAM_SIOD        26
#define CAM_SIOC        27
#define CAM_VSYNC       25
#define CAM_PWDN        -1
#define CAM_RESET       -1

class Sensor_Class {
  public:
    //Real Sensors
    uint16_t BAT_Level = 0;           //[] Battery Voltage
    float w_dot[3] = {0,0,0};         //[rad/s] Angular Velocity
    float x_ddot[3] = {0,0,0};        //[m/s^2] Acceleration
    float T = 0;                      //[C] Temperature
    //Inferred Sensors
    float w[3] = {0,0,0};             //[rad] Angular Rotation
    float x_dot[3] = {0,0,0};         //[m/s] Velocity
    //Calibration Values
    float w_dot_offset[3] = {0,0,0};  //[rad/s] Angular Acceleration Offset
    float x_ddot_offset[3] = {0,0,0}; //[m/s^2] Acceleration Offset
    //Support Varriables
    static bool IMU_ISR_Flag;

    //Public Functions
    //---------------------
    void Update();
    void Init();
    void IMU_Calibrate();
  private:
    //Private Functions
    //---------------------
    static void IRAM_ATTR IMU_INTERRUPT();
    bool IMU_Init();
    bool BAT_Init();
    bool CAM_Init();
    void IMU_Update();
    void BAT_Update();
    void CAM_Update();
};

extern Sensor_Class sensor;

#endif