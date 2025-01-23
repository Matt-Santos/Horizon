// Bicopter Firmware
// Written by Matthew Santos

//Libraries
#include <WiFi.h>
#include <Wire.h>
#include <MAVLink_minimal.h>
#include "esp_camera.h"
#include "configs.h"
#include "communications.h"

//Todo
/*

Redirect Serial over Wifi Connection
Direct MAVLink to transmit over serial
Implement the MAVLink Message System

(These are Initialized at the moment)
-Network Connection 
-Motor Control System
-IMU Data Processing
-Battery Monitor
-Camera System
*/

//Global Varriables
uint16_t BAT_Level = 0;
uint32_t L_PWM = 0;
uint32_t R_PWM = 0;
struct imu_data {
  uint16_t x[3] = {0,0,0};
  uint16_t a[3] = {0,0,0};
  uint16_t T = 0;
} IMU_Data;

//Startup
void setup() {
  initialize();
  //Get Inertia Matrix
  //Start Control System
}

//Main Program Loop
void loop() {

  //Reconnect to wifi if needed

  //update motor controls (or put this in wifi update cmd)
  ledcWrite(Motor_PWM_L,L_PWM);
  ledcWrite(Motor_PWM_R,R_PWM);

  //Send Heartbeat Signal
}

//Support Functions
//------------------------
void IRAM_ATTR IMU_INTERRUPT(){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); //Select Data Register
  uint8_t bytesReceived = Wire.requestFrom((int)MPU6050_ADDR,14);
  uint8_t buffer[bytesReceived];
  Wire.readBytes(buffer,bytesReceived);
  Wire.endTransmission(true);
  if (bytesReceived == 14) {
    for (uint8_t i=0;i<3;i++) {
      IMU_Data.x[i] = buffer[2*i] << 8 | buffer[2*i+1];
      IMU_Data.a[i] = buffer[2*i+8] << 8 | buffer[2*i+9];
    }
    IMU_Data.T = buffer[6] << 8 | buffer[7];
  }
}

void ARDUINO_ISR_ATTR ADC_ISR(){
  adc_continuous_data_t *result;
  bool success = analogContinuousRead(&result, 10);
  if (success){
    BAT_Level = result->avg_read_raw;
  }
}

void initialize(void){

  //Setup Pins
  pinMode(Motor_PWM_L,OUTPUT);
  pinMode(Motor_PWM_R,OUTPUT);
  pinMode(Motor_PWM_P,OUTPUT);
  pinMode(IMU_SDA,INPUT_PULLUP);
  pinMode(IMU_SCL,INPUT_PULLUP);
  pinMode(IMU_INT,INPUT);
  pinMode(BAT_SENSE,INPUT);
  pinMode(EXTRA_PIN,INPUT);

  //Configure Serial Communications
  Serial.begin(115200,SERIAL_8N1);
  Serial.setTimeout(500);

  //Configure Motor PWM Settings (13bit @ 10kHz)
  bool success = true;
  success &= ledcSetClockSource((ledc_clk_cfg_t) 4); //APB clock
  success &= ledcAttach(Motor_PWM_L, 10000, 13);
  success &= ledcAttach(Motor_PWM_R, 10000, 13);
  success &= ledcAttach(Motor_PWM_P, 10000, 13);
  success &= ledcOutputInvert(Motor_PWM_L,true);
  success &= ledcOutputInvert(Motor_PWM_R,true);
  success &= ledcOutputInvert(Motor_PWM_P,true);
  success &= ledcWrite(Motor_PWM_L,0);
  success &= ledcWrite(Motor_PWM_R,0);
  success &= ledcWrite(Motor_PWM_P,0);
  if (!success){Serial.printf("Error Initializing PWM");}

  //Configure Network Settings
  IPAddress local_IP(WIFI_LOCAL_IP);
  IPAddress gateway(WIFI_GATEWAY);
  IPAddress subnet(WIFI_SUBNET);
  WiFi.useStaticBuffers(false);
  success &= WiFi.softAPsetHostname(WIFI_Hostname);
  success &= WiFi.softAPConfig(local_IP, gateway, subnet);
    //WiFi.onEvent(WiFiEvent);	//Assign Event Handler
  success &= WiFi.softAPdisconnect(true);
  success &= WiFi.softAP(WIFI_SSID,WIFI_PASS,WIFI_channel,WIFI_Hidden,WIFI_Connections);
  if (!success){Serial.printf("Error Initializing Wifi");}

  //Configure I2C Communications
  Wire.setTimeOut(500);
  success &= Wire.begin(IMU_SDA,IMU_SCL,400000);
  if (!success){Serial.printf("Error Initializing I2C");}

  //Configure IMU MPU6050
  uint8_t IMUsettings[] = {
    0x1a,0x00,  //Config
    0x1b,0x00,  //Gyro  (250deg/s)
    0x1c,0x08,  //Accel (4g)
    0x6b,0x01,  //PW1
    0x38,0x01   //Enable Interrupt
  };
  attachInterrupt(IMU_INT,IMU_INTERRUPT,RISING);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(IMUsettings,sizeof(IMUsettings));
  Wire.endTransmission(true);

  //Configure the Camera
  camera_config_t camera_config;
  camera_config.pin_d0 = CAM_Y2;
  camera_config.pin_d1 = CAM_Y3;
  camera_config.pin_d2 = CAM_Y4;
  camera_config.pin_d3 = CAM_Y5;
  camera_config.pin_d4 = CAM_Y6;
  camera_config.pin_d5 = CAM_Y7;
  camera_config.pin_d6 = CAM_Y8;
  camera_config.pin_d7 = CAM_Y9;
  camera_config.pin_xclk = CAM_XCLK;
  camera_config.pin_pclk = CAM_PCLK;
  camera_config.pin_vsync = CAM_VSYNC;
  camera_config.pin_href = CAM_HREF;
  camera_config.pin_sccb_sda = CAM_SIOD;
  camera_config.pin_sccb_scl = CAM_SIOC;
  camera_config.xclk_freq_hz = 20000000;
  camera_config.pixel_format = PIXFORMAT_JPEG;
  camera_config.frame_size = FRAMESIZE_UXGA;
  camera_config.jpeg_quality = 10;
  camera_config.fb_count = 2;
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK){Serial.printf("Error Initializing Camera");}

  //Configure the ADC for Battery Monitoring
  uint8_t ADC_Pins[] = {BAT_SENSE};
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_ATTENDB_MAX);
  success &= analogContinuous(ADC_Pins,1,ADC_SampleCount,ADC_SampleFreq,&ADC_ISR);
  success &= analogContinuousStart();
  if (!success){Serial.printf("Error Initializing ADC");}
}


