//Bicopter Sensor System
//Written by Matthew Santos

#include "sensor_system.h"
#include "storage_system.h"

//IMU Settings (MPU6050)
#define MPU6050_ADDR    0x68    //I2C Address
#define IMU_SDA         26      //I2C Data Pin
#define IMU_SCL         27      //I2C Clock Pin
#define IMU_INT         1       //Interrupt Pin

//BATTERY Settings (ADC)
#define BAT_SENSE       13      //GPIO Battery Pin

//Camera Settings (OV2640)
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

extern Storage_System *storage;

//Inertia Measurement Unit (IMU)
//--------------------------
bool Sensor_System::IMU_ISR_Flag;
void Sensor_System::IMU_Calibrate(){
  //Reset Calibration Factors
  for (uint8_t i=0;i<3;i++){
    w_dot_offset[i] = 0;
    x_ddot_offset[i] = 0;
  }
  //Perform Calibration Sampling
  for (uint16_t i=0;i<storage->Sensor.IMU_Cal_Samples;i++){
    IMU_ISR_Flag = true;
    IMU_Update();
    for (uint8_t i=0;i<3;i++){
      w_dot_offset[i] += w_dot[i]/((float) storage->Sensor.IMU_Cal_Samples);
      x_ddot_offset[i] += x_ddot[i]/((float) storage->Sensor.IMU_Cal_Samples);
    }
    delay(50);
  }
  //Reset Inferred Sensor Data
  for (uint8_t i=0;i<3;i++){
    w[i] = 0;
    x_dot[i] = 0;
  }
}
bool Sensor_System::IMU_Init(){
  bool success = true;
  //Configure IMU Pins (MPU6050)
  pinMode(IMU_SDA,INPUT_PULLUP);
  pinMode(IMU_SCL,INPUT_PULLUP);
  // pinMode(IMU_INT,INPUT_PULLUP); //(This apparently halts the code...)
  //Configure I2C Interface
  success &= Wire.begin(IMU_SDA,IMU_SCL,storage->Sensor.I2C_Freq);
  //Configure IMU Settings (MPU6050)
  uint8_t IMUsettings[14] = {
    0x19,0x00,  //SMPLRT_DIV
    0x1a,0x00,  //Config
    0x1b,0x00,  //Gyro  (250deg/s)
    0x1c,0x08,  //Accel (4g)
    0x6b,0x01,  //PW1
    0x37,0xd0,  //Int Pin Config
    0x38,0x01   //Enable Interrupt
  };
  for (uint8_t i=0;i<sizeof(IMUsettings)/2;i++){
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(IMUsettings[0+2*i]);
    Wire.write(IMUsettings[1+2*i]);
    success &= (Wire.endTransmission() == 0);
  }
  //Perform IMU Startup Calibration
  IMU_Calibrate();
  attachInterrupt(IMU_INT,IMU_INTERRUPT,ONLOW);
  return success;
}
void Sensor_System::IMU_Filter(){
  //Check if not Accelerating
  // float upperlimit = 2;
  // float lowerlimit = 0.5;
  // float x_ddot_mag = abs(x_ddot[0])+abs(x_ddot[1])+abs(x_ddot[2]);
  // if (x_ddot_mag < upperlimit && x_ddot_mag > lowerlimit){
  //   //Perform Filtering
  //   float w_rel[x] = atan2f(x_ddot[0],x_ddot[1]);  //pitch
  //   float w_rel[x] = atan2f(x_ddot[0],x_ddot[1]);  //roll
  //   w[0] = w[0]*0.95 + w_rel[0]*0.05;
  //   w[1] = w[1]*0.95 + w_rel[1]*0.05;
  //   //roll
  //   //yaw (no absolute reference at the moment)

  //}
}
void Sensor_System::IMU_Update(){
  if (!IMU_ISR_Flag) return;  //Check Interrupt Flag
  //Get IMU Data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); //First Data Register
  Wire.endTransmission(false);
  uint8_t bytesReceived = Wire.requestFrom((int)MPU6050_ADDR,14);
  uint8_t buffer[bytesReceived];
  Wire.readBytes(buffer,bytesReceived);
  //Get RawValues
  int16_t raw_data[7];
  raw_data[0] = (buffer[0] << 8) | buffer[1];   //w_x
  raw_data[1] = (buffer[2] << 8) | buffer[3];   //w_y
  raw_data[2] = (buffer[4] << 8) | buffer[5];   //w_z
  raw_data[3] = (buffer[6] << 8) | buffer[7];   //T
  raw_data[4] = (buffer[8] << 8) | buffer[9];   //a_x
  raw_data[5] = (buffer[10] << 8) | buffer[11]; //a_y
  raw_data[6] = (buffer[12] << 8) | buffer[13]; //a_z
  //Measured Values
  w_dot[0] = (raw_data[0]/131.0)*(PI/180); //[rad/s]
  w_dot[1] = (raw_data[1]/131.0)*(PI/180); //[rad/s]
  w_dot[2] = (raw_data[2]/131.0)*(PI/180); //[rad/s]
  T = (raw_data[3] + 12420.2)/340.0;       //[C]
  x_ddot[0] = (raw_data[4]/16384.0)*9.807; //[m/s^2]
  x_ddot[1] = (raw_data[5]/16384.0)*9.807; //[m/s^2]
  x_ddot[2] = (raw_data[6]/16384.0)*9.807; //[m/s^2]
  //Calibrated Values
  w_dot[0] -= w_dot_offset[0];   //[rad/s]
  w_dot[1] -= w_dot_offset[1];   //[rad/s]
  w_dot[2] -= w_dot_offset[2];   //[rad/s]
  x_ddot[0] -= x_ddot_offset[0]; //[m/s^2]
  x_ddot[1] -= x_ddot_offset[1]; //[m/s^2]
  x_ddot[2] -= x_ddot_offset[2]; //[m/s^2]
  //Inferred Values
  static unsigned long last_time;
  float interval = (millis()-last_time)*0.001;
  w[0] += w_dot[0]*interval;      //[rad]
  w[1] += w_dot[1]*interval;      //[rad]
  w[2] += w_dot[2]*interval;      //[rad]
  IMU_Filter();

  x_dot[0] += x_ddot[0]*interval; //[m/s]
  x_dot[1] += x_ddot[1]*interval; //[m/s]
  x_dot[2] += x_ddot[2]*interval; //[m/s]
  last_time = millis();
  IMU_ISR_Flag = false;       //Reset Interrupt Flag
}
void IRAM_ATTR Sensor_System::IMU_INTERRUPT(){
  IMU_ISR_Flag = true;
}

//Battery Monitor (BAT)
//--------------------------
bool Sensor_System::BAT_Init(){
  //Configure Battery Monitoring
  pinMode(BAT_SENSE,INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  return true;
}
void Sensor_System::BAT_Update(){
  static int last_time;
  if (millis() - last_time < storage->Sensor.BAT_Period) return;
  BAT_Level = analogRead(BAT_SENSE);
  last_time = millis();
}

//Camera Driver (BAT)
//--------------------------
void Sensor_System::CAM_GetFrame(){
  //Update Camera Settings
  // s->set_framesize(s, (framesize_t) storage.Sensor.CAM_Size);
  // s->set_brightness(s, 1);   // up the brightness just a bit
  // s->set_saturation(s, -2);  // lower the saturation
  //Update Frame Data
  fb = esp_camera_fb_get();
}
void Sensor_System::CAM_FreeFrame(){
  esp_camera_fb_return(fb);
}
bool Sensor_System::CAM_Init(){
  bool success = true; 
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
  camera_config.pin_pwdn = CAM_PWDN;
  camera_config.pin_reset = CAM_RESET;
  camera_config.xclk_freq_hz = 20000000;
  camera_config.pixel_format = PIXFORMAT_JPEG;
  camera_config.frame_size = FRAMESIZE_96X96;
  camera_config.jpeg_quality = 10;
  camera_config.fb_count = 2;
  camera_config.ledc_channel = LEDC_CHANNEL_0;
  camera_config.ledc_timer = LEDC_TIMER_0;
  camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  camera_config.fb_location = CAMERA_FB_IN_PSRAM;
  esp_err_t err = esp_camera_init(&camera_config);
  success &= (err == ESP_OK);
  //Apply Default Settings
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, (framesize_t) storage->Sensor.CAM_Size);
  //s->set_brightness(s, 1);   // up the brightness just a bit
  //s->set_saturation(s, -1);  // lower the saturation
  // s->set_vflip(s, 1);
  // s->set_hmirror(s, 1);
  return success;
}

//Sensor System Class 
//--------------------------
void Sensor_System::Init(){
  if(!IMU_Init()) Serial.println("Error Initializing Sensor->IMU");
  if(!BAT_Init()) Serial.println("Error Initializing Sensor->BAT");
  if(!CAM_Init()) Serial.println("Error Initializing Sensor->CAM");
}
void Sensor_System::Update(){
  IMU_Update();
  Serial.printf("w_x = %f [rad] ",w[0]);
  Serial.printf("w_y = %f [rad] ",w[1]);
  Serial.printf("w_z = %f [rad]\n",w[2]);
  Serial.printf("wd_x = %f [rad/s] ",w_dot[0]);
  Serial.printf("wd_y = %f [rad/s] ",w_dot[1]);
  Serial.printf("wd_z = %f [rad/s]\n",w_dot[2]);
  BAT_Update();
}
