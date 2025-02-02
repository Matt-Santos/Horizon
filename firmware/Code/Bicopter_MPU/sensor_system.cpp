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
  //Measure for 2 seconds to correct for filter delay
  for (uint16_t i=0;i<200;i++){
    IMU_ISR_Flag = true;
    IMU_Update();
    delay(10);
  }
  //Perform Calibration Sampling
  float w_dot_cal[3] = {0};
  float x_ddot_cal[3] = {0};
  for (uint16_t i=0;i<storage->Sensor.IMU_Cal_Samples;i++){
    IMU_ISR_Flag = true;
    IMU_Update();
    for (uint8_t i=0;i<3;i++){
      w_dot_cal[i]  += w_dot[i] /((float) storage->Sensor.IMU_Cal_Samples);
      x_ddot_cal[i] += x_ddot[i]/((float) storage->Sensor.IMU_Cal_Samples);
    }
    delay(10);
  }
  for (uint8_t i=0;i<3;i++){
    w_dot_offset[i]  = w_dot_cal[i];
    x_ddot_offset[i] = x_ddot_cal[i];
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
  uint8_t IMU_Accel_Reg, IMU_Gyro_Reg;
  switch(storage->Sensor.accel_range){
    case 2:
      IMU_Accel_Reg = 0x00;
      break;
    case 4:
      IMU_Accel_Reg = 0x08;
      break;
    case 8:
      IMU_Accel_Reg = 0x10;
      break;
    default:  //also handles 16
      IMU_Accel_Reg = 0x18;
      break;
  }
  switch(storage->Sensor.gyro_range){
    case 250:
      IMU_Gyro_Reg = 0x00;
      break;
    case 500:
      IMU_Gyro_Reg = 0x08;
      break;
    case 1000:
      IMU_Gyro_Reg = 0x10;
      break;
    default:  //also handles 5000
      IMU_Gyro_Reg = 0x18;
      break;
  }
  uint8_t IMUsettings[14] = {
    0x19,0x00,            //SMPLRT_DIV
    0x1a,0x01,            //Config
    0x1b,IMU_Gyro_Reg,    //Gyro
    0x1c,IMU_Accel_Reg,   //Accel
    0x6b,0x01,            //PW1
    0x37,0xd0,            //Int Pin Config
    0x38,0x01             //Enable Interrupt
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
void Sensor_System::IMU_Gyro_Filter(){
  //Check if not Accelerating
  //float upperlimit = 2;   //move to storage
  //float lowerlimit = 0.5; //move to storage
  //float x_ddot_mag = abs(x_ddot[0])+abs(x_ddot[1])+abs(x_ddot[2]-1.0);
  //if (x_ddot_mag < upperlimit && x_ddot_mag > lowerlimit){
    //Perform Filtering
    float w_acc[3] = {
      atan2f(x_ddot[1],x_ddot[2]-1.0),  //roll
      atan2f(x_ddot[0],x_ddot[2]-1.0),  //pitch
      0
    };
    w[0] = w[0]*0.99 + w_acc[0]*0.01;   //roll
    w[1] = w[1]*0.99 + w_acc[1]*0.01;   //pitch
    //yaw (no absolute reference at the moment)
  //}
}
void Sensor_System::IMU_Accel_Filter(){
  static float buff[3] = {0};
  float K_decay = 0.8;
  buff[0] += (1-K_decay)*(x_ddot[0]-buff[0]);
  buff[1] += (1-K_decay)*(x_ddot[1]-buff[1]);
  buff[2] += (1-K_decay)*(x_ddot[2]-buff[2]);
  x_ddot[0] = buff[0];
  x_ddot[1] = buff[1];
  x_ddot[2] = buff[2];
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
  x_ddot[0] = (raw_data[0]/16384.0)*9.807; //[m/s^2]
  x_ddot[1] = (raw_data[1]/16384.0)*9.807; //[m/s^2]
  x_ddot[2] = (raw_data[2]/16384.0)*9.807; //[m/s^2]
  T = (raw_data[3] + 12420.2)/340.0;       //[C]
  w_dot[0] = (raw_data[4]/131.0)*(PI/180); //[rad/s]
  w_dot[1] = (raw_data[5]/131.0)*(PI/180); //[rad/s]
  w_dot[2] = (raw_data[6]/131.0)*(PI/180); //[rad/s]
  //Calibrated Values
  x_ddot[0] -= x_ddot_offset[0]; //[m/s^2]
  x_ddot[1] -= x_ddot_offset[1]; //[m/s^2]
  x_ddot[2] -= x_ddot_offset[2]; //[m/s^2]
  w_dot[0] -= w_dot_offset[0];   //[rad/s]
  w_dot[1] -= w_dot_offset[1];   //[rad/s]
  w_dot[2] -= w_dot_offset[2];   //[rad/s]
  //Sensor Filtering
  IMU_Gyro_Filter();
  IMU_Accel_Filter();
  //Inferred/Integrated Values
  static unsigned long last_time;
  float interval = (millis()-last_time)*0.001;
  w[0] += w_dot[0]*interval;      //[rad]
  w[1] += w_dot[1]*interval;      //[rad]
  w[2] += w_dot[2]*interval;      //[rad]
  
  x_dot[0] += x_ddot[0]*interval; //[m/s]
  x_dot[1] += x_ddot[1]*interval; //[m/s]
  x_dot[2] += x_ddot[2]*interval; //[m/s]
  last_time = millis();
  IMU_ISR_Flag = false;       //Reset Interrupt Flag

  Serial.print("w_x:");Serial.print(w[0]*(180.0/PI));Serial.print(",");
  Serial.print("w_y:");Serial.print(w[1]*(180.0/PI));Serial.print(",");
  Serial.print("w_z:");Serial.print(w[2]*(180.0/PI));Serial.print(",");
  //Serial.print("T:");Serial.print(T);Serial.print(",");
  Serial.print("wd_x:");Serial.print(w_dot[0]);Serial.print(",");
  Serial.print("wd_y:");Serial.print(w_dot[1]);Serial.print(",");
  Serial.print("wd_z:");Serial.print(w_dot[2]);Serial.print(",");
  //Serial.print("a_x:");Serial.print(x_ddot[0]);Serial.print(",");
  //Serial.print("a_y:");Serial.print(x_ddot[1]);Serial.print(",");
  //Serial.print("a_z:");Serial.print(x_ddot[2]);
  Serial.print("\n");

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
  BAT_Update();
}
