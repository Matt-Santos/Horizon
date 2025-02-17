//Bicopter Sensor System
//Written by Matthew Santos

#include "sensor_system.h"
#include "storage_system.h"

//Debug Flags
// #define IMU_Debug
// #define GPS_Debug
// #define BAR_Debug
// #define BAT_Debug

//I2C Interface
#define I2C_SDA         26      //I2C Data Pin
#define I2C_SCL         27      //I2C Clock Pin
#define I2C_Freq        400000  //[Hz] I2C Clock Frequency

//IMU Settings (MPU6050)
#define MPU6050_ADDR    0x68    //I2C Address
#define IMU_accel_range 2       //[g] (valid entries are 2,4,8,16)
#define IMU_gyro_range  250     //[deg/s] (valid entries are +/- 250,500,1000,2000)
#define IMU_Period      1       //[ms] Time between Battery Measurements
#define IMU_Gravity     9.807   //[m/s^2] Acceleration due to Gravity

//GPS Settings (G28U7FTTL)
//#define TX...

//BAR Settings (BMP180)
#define BAR_Mode        0       //Measurement Mode (0=LowPower,1=Standard,2=HighRes,3=UltraHighRes)
#define BMP180_ADDR     0x77    //I2C Address
#define BAR_Period      1000    //[ms] Time between Pressure Measurements
#define BAR_SLPressure  101325  //[Pa] Pressure at Sea Level

//BATTERY Settings (ADC)
#define BAT_SENSE       13      //GPIO Battery Pin
#define BAT_Period      1000    //[ms] Time between Battery Measurements

//Camera Settings (OV2640)
#define CAM_Size FRAMESIZE_96X96
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
bool Sensor_System::IMU_Init(){
  bool success = true;
  //Configure I2C Interface
  pinMode(I2C_SDA,INPUT_PULLUP);
  pinMode(I2C_SCL,INPUT_PULLUP);
  success &= Wire.begin(I2C_SDA,I2C_SCL,I2C_Freq);
  //Create IMU Object
  imu = MPU6050(MPU6050_ADDR,&Wire);
  //Configure IMU Settings (MPU6050)
  ACCEL_FS accel_range;
  GYRO_FS gyro_range;
  switch(IMU_accel_range){
    case 2:   //2 [g]
      accel_range = ACCEL_FS::A2G;
      break;
    case 4:   //4 [g]
      accel_range = ACCEL_FS::A4G;
      break;
    case 8:   //8 [g]
      accel_range = ACCEL_FS::A8G;
      break;
    default:  //16 [g]
      accel_range = ACCEL_FS::A16G;
  }
  switch(IMU_gyro_range){
    case 250:   //250 [rad/s]
      gyro_range = GYRO_FS::G250DPS;
      break;
    case 500:   //500 [rad/s]
      gyro_range = GYRO_FS::G500DPS;
      break;
    case 1000:  //1000 [rad/s]
      gyro_range = GYRO_FS::G1000DPS;
      break;
    default:    //2000 [rad/s]
      gyro_range = GYRO_FS::G2000DPS;
  }
  imu.initialize(accel_range,gyro_range);
  success &= imu.testConnection();
  if (success){
    imu.setRate(0);           //Set Sample Rate to Default
    imu.setDLPFMode(1);       //Set 1kHz Rate
    imu.setInterruptMode(0);  //Set Interrupt Pin LOW
    imu.setIntEnabled(0);     //Disable Interrupts
    //Perform Sensor Calibration
    imu.CalibrateGyro(15);
    imu.CalibrateAccel(15);
    w_dot_offset[0]  = imu.getXGyroOffset()*imu.get_gyro_resolution()*(PI/180.0);   //[rad/s]
    w_dot_offset[1]  = imu.getYGyroOffset()*imu.get_gyro_resolution()*(PI/180.0);   //[rad/s]
    w_dot_offset[2]  = imu.getZGyroOffset()*imu.get_gyro_resolution()*(PI/180.0);   //[rad/s]
    x_ddot_offset[0] = imu.getXAccelOffset()*imu.get_acce_resolution()*IMU_Gravity; //[m/s^2]
    x_ddot_offset[1] = imu.getYAccelOffset()*imu.get_acce_resolution()*IMU_Gravity; //[m/s^2]
    x_ddot_offset[2] = imu.getZAccelOffset()*imu.get_acce_resolution()*IMU_Gravity; //[m/s^2]
  }
  #ifdef IMU_Debug
    Serial.printf("getOTPBankValid(): %d\n",imu.getOTPBankValid());
    Serial.printf("getXGyroOffsetTC(): %d\n",imu.getXGyroOffsetTC());
    Serial.printf("getYGyroOffsetTC(): %d\n",imu.getYGyroOffsetTC());
    Serial.printf("getZGyroOffsetTC(): %d\n",imu.getZGyroOffsetTC());
    Serial.printf("getXFineGain(): %d\n",imu.getXFineGain());
    Serial.printf("getYFineGain(): %d\n",imu.getYFineGain());
    Serial.printf("getZFineGain(): %d\n",imu.getZFineGain());
    Serial.printf("getXAccelOffset()(): %d\n",imu.getXAccelOffset());
    Serial.printf("getYAccelOffset()(): %d\n",imu.getYAccelOffset());
    Serial.printf("getZAccelOffset()(): %d\n",imu.getZAccelOffset());
    Serial.printf("getXGyroOffset()(): %d\n",imu.getXGyroOffset());
    Serial.printf("getYGyroOffset()(): %d\n",imu.getYGyroOffset());
    Serial.printf("getZGyroOffset()(): %d\n",imu.getZGyroOffset());
    Serial.printf("getAccelXSelfTestFactoryTrim()(): %d\n",imu.getAccelXSelfTestFactoryTrim());
    Serial.printf("getAccelYSelfTestFactoryTrim()(): %d\n",imu.getAccelYSelfTestFactoryTrim());
    Serial.printf("getAccelZSelfTestFactoryTrim()(): %d\n",imu.getAccelZSelfTestFactoryTrim());
    Serial.printf("getGyroXSelfTestFactoryTrim()(): %d\n",imu.getGyroXSelfTestFactoryTrim());
    Serial.printf("getGyroYSelfTestFactoryTrim()(): %d\n",imu.getGyroYSelfTestFactoryTrim());
    Serial.printf("getGyroZSelfTestFactoryTrim()(): %d\n",imu.getGyroZSelfTestFactoryTrim());
  #endif

  return success;
}
void Sensor_System::IMU_Filter(){
  // Accelerometer LPF (IIR Averager)
  static float buff[3] = {0};
  float K_decay = 0.9;
  buff[0] += (1-K_decay)*(x_ddot[0]-buff[0]);
  buff[1] += (1-K_decay)*(x_ddot[1]-buff[1]);
  buff[2] += (1-K_decay)*(x_ddot[2]-buff[2]);
  x_ddot[0] = buff[0];
  x_ddot[1] = buff[1];
  x_ddot[2] = buff[2];

  //Angular Position Complementary Filter Drift Control
  // float a_tolerence = 0.5; //[g] Reject accelerometer impulses exceeding this value
  // float x_ddot_mag = sqrt(x_ddot[0]*x_ddot[0]+x_ddot[1]*x_ddot[1]+x_ddot[2]*x_ddot[2]);
  // if (x_ddot_mag < IMU_Gravity+a_tolerence && x_ddot_mag > IMU_Gravity-a_tolerence){
  //   float w_acc[3] = {
  //     atan2f(x_ddot[1],x_ddot[2]),  //roll
  //     atan2f(x_ddot[0],x_ddot[2]),  //pitch
  //     w[2]                          //yaw (no absolute reference atm)
  //   };
  //   w[0] = w[0]*0.95 + w_acc[0]*0.05;   //roll
  //   w[1] = w[1]*0.95 + w_acc[1]*0.05;   //pitch
  //   w[2] = w[2]*0.95 + w_acc[2]*0.05;   //yaw
  // }
  
  //Linear Velocity Complementary Filter Drift Control (todo)


}
void Sensor_System::IMU_Update(){
  static unsigned long last_time;
  if (millis() - last_time < IMU_Period) return;
  //Obtain Measured Values
  w_dot[0]  = imu.getRotationX()*imu.get_gyro_resolution()*(PI/180.0)/2.0;      //[rad/s]
  w_dot[1]  = imu.getRotationY()*imu.get_gyro_resolution()*(PI/180.0)/2.0;      //[rad/s]
  w_dot[2]  = imu.getRotationZ()*imu.get_gyro_resolution()*(PI/180.0)/2.0;      //[rad/s]
  x_ddot[0] = imu.getAccelerationX()*imu.get_acce_resolution()*IMU_Gravity/2.0; //[m/s^2]
  x_ddot[1] = imu.getAccelerationY()*imu.get_acce_resolution()*IMU_Gravity/2.0; //[m/s^2]
  x_ddot[2] = imu.getAccelerationZ()*imu.get_acce_resolution()*IMU_Gravity/2.0; //[m/s^2]
  //Perform Sensor Filtering
  IMU_Filter();
  //Update Inferred Values
  float interval = (millis()-last_time)*0.001;
  w[0] += w_dot[0]*interval;      //[rad]
  w[1] += w_dot[1]*interval;      //[rad]
  w[2] += w_dot[2]*interval;      //[rad]
  x_dot[0] += x_ddot[0]*interval; //[m/s]
  x_dot[1] += x_ddot[1]*interval; //[m/s]
  x_dot[2] += x_ddot[2]*interval; //[m/s]
  last_time = millis();

  #ifdef IMU_Debug
    Serial.print("w_x:");Serial.print(w[0]*(180.0/PI));Serial.print(",");
    Serial.print("w_y:");Serial.print(w[1]*(180.0/PI));Serial.print(",");
    Serial.print("w_z:");Serial.print(w[2]*(180.0/PI));Serial.print(",");
    Serial.print("wd_x:");Serial.print(w_dot[0]*(180.0/PI));Serial.print(",");
    Serial.print("wd_y:");Serial.print(w_dot[1]*(180.0/PI));Serial.print(",");
    Serial.print("wd_z:");Serial.print(w_dot[2]*(180.0/PI));Serial.print(",");
    Serial.print("v_x:");Serial.print(x_dot[0]);Serial.print(",");
    Serial.print("v_y:");Serial.print(x_dot[1]);Serial.print(",");
    Serial.print("v_z:");Serial.print(x_dot[2]);Serial.print(",");
    Serial.print("a_x:");Serial.print(x_ddot[0]);Serial.print(",");
    Serial.print("a_y:");Serial.print(x_ddot[1]);Serial.print(",");
    Serial.print("a_z:");Serial.print(x_ddot[2]);
    Serial.print("\n");
  #endif
}

//Global Position System (GPS)
//--------------------------
bool Sensor_System::GPS_Init(){
  //Check for presence

  //Configure the GPS (G28U7FTTL)
  // delay(1000);  //Wait for GPS Startup
  // Serial.print(0xB56206090D00FFFF000000000000FFFF0000071F9E); //reset to manufacturer defaults
  // // //Serial.print(0x); //disable GGA
  // Serial.print(0xB562060806006400010001007A12B562060800000E30); //set output rate to 10Hz
  // Serial.print(0xb5620600140001000000d008000000c201000700070000000000c496b56206000100010822); //set baud rate to 115200
  // Serial.print(0xB56206090D0000000000FFFF0000000000001731BF); //save changes
  // Serial.updateBaudRate(115200);
  return true;
}
void Sensor_System::GPS_Update(){
  int count = Serial.available();
  if (count == 0) return;
  for (int i=0;i<count;i++){
    char c = Serial.read();
    // Serial.print(c);  //advanced gps debugging
    if(gps.encode(c)){
      //Perform GPS Update
      gps.get_position(&latitude,&longitude); //[deg*1E6]
      latitude *= 10;                         //[deg*1E7]
      longitude *= 10;                        //[deg*1E7]
      // altitude = gps.altitude();              //[cm]
      // gps.get_datetime(unsigned long *date, unsigned long *time); //[ddmmyy],[hhmmss],[ms]
      // gps.course(); //[deg*100]
      // gps.f_speed_mps(); //[m/s]
      // gps.satellites(); //# satellites
      // gps.hdop(); //[*100] horizontal dilution
      // distance_between(float lat1, float long1, float lat2, float long2);
      // course_to(float lat1, float long1, float lat2, float long2);
      // cardinal(float course);
    }
  }

  #ifdef GPS_Debug
    Serial.printf("GPS_latitude: %d\n",latitude);
    Serial.printf("GPS_longitude: %d\n",longitude);
    Serial.printf("GPS_altitude: %d\n",altitude);
    Serial.printf("GPS_speed: %d [knots]\n",gps.speed());
    Serial.printf("GPS_speed_mps: %f\n",gps.f_speed_mps());
    Serial.printf("GPS_course: %f\n",gps.f_course());
    Serial.printf("GPS_hdop: %d\n",gps.hdop());
  #endif
}

//Barometric Pressure Sensor (BAR)
//--------------------------
bool Sensor_System::BAR_Init(){
  bool success = true;
  //Configure I2C Interface
  pinMode(I2C_SDA,INPUT_PULLUP);
  pinMode(I2C_SCL,INPUT_PULLUP);
  success &= Wire.begin(I2C_SDA,I2C_SCL,I2C_Freq);
  //Create the BAR Object (BMP180)
  bar = BMP180I2C(BMP180_ADDR);
  //Configure the BAR Object
  success &= bar.begin();
  bar.resetToDefaults();
  bar.setSamplingMode(BAR_Mode);
  return success;
}
void Sensor_System::BAR_Update(){
  static int last_time;
  if (millis() - last_time < BAR_Period) return;
  temp = bar.readTemperature();
  pressure = bar.readPressure();
  last_time = millis();

  #ifdef BAR_Debug
    Serial.printf("Temp: %f , Press: %f \n",temp,pressure);
  #endif
}

//Battery Monitor (BAT)
//--------------------------
bool Sensor_System::BAT_Init(){
  pinMode(BAT_SENSE,INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  return true;
}
void Sensor_System::BAT_Update(){
  static int last_time;
  if (millis() - last_time < BAT_Period) return;
  BAT_Level = analogRead(BAT_SENSE);
  last_time = millis();

  #ifdef BAT_Debug
    Serial.printf("BAT: %f\n",BAT_Level);
  #endif
}

//Camera Driver (CAM)
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
  s->set_framesize(s,CAM_Size);
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
  if(!GPS_Init()) Serial.println("Error Initializing Sensor->GPS");
  if(!BAR_Init()) Serial.println("Error Initializing Sensor->BAR");
  if(!BAT_Init()) Serial.println("Error Initializing Sensor->BAT");
  if(!CAM_Init()) Serial.println("Error Initializing Sensor->CAM");
}
void Sensor_System::Update(){
  IMU_Update();
  GPS_Update();
  BAR_Update();
  BAT_Update();
  //Camera data only updated on request
}
