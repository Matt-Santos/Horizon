// Bicopter Configuration
// Written by Matthew Santos

//Firmware Configuration
//--------------------------

//Wifi Settings
#define WIFI_SSID         "Bicopter"
#define WIFI_PASS         "Bicopter"
#define WIFI_Hostname     "Bicopter"
#define WIFI_channel      1
#define WIFI_Hidden       false
#define WIFI_Connections  2 //Max active connections
#define WIFI_LOCAL_IP     192,168,1,100
#define WIFI_GATEWAY      192,168,1,10
#define WIFI_SUBNET       255,255,255,0

//IMU MPU6050 Settings
#define MPU6050_ADDR      0x68

//ADC Settings
#define ADC_SampleCount   10  //Samples to Average
#define ADC_SampleFreq    1   //[Hz]

//Pin Definitions
//--------------------------
#define Motor_PWM_L   32
#define Motor_PWM_R   33
#define Motor_PWM_P   3

#define IMU_SDA       26
#define IMU_SCL       27
#define IMU_INT       1

#define BAT_SENSE     13
#define EXTRA_PIN     0

//Hardwired Pin Functions
#define BOOT_BTN      0

#define LED_IO2       2
#define LED_TX        1
#define LED_RX        3

#define SDCard_CLK    14
#define SDCard_CMD    15
#define SDCard_DAT    2

#define CAM_Y2        4
#define CAM_Y3        5
#define CAM_Y4        18
#define CAM_Y5        19
#define CAM_Y6        36
#define CAM_Y7        39
#define CAM_Y8        34
#define CAM_Y9        35
#define CAM_XCLK      21
#define CAM_PCLK      22
#define CAM_HREF      23
#define CAM_SIOD      26
#define CAM_SIOC      27
#define CAM_VSYNC     25
#define CAMERA_MODEL_WROVER_KIT
