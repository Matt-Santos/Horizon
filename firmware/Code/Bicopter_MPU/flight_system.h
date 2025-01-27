//Bicopter Varriable
//Written by Matthew Santos

//Note: This file handles all global varriables used outside of Bicopter_MPU.ino

//PWM Settings
#define PWM_Frequency       10000 //[Hz]

//Pin Definitions
//--------------------------
#define Motor_PWM_L   3
#define Motor_PWM_R   33
#define Motor_PWM_P   32

class Flight_Param {
  public:
    uint32_t L_PWM = 0;
    uint32_t R_PWM = 0;
  void Init(){

  };
  void Update(){
    ledcWrite(Motor_PWM_L,L_PWM); //Update Left Motor
    ledcWrite(Motor_PWM_R,R_PWM); //Update Right Motor
  }
  private:


}

//Initialize Motors
  pinMode(Motor_PWM_L,OUTPUT);
  pinMode(Motor_PWM_R,OUTPUT);
  pinMode(Motor_PWM_P,OUTPUT);
  bool success = true;
  
  success &= ledcSetClockSource((ledc_clk_cfg_t) 4); //APB clock
  success &= ledcAttach(Motor_PWM_L, PWM_Frequency, 12);
  success &= ledcAttach(Motor_PWM_R, PWM_Frequency, 12);
  success &= ledcAttach(Motor_PWM_P, PWM_Frequency, 12);
  success &= ledcOutputInvert(Motor_PWM_L,true);
  success &= ledcOutputInvert(Motor_PWM_R,true);
  success &= ledcOutputInvert(Motor_PWM_P,true);
  success &= ledcWrite(Motor_PWM_L,0);
  success &= ledcWrite(Motor_PWM_R,0);
  success &= ledcWrite(Motor_PWM_P,0);
  if (!success && DEBUG_MODE){Serial.println("Error Initializing PWM");}