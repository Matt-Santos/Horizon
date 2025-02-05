//Bicopter Flight System
//Written by Matthew Santos

/* Author Notes
- this system includes these submodules
  - Motor Control (MOTOR)
  - (NAVIGATE)  //planned feature to regulate linear movement
  - (STABILIZE) //planned feature to regulate hover movement
  - (MANUAL)    //planned feature to enable manual control
  - 
- the MOTOR Control module handles motor PWM pulses
*/

#ifndef FLIGHT_SYSTEM_H
#define FLIGHT_SYSTEM_H

#include "Arduino.h"

class Flight_System {

  //Motor Control (MOTOR)
  //--------------------------
  public:
    //MOTOR Varriables
    uint32_t L_PWM = 0;
    uint32_t R_PWM = 0;
    //MOTOR Functions
  private:
    bool MOTOR_Init();
    void MOTOR_ESC_Calibrate();


  //Manual Control
  //--------------------------
  public:
    //General Varriables
    bool ARMED = false;
    bool PREFLIGHT = false;
    uint8_t BASE_MODE = 0;
    uint8_t FLIGHT_MODE = 0;
    

  //Flight System Class 
  //--------------------------
  public:
    void Fast_Init();
    void Init();
    void Update();
    
};

#endif