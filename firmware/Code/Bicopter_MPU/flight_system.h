//Bicopter Flight System
//Written by Matthew Santos
#ifndef FLIGHT_SYSTEM_H
#define FLIGHT_SYSTEM_H

#include "Arduino.h"

//Motor Settings (PWM)
#define PWM_Frequency     10000 //[Hz]
#define Motor_PWM_L       3     //Left Motor GPIO Pin
#define Motor_PWM_R       33    //Right Motor GPIO Pin
#define Motor_PWM_P       32    //Pitch Motor GPIO Pin

class Flight_System {
  public:
    //Motor Varriables
    uint32_t L_PWM = 0;
    uint32_t R_PWM = 0;
    //
  
    //Public Functions
    //---------------------
    void Init();
    void Update();
  private:
    //Private Functions
    //---------------------
    bool MOTOR_Init();
};

extern Flight_System flight;

#endif