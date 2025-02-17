//Bicopter Flight System
//Written by Matthew Santos

/* Author Notes
- this system includes these submodules
  - Motor Control (MOTOR)
  - Flight Control Mode (MODE)
    -> Hold Altitude (Joystick controls 2D Position)
    -> Stabalize (Stop Movement when Joystick Released)
    -> Auto (Fully controlled by waypoints/Missions)
    -> Circle (Encircle a given location)
    -> Drift Mode (convert to plane like control scheme)
    -> Guided (Manual positioning via waypoints)
    -> Land (land at designated position)
    -> Hold Position (maintain position, altitude, and heading)
- the MOTOR Control module handles motor setpoints
- 
*/

#ifndef FLIGHT_SYSTEM_H
#define FLIGHT_SYSTEM_H

#include "Arduino.h"
#include <PCA9685.h>

typedef enum {
  MODE_MANUAL,            //Manual (joystick controls rotation, throttle controls motor power)
  MODE_HOLD_ALT,          //Hold Altitude (joystick only controls X,Y position)
  MODE_STABILIZE,         //Stabilize ()
  MODE_AUTO,              //Automatic mission based waypoint controls
  MODE_CIRCLE,            //Orbit around current location
  MODE_DRIFT,             //Plane flight mode
  MODE_GUIDED,            //Manual positioning via waypoints
  MODE_LAND,              //Land Control Mode
  MODE_TAKEOFF,           //Takeoff Control Mode
  MODE_HOLD_POS           //Maintain 3D position and rotational yaw/heading
} FLIGHT_MODE_t;

class Flight_System {

  //Motor Control (MOTOR)
  //--------------------------
  public:
    bool ARMED = false;           //Motor Enable/Disable Toggle
  private:
    double L_Motor = 0;           //[%] Power to Left Motor  [0,100.0]
    double R_Motor = 0;           //[%] Power to Right Motor [0,100.0]
    double P_Motor = 0;           //[%] Target Position of Pitch Motor [0,100]
    PCA9685 motor;                //PCA9685 PWM Driver Object
    bool MOTOR_Init();
    void MOTOR_ESC_Calibrate();
    void MOTOR_Pitch_Calibrate();
    bool MOTOR_Pitch_Control(float target);
    void MOTOR_Update();

  //Flight Control Modes (MODE)
  //--------------------------
  public:
    FLIGHT_MODE_t flight_mode = MODE_MANUAL;
  private:
    bool MODE_Init();
    void MODE_Update();

  //Flight System Class 
  //--------------------------
  public:
    void Init();
    void Update();
};

#endif