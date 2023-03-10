/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to implement controller
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <math.h>

// the gains are stored in this structure for easy access. 
// Enter your current best gains in this structure.  After tuning,
// modify the structure to reflect the tuned gains.
struct {
  float kp_theta = ;
  float kd_theta = ;
  float ki_theta = ; 
  float km = ; 
} gains;

#include "tuning_utilities.h"

// physical parameters of the system
// This creates a global structure that allows all system parameters
// to be accessed via, for example, P.m1
static struct {
  float m1=0.108862;
  float ell1=0.247;
  float m2=0.4717;
  float ell2=-0.039;
  float m3=.1905;
  float g = 9.81;
  float ellT = 0.29;
  float ell3x=-.007;
  float ell3y=.018;
  float J1x = 0.000189;
  float J1y = 0.001953;
  float J1z = 0.001894;
  float J2x = 0.00231;
  float J2y = 0.003274;
  float J2z = 0.003416;
  float J3x = 0.0002222;
  float J3y = 0.0001956;
  float J3z = 0.000027;
  float d = 0.12;
  float fe = (m1*ell1+m2*ell2)*g/ellT;  
  float force_max = 0.1;
} P;

// reference structure the reference signals for psi and theta
struct Reference {
  float theta = 0.0;
  float psi = 0.0;
  float phi = 0.0;  
};

// Controller to find the motor constant km
class CtrlLonPID {
  // these define public internal variables.  In C++ you don't need to use
  // the "self.*" syntax.  Public variables are available within the class
  // and can be accessed outside the class using classname.variable.
  public:
    float theta_d2;
    float theta_d3;
    float theta_dot_d2;
    float theta_dot_d3;
    float integrator_theta;
    float error_theta_d1;

    // This is the constructor, but we will put initialization stuff
    // inside the init() function.
    CtrlLonPID() {  
    }

    // This function gets called once in the Arduino setup() function
    void init() {
      // persistent variables
      integrator_theta = 0.0;
      theta_d2 = 0.0;
      theta_d3 = 0.0;
      theta_dot_d2 = 0.0;
      theta_dot_d3 = 0.0;
      error_theta_d1 = 0.0;
    }

    // This is the update function that gets called in every
    // Arduino loop
    void update(float theta_ref, 
                SensorUtilities &sensors, 
                MotorUtilities &rotors, 
                float Ts) {

      // This function can be used to tune the gains in real-time using 
      // the joystick
      tuneGains();

      // compute theta and theta_dot (with quadratic prediction)
      float theta_d1 = sensors.pitch;
      float theta = 3*theta_d1 - 3*theta_d2 + theta_d3;
      float theta_dot_d1 = (sensors.pitch - theta_d2) / Ts;
      float theta_dot = 3*theta_dot_d1 - 3*theta_dot_d2 + theta_dot_d3;

      // compute feedback linearized force      
      float force_fl = 

      // compute error
      float error_theta = 
      
      // update integrator 
      integrator_theta += 
      
      // pitch control
      float f_tilde =                        
      float force = 
      float torque = 0.0;
      
      // convert force and torque to pwm and send to motors
      float left_pwm = (force+torque/P.d)/(2.0*gains.km);
      float right_pwm = (force-torque/P.d)/(2.0*gains.km);
      rotors.update(left_pwm, right_pwm); 

      // update all delayed variables
      theta_d3 = theta_d2;
      theta_d2 = theta_d1;
      theta_dot_d3 = theta_dot_d2;
      theta_dot_d2 = theta_dot_d1;
      error_theta_d1 = error_theta;
      // print stuff to the Arduino plotter/monitor for debugging
      Serial.print("Theta_ref:");
      Serial.print(theta_ref*180/PI);
      Serial.print(",");
      Serial.print("Theta:");
      Serial.println(theta*180/PI);
//      Serial.print(",");
//      Serial.print("Theta_dot:");
//      Serial.print(theta_dot*180/PI);
//      Serial.print(",");
//      Serial.print("force:");
//      Serial.println(10*force);
     
    }

    float saturate(float value, float min_value, float max_value) {
      // Implements the saturation function
      return min(max(min_value, value), max_value);
    }  
};

#endif 
