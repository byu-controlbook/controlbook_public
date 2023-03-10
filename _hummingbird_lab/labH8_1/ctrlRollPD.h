/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to implement controller
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <math.h>

struct {
  float kp_phi = 
  float kd_phi = 
  float km = 
} gains;

#include "tuning_utilities.h"

// physical parameters of the system
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

// Lateral controller for hummingbird
class CtrlRollPD {
  public:
    float phi_d1;
    float phi_d2;
    float phi_d3;
    float phi_dot_d2;
    float phi_dot_d3;
    
    CtrlRollPD() {  
    }

    void init() {
      // persistent variables
      phi_d1 = 0.0;
      phi_d2 = 0.0;
      phi_d3 = 0.0;
      phi_dot_d2 = 0.0;
      phi_dot_d3 = 0.0;
    }

    void update(float phi_ref, 
                SensorUtilities &sensors, 
                MotorUtilities &rotors, 
                float Ts) {

      // tune gains
      tuneGains();

      // compute phi/psi and phi_dot/psi_dot (with quadratic prediction)
      float phi_d1 = sensors.roll;
      float phi = 3*phi_d1 - 3*phi_d2 + phi_d3;
      float phi_dot_d1 = (phi_d1 - phi_d2) / Ts;
      phi_dot_d1 = (phi-phi_d1)/Ts;
      float phi_dot = 3*phi_dot_d1 - 3*phi_dot_d2 + phi_dot_d3;

      // compute feedback linearized force      
      float force_fl = 
                         
      // compute error
      float error_phi = 

      // roll control
      float torque = 
                                            
      float force = 
      
      // convert force and torque to pwm and send to motors
      float left_pwm = (force+torque/P.d)/(2.0*gains.km);
      float right_pwm = (force-torque/P.d)/(2.0*gains.km);
      rotors.update(left_pwm, right_pwm); 

      // update all delayed variables
      phi_d3 = phi_d2;
      phi_d2 = phi_d1;
      phi_d1 = phi;
      phi_dot_d3 = phi_dot_d2;
      phi_dot_d2 = phi_dot_d1;
      // print commanded values
      Serial.print("Phi_ref:");
      Serial.print(phi_ref*180/PI);
      Serial.print(",");
      Serial.print("Phi:");
      Serial.println(phi*180/PI);      
    }

    float saturate(float value, float min_value, float max_value) {
      // Implements the saturation function
      return min(max(min_value, value), max_value);
    }  
};

#endif 
