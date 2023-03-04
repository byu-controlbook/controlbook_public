#ifndef CONTROL_H
#define CONTROL_H

#include <math.h>
#include "tuning_utilities.h"

// physical parameters of the system
static struct {
  float km=0.54;  // this will be different for every hummingbird
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

// use these classes to tune different gain values
GainTuning tune_kp;
GainTuning tune_kd;
GainTuning tune_ki;
GainTuning tune_km;

// Controller to find the motor constant km
class CtrlLonPID {
  public:
    float ki_pitch;
    float kp_pitch;
    float kd_pitch;
    float theta_d1;
    float theta_dot;
    float theta_dot_d1;
    float theta_ddot;
    float integrator_theta;
    float error_theta_d1;
    float beta;
    float b_theta;
    float km;
    
    CtrlLonPID() {  
      // tuning parameters
      tr_pitch = ; // rise time for pitch
      zeta_pitch = ; // damping ratio for pitch
      ki_pitch = ;  // integrator gain for pitch
      // gain calculation
      b_theta = P.ellT / (P.m1 * P.ell1 * P.ell1 + P.m2 * P.ell2 * P.ell2 + P.J1y + P.J2y);
      float wn_pitch = ;  // natural frequency for pitch
      kp_pitch = ;  
      kd_pitch = ; 
      km = P.km; 
    }

    void init() {
      // delayed variables
      theta_d1 = 0.0;
      theta_dot = 0.0;
      integrator_theta = 0.0;
      error_theta_d1 = 0.0;

      // Use these to setup gain tuning with the joystick
      tune_kp.init(JOYSTICK_UPDOWN, kp_pitch, 0.1);
      tune_kd.init(JOYSTICK_SIDESIDE, kd_pitch, 0.1);
      //tune_ki.init(JOYSTICK_UPDOWN, ki_pitch, 0.01);
      //tune_km.init(JOYSTICK_UPDOWN, km, 0.01);
    }

    void update(float theta_ref, State state, MotorUtilities &rotors, float Ts) {
      // tune gains
      //km = tune_km.update();
      kp_pitch = tune_kp.update();  
      kd_pitch = tune_kd.update(); 
      //ki_pitch = tune_ki.update();  

      theta_dot = (state.theta - theta_d1) / Ts;

      float force = ; 
      float torque = 0.0;

      // convert force and torque to pwm and send to motors
      float left_pwm = (force+torque/P.d)/(2.0*km);
      float right_pwm = (force-torque/P.d)/(2.0*km);
      rotors.update(left_pwm, right_pwm); 

      // update all delayed variables
      theta_d1 = state.theta;
      theta_dot_d1 = theta_dot;
      error_theta_d1 = error_theta;

      // print stuff for serial plotter
      Serial.print("Theta_ref:");
      Serial.print(theta_ref*180/PI);
      Serial.print(",");
      Serial.print("Theta:");
      Serial.print(state.theta*180/PI);
      Serial.print(",");
      Serial.print("Theta_dot:");
      Serial.print(theta_dot*180/PI);
      Serial.print(",");
      Serial.print("kp:");
      Serial.print(kp_pitch);
      Serial.print(",");
      Serial.print("kd:");
      Serial.println(kd_pitch);             
      //Serial.print("force:");
      //Serial.println(10*force);
      //Serial.print(",");
      //Serial.print("b_theta:");
      //Serial.println(b_theta);      
    }

    float saturate(float value, float min_value, float max_value) {
      // Implements the saturation function
      return min(max(min_value, value), max_value);
    }  
};

#endif 
