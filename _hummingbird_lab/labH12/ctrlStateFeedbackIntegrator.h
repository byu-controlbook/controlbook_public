/**
 * class to implement controller
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <math.h>

struct {
  float wn_th = 
  float zeta_th = 
  float pi_lon = 
  float wn_phi = 
  float zeta_phi = 
  float wn_psi = 
  float zeta_psi = 
  float pi_lat = 
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
  float JT = m1*pow(ell1,2)+m2*pow(ell2,2)+J2z+m3*(pow(ell3x,2)+pow(ell3y,2));
  float fe = (m1*ell1+m2*ell2)*g/ellT;  
  float force_max = 0.1;
  float b_theta = ellT/(m1*pow(ell1,2)+m2*pow(ell2,2)+J1y+J2y);
} P;

// Lateral controller for hummingbird
class CtrlStateFeedbackIntegrator {
  private:
    float theta;
    float theta_d2;
    float theta_d3;
    float theta_dot_d2;
    float theta_dot_d3;
    float integrator_lon;
    float error_lon_d1;
    float phi_d1;
    float phi_d2;
    float phi_d3;
    float phi_dot_d2;
    float phi_dot_d3;
    float psi;
    float psi_d1;
    float psi_d2;
    float psi_d3;
    float psi_dot_d2;
    float psi_dot_d3;    
    float integrator_lat;
    float error_lat_d1;    
  
  public:   
    ctrlStateFeedbackIntegrator() {  
    }

    void init() {
      // persistent variables
      theta_d2 = 0.0;
      theta_d3 = 0.0;
      theta_dot_d2 = 0.0;
      theta_dot_d3 = 0.0;
      integrator_lon = 0.0;     
      error_lon_d1 = 0.0;
      phi_d1 = 0.0;
      phi_d2 = 0.0;
      phi_d3 = 0.0;
      phi_dot_d2 = 0.0;
      phi_dot_d3 = 0.0;
      psi_d1 = 0.0;
      psi_d2 = 0.0;
      psi_d3 = 0.0;
      psi_dot_d2 = 0.0;
      psi_dot_d3 = 0.0; 
      integrator_lat = 0.0;     
      error_lat_d1 = 0.0;      
    }

    void update(Reference const &reference, 
                SensorUtilities const &sensors, 
                MotorUtilities &rotors, 
                float Ts) {

      // tune gains
      tuneGains();
      
      float alpha1_lon = 
      float alpha2_lon = 
      float alpha3_lon = 
      float k_theta = 
      float k_thetadot = 
      float ki_lon = 
      float alpha1_lat = 
      float alpha2_lat = 
      float alpha3_lat = 
      float alpha4_lat = 
      float alpha5_lat = 
      float b1 = 1/P.J1x;
      float a1 = P.ellT*P.fe/(P.JT+P.J1z);
      float k_phi = 
      float k_psi = 
      float k_phidot = 
      float k_psidot = 
      float ki_lat = 
      
      // compute theta and theta_dot (with quadratic prediction)
      float theta_d1 = sensors.pitch;
      float theta = 3*theta_d1 - 3*theta_d2 + theta_d3;
      float theta_dot_d1 = (sensors.pitch - theta_d2) / Ts;
      float theta_dot = 3*theta_dot_d1 - 3*theta_dot_d2 + theta_dot_d3;
      // compute phi/psi and phi_dot/psi_dot (with quadratic prediction)
      float phi_d1 = sensors.roll;
      float phi = 3*phi_d1 - 3*phi_d2 + phi_d3;
      float phi_dot_d1 = (phi_d1 - phi_d2) / Ts;
      phi_dot_d1 = (phi-phi_d1)/Ts;
      float phi_dot = 3*phi_dot_d1 - 3*phi_dot_d2 + phi_dot_d3;
      float psi = sensors.yaw;
      float psi_dot_d1 = (psi - psi_d1) / Ts;
      float psi_dot = 3*psi_dot_d1 - 3*psi_dot_d2 + psi_dot_d3;      

      theta = sensors.pitch;
      // compute feedback linearized force      
      float force_fl = 
      // compute error
      float error_lon = reference.theta - theta;      
      float error_lat = reference.psi - psi; 
      // update integrator 
      integrator_lon +=   
      integrator_lat += 
      // longitudinal control
      float force =                      
      // lateral control
      float torque = 
      // convert force and torque to pwm and send to motors
      float left_pwm = (force+torque/P.d)/(2.0*gains.km);
      float right_pwm = (force-torque/P.d)/(2.0*gains.km);
      rotors.update(left_pwm, right_pwm); 

      // update all delayed variables
      theta_d3 = theta_d2;
      theta_d2 = theta_d1;
      theta_d1 = theta;
      theta_dot_d3 = theta_dot_d2;
      theta_dot_d2 = theta_dot_d1;
      theta_dot_d1 = theta_dot;
      phi_d3 = phi_d2;
      phi_d2 = phi_d1;
      phi_d1 = phi;
      phi_dot_d3 = phi_dot_d2;
      phi_dot_d2 = phi_dot_d1;
      psi_d3 = psi_d2;
      psi_d2 = psi_d1;
      psi_d1 = psi;
      psi_dot_d3 = psi_dot_d2;
      psi_dot_d2 = psi_dot_d1;      
      error_lon_d1 = error_lon;
      error_lat_d1 = error_lat;
      printToSerial(theta, reference.theta, psi, reference.psi);      
    }

    float saturate(float value, float min_value, float max_value) {
      // Implements the saturation function
      return min(max(min_value, value), max_value);
    }  

    void printToSerial(float theta, float theta_ref, float psi, float psi_ref) {
      // print commanded values
      Serial.print("Theta_ref:");
      Serial.print(theta_ref*180/PI);
      Serial.print(",");
      Serial.print("Theta:");
      Serial.print(theta*180/PI);
      Serial.print(",");
      Serial.print("Psi_ref:");
      Serial.print(psi_ref*180/PI);
      Serial.print(",");
      Serial.print("Psi:");
      Serial.print(psi*180/PI);
      Serial.println(",");
    }
};

#endif 
