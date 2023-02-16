/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to implement controller
 */

#ifndef CONTROL_H
#define CONTROL_H


// physical parameters of the system
static struct {
  float km=0.3380;
  float m1=.108862;
  float ell1=.247;
  float m2=.4717;
  float ell2=-.039;
  float m3=.1905;
  float g = 9.81;
  float ellT = .355;
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
  float d=.12;
  float fe=(m1*ell1+m2*ell2)*g/ellT;  
} P;

// state structure contains current estimate of state
struct State {
  float phi=0.0;
  float theta=0.0;
  float psi=0.0;
  float phi_dot=0.0;
  float theta_dot=0.0;
  float psi_dot=0.0;
};

// reference structure the reference signals for psi and theta
struct Reference {
  float theta = 0.0;
  float psi = 0.0;
  float phi = 0.0;  
};

// Controller to find the motor constant km
class CtrlEquilibrium  {
  public:

    CtrlEquilibrium() {      
    }

    void init() {
      
    }

    void update(State state, MotorUtilities &rotors) {

      float theta_ref = getPinVoltage(PITCH_REFERENCE);
      float psi_ref = getPinVoltage(YAW_REFERENCE);
      // Serial.print(psi_ref);
      // Serial.print(",");
      // Serial.println(theta_ref);  

      static int state_machine=0;
      switch(state_machine) {
        case 0:
          if (theta_ref > 4.0) { state_machine = 1; }
          if (theta_ref < 1.5) { state_machine = 2; }
          break;
        case 1:
          P.km += -0.001;
          state_machine = 3;
          break;
        case 2:
          P.km += 0.001;
          state_machine = 4;
          break;
        case 3:
          if (theta_ref < 4.0) { state_machine = 0; }
          break;
        case 4:
          if (theta_ref > 1.5) { state_machine = 0; }
          break;
      }
      

      // if (state.theta < 0) {
      //   P.km += -0.0001;
      // }
      // if (state.theta > 0) {
      //   P.km += +0.0001;
      // }

      // equilibrium force
      float force = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2) / P.ellT;
      float torque = 0.0;

      // convert force and torque to pwm and send to motors
      float left_pwm = (force+torque/P.d)/(2.0*P.km);
      float right_pwm = (force-torque/P.d)/(2.0*P.km);
      rotors.update(left_pwm, right_pwm); 

        // print theta and km
      Serial.print(P.km, 4);
      Serial.print(",");
      Serial.print(state.theta);
      Serial.print(",");
      Serial.print(left_pwm);
      Serial.print(",");
      Serial.println(right_pwm);
    }
};

#endif 
