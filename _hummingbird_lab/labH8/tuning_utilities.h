/**
 * \file reference.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to manage references
 */

#ifndef TUNING_H
#define TUNING_H

#include <math.h>
#include <string.h>

class SingleGainTuning  {
  public:
    int state;
    int pin;
    float gain;
    float increment;

    SingleGainTuning() {
    }

    void init(int _pin, float _gain, float _increment) {
      state = 0;
      pin = _pin;
      gain = _gain;
      increment = _increment;
    }

    float update() {
      float joy = getPinVoltage(pin);
      switch(state) {
        case 0:
          if (joy > 4.0) { state = 1; }
          if (joy < 1.5) { state = 2; }
          break;
        case 1:
          gain = gain - increment;
          state = 3;
          break;
        case 2:
          gain = gain + increment;
          state = 4;
          break;
        case 3:
          if (joy < 4.0) { state = 0; }
          break;
        case 4:
          if (joy > 1.5) { state = 0; }
          break;
      }
      return(gain);
    }
};

SingleGainTuning tune_kp_phi;
SingleGainTuning tune_kd_phi;
SingleGainTuning tune_kp_psi;
SingleGainTuning tune_kd_psi;
SingleGainTuning tune_ki_psi;
SingleGainTuning tune_km;

int tuneGains() {
  float joy = getPinVoltage(JOYSTICK_PUSH);
  char buffer[40];
  
  static int state = 0;
  switch(state) {
    case 0:  // set up single gain tuners
      tune_kp_phi.init(JOYSTICK_SIDESIDE, gains.kp_phi, 0.0005);
      tune_kd_phi.init(JOYSTICK_SIDESIDE, gains.kd_phi, 0.0005);
      tune_kp_psi.init(JOYSTICK_SIDESIDE, gains.kp_psi, 0.01);
      tune_kd_psi.init(JOYSTICK_SIDESIDE, gains.kd_psi, 0.01);      
      tune_ki_psi.init(JOYSTICK_SIDESIDE, gains.ki_psi, 0.01);
      tune_km.init(JOYSTICK_SIDESIDE, gains.km, 0.01);
      state = 1;
      break;
    case 1: // tune kp_phi
      gains.kp_phi = tune_kp_phi.update();
      Serial.print("kp_phi ");
      Serial.print(100*gains.kp_phi);
      Serial.print(": ");
      Serial.print(gains.kp_phi);
      Serial.print(",");
      if (abs(joy)<0.1) state=2;
      break;
    case 2: // button pushed
      if (abs(joy)>=0.1) state=3;
      break;
    case 3: // tune kd_phi
      gains.kd_phi = tune_kd_phi.update();
      Serial.print("kd_phi ");
      Serial.print(100*gains.kd_phi);
      Serial.print(": ");
      Serial.print(gains.kd_phi);
      Serial.print(",");
      if (abs(joy)<0.1) state=4;
      break;
    case 4: // button pushed
      if (abs(joy)>=0.1) state=5;
      break; 
    case 5: // tune kp_psi
      gains.kp_psi = tune_kp_psi.update();
      Serial.print("kp_psi ");
      Serial.print(100*gains.kp_psi);
      Serial.print(": ");
      Serial.print(gains.kp_psi);
      Serial.print(",");
      if (abs(joy)<0.1) state=6;
      break;
    case 6: // button pushed
      if (abs(joy)>=0.1) state=7;
      break;
    case 7: // tune kd_psi
      gains.kd_psi = tune_kd_psi.update();
      Serial.print("kd_psi ");
      Serial.print(100*gains.kd_psi);
      Serial.print(": ");
      Serial.print(gains.kd_psi);
      Serial.print(",");
      if (abs(joy)<0.1) state=8;
      break;
    case 8: // button pushed
      if (abs(joy)>=0.1) state=9;
      break;      
    case 9: // tune ki_psi
      gains.ki_psi = tune_ki_psi.update();
      Serial.print("ki_psi ");
      Serial.print(100*gains.ki_psi);
      Serial.print(": ");
      Serial.print(gains.ki_psi);
      Serial.print(",");
      if (abs(joy)<0.1) state=10;
      break;
    case 10: // button pushed
      if (abs(joy)>=0.1) state=13; // change to 11 to include km
      break;     
    case 11: // tune km
      gains.km = tune_km.update();
      Serial.print("km ");
      Serial.print(gains.km);
      Serial.print(": ");
      Serial.print(gains.km);
      Serial.print(",");
      if (abs(joy)<0.1) state=12;
      break;
    case 12: // button pushed
      if (abs(joy)>=0.1) state=13;
      break;     
    case 13: // tune km
      Serial.print("tunning off:");
      Serial.print(",");
      if (abs(joy)<0.1) state=14;
      break;
    case 14: // button pushed
      if (abs(joy)>=0.1) state=1;
      break;    }
  
}

#endif 
