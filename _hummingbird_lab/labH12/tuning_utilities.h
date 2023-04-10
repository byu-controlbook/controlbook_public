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

SingleGainTuning tune_wn_th;
SingleGainTuning tune_zeta_th;
SingleGainTuning tune_pi_lon;
SingleGainTuning tune_wn_phi;
SingleGainTuning tune_zeta_phi;
SingleGainTuning tune_wn_psi;
SingleGainTuning tune_zeta_psi;
SingleGainTuning tune_pi_lat;
SingleGainTuning tune_km;

int tuneGains() {
  float joy = getPinVoltage(JOYSTICK_PUSH);
  char buffer[40];
  
  static int state = 0;
  switch(state) {
    case 0:  // set up single gain tuners
      tune_wn_th.init(JOYSTICK_SIDESIDE, gains.wn_th, 0.5);
      tune_zeta_th.init(JOYSTICK_SIDESIDE, gains.zeta_th, 0.1);
      tune_pi_lon.init(JOYSTICK_SIDESIDE, gains.pi_lon, 0.1);
      tune_wn_phi.init(JOYSTICK_SIDESIDE, gains.wn_phi, 0.5);
      tune_zeta_phi.init(JOYSTICK_SIDESIDE, gains.zeta_phi, 0.1);
      tune_wn_psi.init(JOYSTICK_SIDESIDE, gains.wn_psi, 0.1);
      tune_zeta_psi.init(JOYSTICK_SIDESIDE, gains.zeta_psi, 0.1);
      tune_pi_lat.init(JOYSTICK_SIDESIDE, gains.pi_lat, 0.1);
      tune_km.init(JOYSTICK_SIDESIDE, gains.km, 0.01);
      state = 1;
      break;
    case 1: // tune wn_th
      gains.wn_th = tune_wn_th.update();
      Serial.print("wn_th ");
      Serial.print(gains.wn_th);
      Serial.print(": ");
      Serial.print(gains.wn_th);
      Serial.print(", ");
      if (abs(joy)<0.1) state=2;
      break;
    case 2: // button pushed
      if (abs(joy)>=0.1) state=3;
      break;
    case 3: // tune zeta_theta
      gains.zeta_th = tune_zeta_th.update();
      Serial.print("zeta_th ");
      Serial.print(gains.zeta_th);
      Serial.print(": ");
      Serial.print(gains.zeta_th);
      Serial.print(", ");
      if (abs(joy)<0.1) state=4;
      break;
    case 4: // button pushed
      if (abs(joy)>=0.1) state=5;
      break; 
    case 5: // tune pi_lon
      gains.pi_lon = tune_pi_lon.update();
      Serial.print("pi_lon ");
      Serial.print(gains.pi_lon);
      Serial.print(": ");
      Serial.print(gains.pi_lon);
      Serial.print(", ");
      if (abs(joy)<0.1) state=6;
      break;
    case 6: // button pushed
      if (abs(joy)>=0.1) state=7;
      break;
    case 7: // tune wn_phi
      gains.wn_phi = tune_wn_phi.update();
      Serial.print("wn_phi ");
      Serial.print(gains.wn_phi);
      Serial.print(": ");
      Serial.print(gains.wn_phi);
      Serial.print(", ");
      if (abs(joy)<0.1) state=8;
      break;
    case 8: // button pushed
      if (abs(joy)>=0.1) state=9;
      break;      
    case 9: // tune zeta_phi
      gains.zeta_phi = tune_zeta_phi.update();
      Serial.print("zeta_phi ");
      Serial.print(gains.zeta_phi);
      Serial.print(": ");
      Serial.print(gains.zeta_phi);
      Serial.print(", ");
      if (abs(joy)<0.1) state=10;
      break;
    case 10: // button pushed
      if (abs(joy)>=0.1) state=11;
      break;      
    case 11: // tune wn_psi
      gains.wn_psi = tune_wn_psi.update();
      Serial.print("wn_psi ");
      Serial.print(gains.wn_psi);
      Serial.print(": ");
      Serial.print(gains.wn_psi);
      Serial.print(", ");
      if (abs(joy)<0.1) state=12;
      break;
    case 12: // button pushed
      if (abs(joy)>=0.1) state=13;
      break;      
    case 13: // tune zeta_psi
      gains.zeta_psi = tune_zeta_psi.update();
      Serial.print("zeta_psi ");
      Serial.print(gains.zeta_psi);
      Serial.print(": ");
      Serial.print(gains.zeta_psi);
      Serial.print(", ");
      if (abs(joy)<0.1) state=14;
      break;
    case 14: // button pushed
      if (abs(joy)>=0.1) state=15;
      break;      
    case 15: // tune pi_lat
      gains.pi_lat = tune_pi_lat.update();
      Serial.print("pi_lat ");
      Serial.print(gains.pi_lat);
      Serial.print(": ");
      Serial.print(gains.pi_lat);
      Serial.print(", ");
      if (abs(joy)<0.1) state=1;
      break;    
    case 16: // button pushed
      if (abs(joy)>=0.1) state=17; // change to 11 to include km
      break;     
    case 17: // tune km
      gains.km = tune_km.update();
      Serial.print("km ");
      Serial.print(gains.km);
      Serial.print(": ");
      Serial.print(gains.km);
      Serial.print(", ");
      if (abs(joy)<0.1) state=1;
      break;
  }
}

#endif 
