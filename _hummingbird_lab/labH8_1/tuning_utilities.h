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

SingleGainTuning tune_kp_theta;
SingleGainTuning tune_kd_theta;
SingleGainTuning tune_ki_theta;
SingleGainTuning tune_km;

int tuneGains() {
  float joy = getPinVoltage(JOYSTICK_PUSH);
  char buffer[40];
  
  static int state = 0;
  switch(state) {
    case 0:  // set up single gain tuners
      tune_kp_theta.init(JOYSTICK_SIDESIDE, gains.kp_theta, 0.05);
      tune_kd_theta.init(JOYSTICK_SIDESIDE, gains.kd_theta, 0.05);
      tune_ki_theta.init(JOYSTICK_SIDESIDE, gains.ki_theta, 0.01);
      tune_km.init(JOYSTICK_SIDESIDE, gains.km, 0.01);
      state = 1;
      break;
    case 1: // tune kp_theta
      gains.kp_theta = tune_kp_theta.update();
      Serial.print("kp ");
      Serial.print(gains.kp_theta);
      Serial.print(": ");
      Serial.print(gains.kp_theta);
      Serial.print(",");
      if (abs(joy)<0.1) state=2;
      break;
    case 2: // button pushed
      if (abs(joy)>=0.1) state=3;
      break;
    case 3: // tune kd_theta
      gains.kd_theta = tune_kd_theta.update();
      Serial.print("kd ");
      Serial.print(gains.kd_theta);
      Serial.print(": ");
      Serial.print(gains.kd_theta);
      Serial.print(",");
      if (abs(joy)<0.1) state=4;
      break;
    case 4: // button pushed
      if (abs(joy)>=0.1) state=5;
      break; 
    case 5: // tune ki_theta
      gains.ki_theta = tune_ki_theta.update();
      Serial.print("ki ");
      Serial.print(gains.ki_theta);
      Serial.print(": ");
      Serial.print(gains.ki_theta);
      Serial.print(",");
      if (abs(joy)<0.1) state=6;
      break;
    case 6: // button pushed
      if (abs(joy)>=0.1) state=7; // change to 7 to include km
      break;     
    case 7: // tune km
      gains.km = tune_km.update();
      Serial.print("km ");
      Serial.print(gains.km);
      Serial.print(": ");
      Serial.print(gains.km);
      Serial.print(",");
      if (abs(joy)<0.1) state=8;
      break;
    case 8: // button pushed
      if (abs(joy)>=0.1) state=1;
      break;     
  }
  
}

#endif 
