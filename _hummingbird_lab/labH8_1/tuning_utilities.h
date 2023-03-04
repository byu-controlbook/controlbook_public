/**
 * \file reference.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to manage references
 */

#ifndef TUNING_H
#define TUNING_H

#include <math.h>

class GainTuning  {
  public:
    int state;
    int pin;
    float gain;
    float increment;

    TuningUtilities() {
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
          gain = gain + increment;
          state = 3;
          break;
        case 2:
          gain = gain - increment;
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

#endif 
