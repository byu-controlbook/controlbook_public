/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to manage sensors
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "AMT203.h"

AMT203 encoder_yaw(PIN_ENCODER_YAW_CS);
AMT203 encoder_pitch(PIN_ENCODER_PITCH_CS);
AMT203 encoder_roll(PIN_ENCODER_ROLL_CS);      

class SensorUtilities  {
  public:
    float roll;  // roll angle in radians
    float pitch;  // pitch angle in radians
    float yaw;  // yaw angle in radians
    // sensor polling interval (ms)
    unsigned long POLL_INTERVAL;
    unsigned long previous_poll_time;    
    // setup interface to encoders

    SensorUtilities() {
      POLL_INTERVAL = 4; // sensor poll interval in micro seconds
    }
    
    void init() {
      // set up encoders
      SPI.begin();
      encoder_yaw.init();
      encoder_pitch.init();
      encoder_roll.init();
      
      previous_poll_time = micros();
      roll = (-1.0) * encoder_roll.read();
      pitch = (-1.0) * encoder_pitch.read();
      yaw = (-1.0) * encoder_yaw.read();
    }

    void update() {
      unsigned long current_time = micros();
      if ((current_time - previous_poll_time) >= POLL_INTERVAL) {
        roll = (-1.0) * encoder_roll.read();
        pitch = (-1.0) * encoder_pitch.read();
        yaw = (-1.0) * encoder_yaw.read();
        previous_poll_time = current_time;
      }
    }

    void zero() {
      // turn off interrupts and watchdog, they mess with the code
      detachInterrupt(digitalPinToInterrupt(ARM_SWITCH));
      wdt_disable();
      // zero the encoders
      encoder_yaw.zero();
      encoder_pitch.zero();
      encoder_roll.zero();
      // Reenable the interrupts
      attachInterrupt(digitalPinToInterrupt(ARM_SWITCH), 
                      arm_switch_ISR, 
                      CHANGE);
      wdt_enable(WDTO_60MS);
    }
};

#endif 
