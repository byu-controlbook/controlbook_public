/**
 * \file time_utilities.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to manage time
 */

#ifndef TIME_UTILITIES_H
#define TIME_UTILITIES_H

class TimeUtilities  {
  public:
    unsigned long start_ms;  // start time in microseconds
    unsigned long previous_ms;  // previous time through loop
    unsigned long current_ms;  // current time in microseconds
    float current;  // current time in seconds
    float Ts;  // current sample rate in seconds

    TimeUtilities() {
    }
    
    void init() {
      //start_ms=micros();
      //previous_ms=micros();
      start_ms=millis();
      previous_ms=millis();
      current = 0.0;
      Ts = 0.01;
    }

    void update() {
      //current_ms = micros() - start_ms; 
      //current = float(current_ms) / 1000000.0;  // convert to seconds 
      //Ts = (float(current_ms) - float(previous_ms) ) / 1000000.0; 
      current_ms = millis() - start_ms;
      current = float(current_ms) / 1000.0;  
        // current run time seconds
      Ts = (float(current_ms) - float(previous_ms) ) / 1000.0; 
        // sample rate in seconds 
      previous_ms = current_ms;
    }
};

#endif 
