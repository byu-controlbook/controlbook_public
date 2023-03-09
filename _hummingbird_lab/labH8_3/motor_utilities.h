/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to manage motors
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>

// setup interface to motors
static constexpr float MIN_THROTTLE = 0.05f;
    // minimum armed throttle, minimum command for motors to move
static constexpr float THROTTLE_MAX = 0.5f;
    // maximum throttle value for safety
static constexpr float FULL_THROTTLE = 1.0f;
    // Full throttle command
static constexpr float ZERO_THROTTLE = 0.0f;
    // Zero throttle command
static constexpr unsigned long SETPOINT_TIMEOUT_MS = 100;
    // motor setpoint timeout (ms), turn off motors if no messages within this time
static constexpr unsigned long PWM_FREQ_HZ = 1000; //50; 
    // This is about right according to some sites//was 50, 1000 and 2000
static constexpr unsigned long PWM_MIN_US = 950; 
    //ALT IS 20 0 and 180
static constexpr unsigned long PWM_MAX_US = 1650;

class MotorUtilities  {
  public:
    float motor_left_setpoint;
    float motor_right_setpoint;
    long setpoint_time_ms;  
    Servo esc_right;
    Servo esc_left;

    MotorUtilities() {
      float motor_left_setpoint = 0.0;
      float motor_right_setpoint = 0.0;
      long setpoint_time_ms = 0;   
    }
    
    void init() {
      // Calibrate ESC Throttle Range
      // When an ESC is used with a transmitter in RC planes the range of commands that the transmitter
      // can send varies. In order to get the full range of the ESC with this range of possible commands
      // calibration must be performed. When an ESC is powered on it expects the throttle command to be
      // at its low end before it will "arm" and actually move the motors. This is for safety. When an 
      // ESC is powered and the throttle command is at the high end the ESC will enter throttle programming
      // mode. The highest commanded value it sees in this period will be mapped to the highest speed the 
      // ESC can turn the motor. It should indicate that it has accepted the high command with a beep or 
      // some other signal. Then the throttle stick should be moved to its lowest position (sending the
      // smallest command) and the ESC will accept this command as its 0 speed signal and map the range
      // of speeds between the highest and lowest commands. Refer to the ESC instruction sheet for timings
      // and other values that can be programmed in this manner.
      // TODO: Adjust timings to calibrate the ESCs
      
      // turn off interrupts and watchdog, they mess with the code
      detachInterrupt(digitalPinToInterrupt(ARM_SWITCH));
      wdt_disable();
  
      esc_left.attach(PIN_MOTOR_LEFT_PWM);
      esc_right.attach(PIN_MOTOR_RIGHT_PWM);

      write_to_esc(ZERO_THROTTLE, ZERO_THROTTLE);

      // LED Sequence 1, wait a few seconds for user to turn off power to ESC
      digitalWrite(LED_ARM, HIGH);
      digitalWrite(LED_RX, LOW);
      delay(2000); // delay to allow ESC to reset

  
      if (serial_debug){
        //Serial.println("Begin arming sequence");
      }
      if (serial_debug){
        //Serial.println("Highest throttle, Delay 5 sec");
      }
      // Full throttle command when the ESC gets power power puts the ESC in throttle calibration mode
      write_to_esc(THROTTLE_MAX, THROTTLE_MAX);
      // LED Sequence 2
      digitalWrite(LED_ARM, LOW);
      digitalWrite(LED_RX, HIGH);

      // Wait for motor to accept max throttle command
      delay(3000); // length of time max beeps are going

      if (serial_debug){
        //Serial.println("Throttle Zero, Delay 8.5 sec");
      }
      // This sets the low point of the ESC throttle range
      write_to_esc(ZERO_THROTTLE, ZERO_THROTTLE);
      // LED Sequence 3
      digitalWrite(LED_ARM, HIGH);
      digitalWrite(LED_RX, HIGH);
      delay(2000); // Length of time min beeps are going
      if (serial_debug){
        //Serial.println("ESC's Calibrated");
      }

      // Reset LED's
      digitalWrite(LED_ARM, LOW);
      digitalWrite(LED_RX, LOW);
      write_to_esc(MIN_THROTTLE, MIN_THROTTLE);

      // Restart interrupts and make sure the ESCs are powered
      attachInterrupt(digitalPinToInterrupt(ARM_SWITCH), arm_switch_ISR, CHANGE);
      wdt_enable(WDTO_60MS);
      serial_debug=false;
      return;
    }

    void update(float left, float right) {
      // only turn on motors if armed, and power supply is on
      // analogRead should return ~886 (calculated) when power supply is 12V
      //armed && (analogRead(SUPPLY_ON_PIN) > 500))
      if (analogRead(SUPPLY_ON_PIN) > 700) {
        left = saturate(left, MIN_THROTTLE, THROTTLE_MAX);
        right = saturate(right, MIN_THROTTLE, THROTTLE_MAX);
        write_to_esc(left, right);
      } // otherwise, turn off the motors
      else {
        write_to_esc(MIN_THROTTLE, MIN_THROTTLE);
      }
    }

    void write_to_esc(float left, float right) {
      // This utility sends the throttle commands to the motor ESC (electronic
      //  speed controllers).
      // left and right are values between [0, 1].  
      int left_pwm = pwm_duty_cycle(left);
      int right_pwm = pwm_duty_cycle(right);
      esc_left.writeMicroseconds(left_pwm);
      esc_right.writeMicroseconds(right_pwm);
    }

    unsigned int pwm_duty_cycle(float throttle) {
      // throttle is a value between [0, 1].  This utility converts to arduino PWM
      return PWM_MIN_US + static_cast<unsigned int>(throttle * (PWM_MAX_US - PWM_MIN_US));
    }

    float saturate(float value, float min_value, float max_value) {
      // Implements the saturation function
      return min(max(min_value, value), max_value);
    }    

};

#endif 
