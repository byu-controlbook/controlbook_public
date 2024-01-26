#include <Arduino.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <SPI.h>
#include <avr/wdt.h>
#include <Bounce2.h>
#include "hb_serial.h"

//=============================================================================
// pin definitions
//
// Arduino Nano configuration: (https://store.arduino.cc/usa/arduino-nano)
//   Serial: 0 (RX), 1 (TX) (connected to FTDI)
//   External Interrupts: 2, 3
//   PWM: 3, 5, 6, 9
//   SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK)
//   I2C: A4 (SDA), A5 (SCL)

//   Pin definitions on the Arduino PCB v2.2
//   Arm Switch: D2
//   12V Enable : D3
//   Zero Switch: D4
//   Calibrate Switch: D5
//   Timer1 PWM: D9, D10
//   RX LED : A2
//   ARM LED: A3
//   Supply On : A7

//=============================================================================
// encoders
#define PIN_ENCODER_YAW_CS 8
#define PIN_ENCODER_PITCH_CS 6
#define PIN_ENCODER_ROLL_CS 7

// motor ESCs
#define PIN_MOTOR_LEFT_PWM 9    // timer1 PWM pin
#define PIN_MOTOR_RIGHT_PWM 10  // timer1 PWM pin

// arming switch
#define ARM_SWITCH 2          // Pins 2 and 3 are hardware interrupt pins
#define ZERO_SWITCH 4         // Pin to the button that should zero the encoders
#define CALIBRATION_SWITCH 5  // Pin to the button that should calibrate the ESCs

// Supply Voltage present
#define SUPPLY_ON_PIN A7 // Voltage divider from power supply on pin A2 (A7?)

// status LEDs
// NOTE: A6 and A7 are the only two pins that cannot be digital outputs, 
// they only function as analog inputs
#define LED_ARM A3
#define LED_RX A2
  
// setup arm switch debouncing
static constexpr unsigned long ARM_SWITCH_DEBOUNCE_PERIOD_MS = 50;
// calibration switch debouncing
Bounce2::Button calButton = Bounce2::Button();
Bounce2::Button zeroButton = Bounce2::Button();

#define PITCH_REFERENCE A5
#define YAW_REFERENCE A6



//=============================================================================
// global variables
//=============================================================================

// arm state
volatile bool armed;
volatile bool send_armed_status;
bool serial_debug = false;

//=============================================================================
void arm_switch_ISR()
{
  MsTimer2::start();
}

//=============================================================================
// ??
void debounce_timer_ISR()
{
  MsTimer2::stop();

  bool new_armed = digitalRead(ARM_SWITCH);
  if (new_armed != armed)
  {
    if (serial_debug) Serial.println("new_armed != armed");
    armed = new_armed;

    if (!armed)
    {
      if (serial_debug) Serial.println("Setting Motors to Zero (not armed)");
      //motor_left_setpoint = 0.0f;
      //motor_right_setpoint = 0.0f;
    }

    if (serial_debug) Serial.println("Updating motors");
    //update_motors(motor_left_setpoint, motor_right_setpoint);
    if (serial_debug) Serial.println("Digital write armed led");
    digitalWrite(LED_ARM, (armed ? HIGH : LOW));

    // set flag to send status message next time through loop
    send_armed_status = true;
  }
}

// Function to read pin voltage
float getPinVoltage(int pin) {
  return( 5.0 * ( (float) analogRead(pin) ) / 1024.0 );
}
