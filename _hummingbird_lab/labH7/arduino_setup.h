#include <Arduino.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <SPI.h>
#include <avr/wdt.h>
#include <Bounce2.h>

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
//   Joystick A4, A5, A6

//=============================================================================
// encoders are attached to these digital pins
#define PIN_ENCODER_YAW_CS 8
#define PIN_ENCODER_PITCH_CS 7
#define PIN_ENCODER_ROLL_CS 6

// motor ESCs (electronic speed controllers) are attached to these pins
#define PIN_MOTOR_LEFT_PWM 9    // timer1 PWM pin
#define PIN_MOTOR_RIGHT_PWM 10  // timer1 PWM pin

// arming switch
The arm, zero, and calibration buttons are attached to these pins
#define ARM_SWITCH 2          // Pins 2 and 3 are hardware interrupt pins
#define ZERO_SWITCH 4         // Pin to the button that should zero the encoders
#define CALIBRATION_SWITCH 5  // Pin to the button that should calibrate the ESCs

// Supply Voltage present
// I don't know if this is being used
#define SUPPLY_ON_PIN A7 // Voltage divider from power supply on pin A2 (A7?)

// status LEDs
// NOTE: A6 and A7 are the only two pins that cannot be digital outputs, 
// they only function as analog inputs
// these drive the LEDs that indicate the status of the arduino
#define LED_ARM A3
#define LED_RX A2
  
// setup arm switch debouncing for the zero and calibration buttons
static constexpr unsigned long ARM_SWITCH_DEBOUNCE_PERIOD_MS = 50;
// calibration switch debouncing
Bounce2::Button calButton = Bounce2::Button();
Bounce2::Button zeroButton = Bounce2::Button();

// The joystick is attached to these pins
#define JOYSTICK_PUSH A4
#define JOYSTICK_UPDOWN A5
#define JOYSTICK_SIDESIDE A6

// We use PI for display purposes
#define PI 3.14159



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

    digitalWrite(LED_ARM, (armed ? HIGH : LOW));

    // set flag to send status message next time through loop
    send_armed_status = true;
  }
}

//=============================================================================
// Function to read pin voltage
float getPinVoltage(int pin) {
  return( 5.0 * ( (float) analogRead(pin) ) / 1024.0 );
}

//=============================================================================
// Function to initialize buttons on breakout board
void initialize_buttons() {
    // initialize armed state
  armed = false;
  send_armed_status = false;

  // set up arming switch
  pinMode(ARM_SWITCH, INPUT);
  attachInterrupt(digitalPinToInterrupt(ARM_SWITCH), arm_switch_ISR, CHANGE);

  // set up arming switch debouncing
  MsTimer2::set(ARM_SWITCH_DEBOUNCE_PERIOD_MS, debounce_timer_ISR);
  
  // setup calibration debouncing
  calButton.attach(CALIBRATION_SWITCH, INPUT);
  calButton.interval(100);
  calButton.setPressedState(HIGH);
  
  // setup zeroing button debouncing
  zeroButton.attach(ZERO_SWITCH, INPUT);
  zeroButton.interval(100);
  zeroButton.setPressedState(HIGH);

  // set up status LEDs
  pinMode(LED_ARM, OUTPUT);
  pinMode(LED_RX, OUTPUT);
  digitalWrite(LED_ARM, LOW);
  digitalWrite(LED_RX, LOW);

  // enable watchdog timer
  wdt_enable(WDTO_60MS);
}
