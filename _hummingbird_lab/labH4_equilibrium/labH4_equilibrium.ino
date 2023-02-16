#include "arduino_setup.h"
#include "time_utilities.h"
#include "sensor_utilities.h"
#include "motor_utilities.h"
#include "ctrlEquilibrium.h"

//=============================================================================
// declare global structures
//=============================================================================

// instantiate structures
State state;
Reference reference;

// instantiate classes as global variables
TimeUtilities timing;
SensorUtilities sensors;
MotorUtilities rotors;
CtrlEquilibrium control;


//=============================================================================
// arduino setup function (runs once at start of simulation)
//=============================================================================
void setup()
{
  // set up serial communication
  Serial.begin(9600);

  timing.init();  // initialize current time and sample rate
  sensors.init();  // initialize sensors
  control.init();  // initialize controller
  
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

//=============================================================================
// arduino loop function (loops forever)
//=============================================================================
void loop()
{
  timing.update();  // update current time and sample rate
  sensors.update();  // update sensors
  state.theta = sensors.pitch; // update the estimator
  control.update(state, rotors);  // update controller

  // zero encoders if zero button pushed
  zeroButton.update();  
  if ( zeroButton.pressed() ) { 
    sensors.zero(); 
  }
   
  // calibrate ESCs if calibrate button pushed
  calButton.update();
  if ( calButton.pressed() ){
    rotors.init();
  }
  
  // reset watchdog timer
  wdt_reset();
  digitalWrite(LED_RX, LOW);
}

//=============================================
// print state and references to the serial port
void printState() {
  // send references and signals to serial line
  Serial.print(reference.theta);
  Serial.print(",");
  Serial.print(state.theta);
  Serial.print(",");
  Serial.print(reference.psi);
  Serial.print(",");
  Serial.print(state.psi);
  Serial.print(",");
  Serial.print(reference.phi);
  Serial.print(",");
  Serial.println(state.phi);
}


