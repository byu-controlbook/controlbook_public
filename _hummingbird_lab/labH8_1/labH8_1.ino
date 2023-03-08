#include "arduino_setup.h"
#include "time_utilities.h"
#include "sensor_utilities.h"
#include "motor_utilities.h"
#include "reference_utilities.h"
#include "ctrlLonPID.h"

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
ReferenceUtilities signal_generator;
CtrlLonPID controller;

//=============================================================================
// arduino setup function (runs once at start of simulation)
//=============================================================================
void setup()
{
  // set up serial communication
  Serial.begin(9600);

  timing.init();  // initialize current time and sample rate
  sensors.init();  // initialize sensors
  controller.init();  // initialize controller
  signal_generator.init(15*3.14/180, 0.1, 0.0);

  initialize_buttons();  
}

//=============================================================================
// arduino loop function (loops forever)
//=============================================================================
void loop()
{
  timing.update();  // update current time and sample rate
  sensors.update();  // update sensors
  state.theta = sensors.pitch; // update the estimator
  float theta_ref = signal_generator.square_signal(timing.current);
  //float theta_ref = signal_generator.joystick_pitch();
  controller.update(theta_ref, state, rotors, timing.Ts);  // update controller

  // zero encoders if zero button pushed
  zeroButton.update();  
  if ( zeroButton.pressed() ) sensors.zero();    
  
  // calibrate ESCs if calibrate button pushed
  calButton.update();
  if ( calButton.pressed() ) rotors.init();
  
  // reset watchdog timer
  wdt_reset();
  digitalWrite(LED_RX, LOW);
}

//=============================================
// print state and references to the serial port
void printState() {
  // send references and signals to serial line
  Serial.print("Theta_ref: ");
  Serial.print(reference.theta);
  Serial.print(",");
  Serial.print("theta: ");
  Serial.print(state.theta);
  Serial.print(",");
  Serial.print("Psi_ref: ");
  Serial.print(reference.psi);
  Serial.print(",");
  Serial.print("psi: ");
  Serial.print(state.psi);
  Serial.print(",");
  Serial.print("Phi_ref: ");
  Serial.print(reference.phi);
  Serial.print(",");
  Serial.print("phi: ");
  Serial.println(state.phi);
}
