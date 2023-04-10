#include "arduino_setup.h"
#include "time_utilities.h"
#include "sensor_utilities.h"
#include "motor_utilities.h"
#include "reference_utilities.h"
#include "ctrlStateFeedbackIntegrator.h"

//=============================================================================
// declare global structures
//=============================================================================

// instantiate classes as global variables
TimeUtilities timing;
SensorUtilities sensors;
MotorUtilities rotors;
ReferenceUtilities pitch_reference;
ReferenceUtilities yaw_reference;
CtrlStateFeedbackIntegrator controller;

//=============================================================================
// arduino setup function (runs once at start of simulation)
//=============================================================================
void setup()
{
  // start serial communication
  Serial.begin(19200);

  // initialize all classes
  timing.init();  // initialize current time and sample rate
  sensors.init();  // initialize sensors
  controller.init();  // initialize controller
  pitch_reference.init(15*3.14/180, 0.1, 0.0);
  yaw_reference.init(30*3.14/180, 0.02, 0.0);

  // initialize buttons on breakout board and watchdog timers
  initialize_buttons();
  Serial.println("Hi");
}

//=============================================================================
// arduino loop function (loops forever)
//=============================================================================
void loop()
{
  Reference reference;
  timing.update();  // update current time and sample rate
  sensors.update();  // update sensors
  reference.psi = yaw_reference.square_signal(timing.current);
  reference.theta = pitch_reference.square_signal(timing.current);
  //reference.psi = yaw_reference.joystick_yaw();
  //reference.theta = pitch_reference.joystick_pitch();
  controller.update(reference, sensors, rotors, timing.Ts);  

  // zero encoders if zero button pushed
  zeroButton.update();  
  if ( zeroButton.pressed() ) sensors.zero(); 
   
  // calibrate rotors if calibrate button pushed
  calButton.update();
  if ( calButton.pressed() ) rotors.init();
  
  // reset watchdog timer
  wdt_reset();
  digitalWrite(LED_RX, LOW);
}
