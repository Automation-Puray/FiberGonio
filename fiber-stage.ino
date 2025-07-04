
//Encoder
#include <SimpleFOC.h> //Version 2.3.0 - magnetic_sensor_i2c_example
//modified for MagnTek MT6701 hall based angle encoder sensor


//Stepper driver pins and parameters - Library Stepper(1.4.1) driver by Laurentiu Badea 
#include "BasicStepperDriver.h"
// #define MOTOR_STEPS 200
#define RPM 500
#define MICROSTEPS 8
#define SPEED LINEAR_SPEED
// #define DIR 8
// #define STEP 9
BasicStepperDriver stepper(200, 5, 6);  // (MOTOR_STEPS, DIR, STEP)



// Pins of in- and outputs
const int stepper_on = 4;

// variables
int serial_read = 0;
float resolution =268.05;
String message = "Stepper";


void setup() {
  // monitoring port
  Serial.begin(115200);
  Serial.setTimeout(0.1);


  // initialise Stepper driver
  stepper.begin(RPM, MICROSTEPS);
  stepper.setSpeedProfile(stepper.LINEAR_SPEED, 2000, 2000);


  // in and output
  //  pinMode(angle_stop, INPUT_PULLUP);  //reference switch
  pinMode(stepper_on, OUTPUT);        //enable the stepper
  //  pinMode(x_stop, INPUT_PULLUP);      //reference switch
  // pinMode(y_stop, INPUT_PULLUP);      //relay to power driver
}

void loop() {

  digitalWrite(stepper_on, HIGH);
  Serial.println(message);
  serial_read = 0;
  while (!Serial.available()) {  //identification of device for python function
    delay(500);
  }

  serial_read = Serial.readString().toInt();
  digitalWrite(stepper_on, LOW);
  stepper.move(round(serial_read*resolution));
  message = "ready";

  digitalWrite(stepper_on, HIGH);

  }

