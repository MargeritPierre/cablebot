// Control a stepper using variable microstepping

#include "MicroStepper.h"

const int stepPin = 9;
const int dirPin = 3 ;
const int enablePin = 4;
const int modePins[3] = {5, 6, 7};

// Stepper Parameters
float stepRange = 800.0 ;
float maximumSpeed = 4500.0 ;
float acceleration = 20000.0 ;
float targetSpeed = maximumSpeed;

// Sensor reading update
byte knobInputs[2] = {A4, A5};
float knobValues[2] = {0, 0};
float minValue = 0.01 ;
unsigned long sensorReadPeriod = 50000; // in microseconds
long lastSensorRead = micros();
float lastPosition = 0.0 ;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  stepper = MicroStepper(stepPin, dirPin, modePins);
  //stepper.setDivision(4) ;
  stepper.setMinDivision(4) ;
  //stepper.setMaxDivision(16) ;
  stepper.setMaxSpeed(maximumSpeed) ;
  stepper.setAcceleration(acceleration) ;
}

void loop() {
  // put your main code here, to run repeatedly:

  // Sensor reading & motor update
  long t = micros() ;
  if (1) {
  if (t - lastSensorRead > sensorReadPeriod) {

    // Retrieve knob values
    for (int i = 0; i < 1; i++) knobValues[i] = (float)(analogRead(knobInputs[i]) - 512) / 512.0 ;

    // Cut near-zero values
    for (int i = 0; i < 1; i++) if (abs(knobValues[i]) < minValue) knobValues[i] = 0.0 ;
    float val = knobValues[0];

    // For better speed control
    val = pow(val,3) ;

    // Change motor speed
    //Serial.println(stepper.getPosition()) ;
    targetSpeed = val * maximumSpeed ;
    stepper.setSpeed(targetSpeed) ;

    if (1 and Serial) {
      String msg = "";
      msg += "TargetSpeed: " + String(targetSpeed);
      msg += ", InstantSpeed: " + String(stepper.getSpeed());
      msg += ", TrueSpeed: " + String((1000000.0/float(t-lastSensorRead))*float(stepper.getPosition()-lastPosition));
      msg += ", Division: " + String(stepper.getDivision());
      //msg += ", Position: " + String(stepper.getPosition());
      Serial.println(msg) ;
    }

    lastSensorRead = t ;
    lastPosition = stepper.getPosition();

  }
  }
  else {
    if (stepper.getSpeed() == targetSpeed) {
      targetSpeed = -targetSpeed;
      stepper.setSpeed(targetSpeed);
    }
    else if (stepper.getSpeed() == 0.0) stepper.setSpeed(targetSpeed);
    Serial.println(stepper.getSpeed());
  }


  stepper.run() ;

}
