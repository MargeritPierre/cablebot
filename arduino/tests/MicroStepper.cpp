#include "Arduino.h"
#include "MicroStepper.h"

// Global declaration (to make it available for the timer interrupt)
MicroStepper stepper;

// Class constructors: empty object
MicroStepper::MicroStepper(){}

// Class constructors: full definition
MicroStepper::MicroStepper(int stepPin, int dirPin, int modePins[3]){
  _stepPin = stepPin;
  _dirPin = dirPin;
  for (int p=0;p<=3;p++) _modePins[p] = modePins[p];
  // Set pin modes
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  // Set pin states
  pinMode(_stepPin, LOW);
  pinMode(_dirPin, LOW);
  for (int p=0;p<=3;p++) pinMode(_modePins[p],OUTPUT);
  // Initialize the timer
  Timer1.initialize();
  Timer1.pwm(_stepPin,DUTY,1024);
  Timer1.stop();
  Timer1.attachInterrupt(step);
  // Set to maximum division
  setDivision(_currentDivision);
  // Initialise speed info to zero
  _lastRunMicros = micros();
  setSpeed(0.0);
}


// Return the fullstep position
float MicroStepper::getPosition() {
  if (_currentMicroStep==0) return 0.0 ; // prevent division by zero
  return float(_currentMicroStep)/float(_currentDivision);
}


// Move the motor by signed distance in full steps
void move(float distance){};


// Move the motor to a position in full steps
void moveTo(float position){};


// Get/Set the motor speed (in full steps/sec)
float MicroStepper::getSpeed() {return _speed;}

// Set the motor target speed in full steps/sec
void MicroStepper::setSpeed(float speed) {_targetSpeed = constrain(speed,-_maxSpeed,_maxSpeed);}

// Set the motor instantaneous speed in full steps/sec
void MicroStepper::setInstantSpeed(float speed) {
  // Set direction pin if the speed sign has changed
  if ((speed>0.0 && _speed<=0.0) || (speed<=0.0 && _speed>0.0)) 
    digitalWrite(_dirPin,(_speed>0.0 ? HIGH : LOW));
  // Set the private variable
  _speed = speed;
  // Limit with maximum speed
  if (_maxSpeed>TOL) _speed = constrain(_speed,-_maxSpeed,_maxSpeed);
  // Do you need to change the microstep division ?
  if (_autoSetDivision && _maxSpeed>TOL) setDivision(0);
  // Set the microstep timing properties
  else setMicroStepTiming();
};

// Set the motor maximum speed in full steps/sec
void MicroStepper::setMaxSpeed(float maxSpeed){
  _maxSpeed = maxSpeed;
  // Re-set the motor speed if it is too high
  if (_maxSpeed>TOL and abs(_speed)>_maxSpeed) setSpeed(_speed);
} 

// Set the maximum acceleration in full steps/sec²
void MicroStepper::setAcceleration(float accel) {
  _acceleration = accel;
  _speedIncrementByMicros = _acceleration/1000000.0;
}

// Update the motor speed
void MicroStepper::updateSpeed() {
  // If speed equals target speed, do nothing
  //if (_speed==_targetSpeed) return;
  if (_acceleration>TOL) {
    // Maximum speed increment
    unsigned long t = micros();
    float maxSpeedIncrease = _speedIncrementByMicros*(float)(t-_lastRunMicros);
    _lastRunMicros = t;
    // Constrain target speed
    float newSpeed = constrain(_targetSpeed,_speed-maxSpeedIncrease,_speed+maxSpeedIncrease);
    // Set the new speed
    setInstantSpeed(newSpeed);
  }
  else setInstantSpeed(_targetSpeed);
}


// Set the private microstep timing properties
void MicroStepper::setMicroStepTiming() {
  bool timerRunning = _microStepDirection!=0; 
  if (_speed<TOL && _speed>-TOL) {
    if (timerRunning) Timer1.stop();
    _microStepPeriod = 0;
    _microStepDirection = 0;
  }
  else {
    _microStepPeriod = (unsigned long)(1000000.0/abs(_speed)/float(_currentDivision));
    _microStepDirection = (_speed>0.0 ? 1 : -1);
    if (!timerRunning) {
      Timer1.setPeriod(_microStepPeriod);
      Timer1.start();
    }
  }
}


// Determine the best division with a given speed
int MicroStepper::autoDivision(float speed) {
    if (speed<TOL && speed>-TOL) return _maxDivision;
    float relSpeed = _maxSpeed/abs(speed);
    for (int p=_minDivision;p<=_maxDivision;p*=2) if (relSpeed<2*p) return p;
    return _maxDivision;
}

// Get the microstep division
int MicroStepper::getDivision() {return _currentDivision;}

// Set the microstep division
void MicroStepper::setDivision(int div){
  // Re-enable the autosetdivision ?
  _autoSetDivision = div==0; 
  // Auto-compute division ?
  if (_autoSetDivision) div = autoDivision(_speed);
  // Set the pins
  //writeModePins(div);
  // Re-set the microstep timing properties
  setMicroStepTiming();
  // Has the division changed ?
  if (div==_currentDivision) return ; 
  // Compute the new microstep position
  _currentMicroStep = long(getPosition()*div);
  _currentDivision = div;
  _changePinMode = true;
}

// Set the minimum microstep division
void MicroStepper::setMinDivision(int div) {_minDivision = min(div,_maxDivision);}

// Set the maximum microstep division
void MicroStepper::setMaxDivision(int div) {_maxDivision = max(div,_minDivision);} 


// Run the motor (should be called in loop())
void MicroStepper::run(){
  // Update the motor speed
  updateSpeed();
};

// Step the motor in a given direction
static void MicroStepper::step() {
  // Increment the current microstep position
  stepper._currentMicroStep += stepper._microStepDirection ;
  // Change the timer period if needed
  if (stepper._microStepPeriod!=0) {
    Timer1.setPeriod(stepper._microStepPeriod) ;
    stepper._microStepPeriod = 0;
  }
  // Change the pin mode if needed
  if (stepper._changePinMode) {
    stepper.writeModePins(stepper._currentDivision);
    stepper._changePinMode = false;
  }
}

// Write mode pin state corresponding to a division
void MicroStepper::writeModePins(int div) {
  switch (div){
    case 1: 
      digitalWrite(_modePins[0],LOW);
      digitalWrite(_modePins[1],LOW);
      digitalWrite(_modePins[2],LOW);
      return;
    case 2:
      digitalWrite(_modePins[0],HIGH);
      digitalWrite(_modePins[1],LOW);
      digitalWrite(_modePins[2],LOW);
      return;
    case 4:
      digitalWrite(_modePins[0],LOW);
      digitalWrite(_modePins[1],HIGH);
      digitalWrite(_modePins[2],LOW);
      return;
    case 8:
      digitalWrite(_modePins[0],HIGH);
      digitalWrite(_modePins[1],HIGH);
      digitalWrite(_modePins[2],LOW);
      return;
    case 16:
      digitalWrite(_modePins[0],LOW);
      digitalWrite(_modePins[1],LOW);
      digitalWrite(_modePins[2],HIGH);
      return;
    case 32:
      digitalWrite(_modePins[0],HIGH);
      digitalWrite(_modePins[1],LOW);
      digitalWrite(_modePins[2],HIGH);
      return;
    default: return;
  }
}
