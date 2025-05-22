/*
   Implementation of a stepper motor library with variable microstepping
   The stepping is performed via a variable-period pwm using a timer (TimerOne lib)
   The PWM duty (absolute time HIGH) is fixed
*/

#ifndef MicroStepper_h
#define MicroStepper_h

#include "Arduino.h"
#include "TimerOne.h"

#define TOL 0.0001 // for floating point tests
#define DUTY 2 // pwm duty time in microseconds

class MicroStepper
{
  public:
    // Constructors
    MicroStepper(); // default
    MicroStepper(int stepPin, int dirPin, int modePins[3]);
    // Update function
    void run(); // update the motor speed (should be called in loop())
    // Motion commands
    void move(float distance); // move the motor by signed distance in full steps
    void moveTo(float position); // move the motor to position in full steps
    float getPosition(); // return the motor position (in full steps)
    // Speed control
    float getSpeed(); // return the speed in full steps/s
    void setSpeed(float speed); // set the target motor speed in full steps/sec
    void setInstantSpeed(float speed); // set the motor speed instantaneously in full steps/sec
    void setMaxSpeed(float maxSpeed); // set the maximum speed in full steps/sec
    void setAcceleration(float accel); // set the maximum acceleration in full steps/sec²
    // Microstep division
    void setDivision(int div); // set the microstep division
    int getDivision(); // return the current division
    int autoDivision(float speed); // return the optimal division for a given speed
    void setMinDivision(int div); // Set the minimum microstep division
    void setMaxDivision(int div); // Set the maximum microstep division
  private:
    // Pin definition
    int _stepPin;
    int _dirPin;
    int _modePins[3];
    // Speed
    float _targetSpeed = 0.0; 
    float _speed = 0.0;
    float _maxSpeed = 0.0;
    void updateSpeed();
    // Acceleration
    float _acceleration = 0.0;
    float _speedIncrementByMicros = 0.0; // precompute the speed increment
    // Microstepping position
    volatile long _currentMicroStep = 0; // the current micro step position
    // Microstep division
    bool _autoSetDivision = true; // automatically set the microstep division
    int _currentDivision = 0; // the current microstep division
    int _maxDivision = 32; // maximum microstep division
    int _minDivision = 1; // minimum microstep division
    // Microstep timing
    unsigned long _lastRunMicros; // last time a speed update was performed
    void setMicroStepTiming(); // set the private microstep timing properties
    volatile unsigned long _microStepPeriod; // the period between two microsteps (in microseconds)
    volatile int _microStepDirection; // direction of the microstepping (motor speed)
    // Pin setting
    bool _changePinMode = true; // Is a writeModePins needed ? (on disivion change)
    void writeMode(bool m0, bool m1, bool m2); // write a microstepping mode
    void writeModePins(int div); // write a microstepping mode
    static void step(); // step the motor (timer interrupt)
};
    
// Global declaration (to make it available for the timer interrupt)
extern MicroStepper stepper;

#endif
