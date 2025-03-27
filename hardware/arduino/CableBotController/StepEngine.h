// DEFINES A STEPPING ENGINE BASE ON TIMER INTERRUPTS AND PORT MANIPULATION


#define CIRCULAR_BUFFER_INT_SAFE  // make circular buffers interrupt-compatible
#define STEP_BUFFER_SIZE 256
#define POS_BUFFER_SIZE 64
#define DEFAULT_TIMER_PERIOD 5000  // in microseconds, approximate!
#define TIMER_PERIOD_BITSHIFT 2    // bit shift for conversion between uint16_t period and unsigned long period in microseconds
#include "CircularBuffer.hpp"

// Stepping structure
class step_t {
public:
  uint8_t step = 0b00000000;                                                   // the STEP PORT state
  uint8_t dir = 0b00000000;                                                    // the DIR PORT state
  uint16_t period_bs = DEFAULT_TIMER_PERIOD / (1UL << TIMER_PERIOD_BITSHIFT);  // the corresponding bit-shifted period
  unsigned long getUSPeriod() {return ((unsigned long)period_bs) << TIMER_PERIOD_BITSHIFT;};  // function that returns the corresponding period in microseconds
  void setUSPeriod(unsigned long period_us) {period_bs = (uint16_t)(period_us >> TIMER_PERIOD_BITSHIFT);};  // function that sets the period_b256 from a period in micros
  float getSpeed() {return 1000000.0 / float(getUSPeriod());}
  void setSpeed(float speed) {setUSPeriod((unsigned long)(1000000.0 / speed));}
  void print();
};

void step_t::print() {
  Serial.print("STEP:");
  printOCTET(step);
  Serial.print("\t DIR:");
  printOCTET(dir);
  Serial.print("\t PERIOD:");
  Serial.print(period_bs, DEC);
  Serial.print("uint16");
  Serial.print('/');
  Serial.print(getUSPeriod(), DEC);
  Serial.print("us");
  Serial.print("\t SPEED:");
  Serial.print(getSpeed());
  Serial.print("steps/sec");
}

// Data Buffers
template<class T, size_t N>
class DataBuffer : public CircularBuffer<T, N> {
public:
  bool loop = false;  // is the buffer looping ?
  void print();
};

template<class T, size_t N>
void DataBuffer<T, N>::print() {
  Serial.println("=== BUFFER ===");
  for (unsigned long i = 0; i < size(); i++) {
    T item = this->operator[](i);
    Serial.print(i);
    Serial.print("\t");
    item.print();
    Serial.println();
  }
  Serial.println("======================");
};

// BUFFER CONTAINING A COLLECTION OF STEPS
typedef DataBuffer<step_t, STEP_BUFFER_SIZE> StepBuffer;

// Stepper motion structure: a position and a motion speed
class StepperMotion : public StepperPositions {
  using StepperData<long int>::StepperData;
public:
  float speed = MAX_SPEED;  // the MAXIMUM motion speed in steps/sec
  float acceleration = MAX_ACCELERATION;  // the motion speed in steps/sec
  StepperMotion(StepperData<long int> pos) {for (uint8_t i = 0; i < N_MOTORS; i++) this->operator[](i) = pos[i];};
  StepperMotion(StepperData<long int> pos, float speedArg) {for (uint8_t i = 0; i < N_MOTORS; i++) this->operator[](i) = pos[i];speed = speedArg;};
  StepperMotion(StepperData<long int> pos, float speedArg,float accArg) {for (uint8_t i = 0; i < N_MOTORS; i++) this->operator[](i) = pos[i];speed = speedArg;acceleration=accArg;};
  StepperMotion(Stream &S);
  StepperFloats getSpeeds() {StepperFloats dx = this->_float(); return (dx/(dx._abs()._max()+0.001))*speed;}; //0.001 to avoid division by zero!
  String to_String(char lbl = 'x') {return (StepperPositions::to_String(lbl) + "\tspeed:" + String(speed) + "\tacc:" + String(acceleration) + "\tspeeds:" + getSpeeds().to_String());};
  void print() {Serial.print(to_String());};
};
// Construct with additionnal parameters
StepperMotion::StepperMotion(Stream &S) : StepperPositions(S) {
  float speedFromSerial = S.parseFloat();
  if (speedFromSerial > 0.0) speed = speedFromSerial;
  float accFromSerial = S.parseFloat();
  if (accFromSerial > 0.0) acceleration = accFromSerial;
  print();Serial.println();
};

// Buffer containing a collection of stepper motions (target position and speed)
typedef DataBuffer<StepperMotion, POS_BUFFER_SIZE> MotionBuffer;

// STEPPER ENGINE CLASS
class StepEngine {
  public:
    StepBuffer stepBuffer; // buffer containing the next step structures to execute
    MotionBuffer motionBuffer; // buffer of the upcoming RELATIVE motions
    void setup();
    void setStepGenerationMode(char mode);  // change the step generation mode
    void step();                            // apply the next buffered step; executed by the timer interrupt
    void update();                          // update and fill the step buffer using the position buffer
    StepperPositions getCurrentPosition() {noInterrupts();StepperPositions cp = _currentPosition;interrupts();return cp;}
    void setCurrentPosition(StepperPositions p) {noInterrupts();_currentPosition = p;interrupts();return;}
    StepperPositions getTargetPosition() {return _targetPosition;}
    StepperPositions getLastPosition() {StepperPositions fp=_targetPosition; for (int i=0;i<motionBuffer.size();i++) fp = fp+motionBuffer[i]; return fp;};
    StepperMotion getLastMotion() {if (!motionBuffer.isEmpty()) return motionBuffer.last(); return _updateMotion;};
    void move(StepperMotion motion);
    void moveTo(StepperMotion positions);
  private:
    StepperPositions _currentPosition;       // the current motor step positions (the most up to date, updated by the timer interrupt)
    char _stepGenerationMode = 'M';          // stepping generation: M:from motion buffer, S:from step buffer
    StepperPositions _targetPosition;           // the current positions where the steppers are going
    StepperMotion _updateMotion ;//= StepperMotion(StepperPositions(0),0.0);           // the current positions where the steppers are going
    unsigned long _updateStepsToTarget = 0;  // how many update steps need to be pushed on the buffer to plan the target reach ?
    step_t _updateStep;                      // the current update step structure (allows to keep the direction byte and period/speed)
  private:                                   // bresenham line drawing algorithm used for step generation
    StepperPositions _N_bresenham;           // the total number of steps, by motor, to reach the target position
    long _max_N_bresenham;                   // the maximum number of steps for all motors to reach the target position
    StepperPositions _D_bresenham;           // the distance to the line (some error function)
    void bresenhamInit(StepperMotion dx);
    StepperBools bresenhamStep();
  private: // speed/acceleration handling see http://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
    float _updateSpeed = 0.0; // the speed of the current update step
    float _targetSpeed = 0.0; // the speed to which the update is currently approaching
    int _accelerationDir = 0; // the direction of acceleration increment
    float _motionEndSpeed = 0.0; // the speed that should be reached at the end of the currently updated motion
    unsigned long _updateStepsToTargetSpeedUpdate = 0; // how many steps need to be performed before changing the target speed ?
    unsigned long minSpeed(const float acceleration = MAX_ACCELERATION) {return sqrt(acceleration/2.0/0.676);};
    unsigned long nStepsToReachSpeed(const float targetSpeed,const float currentSpeed=0.0,const float acceleration = MAX_ACCELERATION) {float res = 0.5*(targetSpeed*targetSpeed - currentSpeed*currentSpeed)/acceleration; return (unsigned long)(abs(res)+1);};
};

// Define the unique stepper engine
StepEngine steppers;

// Function for the timer interrupt
void stepSteppers() {
  steppers.step();
};

void StepEngine::setup() {
  Timer1.initialize(DEFAULT_TIMER_PERIOD);
  Timer1.attachInterrupt(stepSteppers);
}

void StepEngine::setStepGenerationMode(char mode) {
  // change the step generation mode
  if (mode == _stepGenerationMode) return;
  switch (mode) {
    case 'M': {  // to motionBuffer mode
        stepBuffer.loop = false;
        while (!stepBuffer.isEmpty()); // wait for the step buffer to be empty
        _targetPosition = _currentPosition; // set the current target position to the current position for next moves
      }; break;
    case 'S': {
        motionBuffer.loop = false;
        while (!motionBuffer.isEmpty()) update(); // empty the motion buffer
        while (!stepBuffer.isEmpty()); // wait for the step buffer to be empty
      }; break;
  }
  _stepGenerationMode = mode;
}

void StepEngine::move(StepperMotion motion) {
  // move relatively to the last planned positions
  setStepGenerationMode('M');
  while (!motionBuffer.available()) steppers.update();  // wait for the positions buffer to have free space
  // StepperBools rotationInversed = (motion.getSpeeds().sign())._int() != (getLastMotion().getSpeeds().sign())._int();
  // if (rotationInversed.any()) {
    // float midSpeed = min(MAX_IMMEDIATE_SPEED_CHANGE/2,0.5*(motion.speed+getLastMotion().speed));
    // Serial.println("ghost motion added with speed="+String(midSpeed));
    // motionBuffer.push(StepperMotion(StepperPositions(0),midSpeed));
  // }
  motionBuffer.push(motion);
};

void StepEngine::moveTo(StepperMotion positions) {
  // move relatively to the last planned positions
  setStepGenerationMode('M');
  StepperMotion motion = StepperMotion(positions - getLastPosition(), positions.speed, positions.acceleration);
  move(motion);
};

void StepEngine::update() {
  if (_stepGenerationMode == 'S') return;  // will not update the buffer if step generation is based on it
  // delay(200);
  while (stepBuffer.available()) {  // while there is some room on the step buffer...
    // Serial.println("Step Engine Update...");
    if (_updateStepsToTarget == 0) {  // there has been enough updates to reach the current target position
      // Serial.println("  target position reached!");
      _updateSpeed = _motionEndSpeed; // correct some bias..
      if (motionBuffer.isEmpty()) { // no next position to go!
        _updateSpeed = 0.0;
        return;                        
      }
      _updateMotion  = motionBuffer.shift();                     // extract the next motion from the motion buffer
      if (motionBuffer.loop) motionBuffer.push(_updateMotion);  // IF LOOPING, RE-PUSH the reached target position at the end of the buffer
      _targetPosition = _targetPosition + _updateMotion; // update the target position
      // Serial.println("  newMotion: "+_updateMotion.to_String());
      // Serial.println("  new target position: "+_targetPosition.to_String());
      // Compute motion-constant data
      _updateStep.dir = StepperBool2Byte(_updateMotion > 0);  // constant direction
      bresenhamInit(_updateMotion);                        // initialize the bresenham stepping scheme
      _updateStepsToTarget = _max_N_bresenham;                               // initialize the number of updates to perform
      // Init speed infos
      _updateSpeed = max(_updateSpeed,minSpeed(_updateMotion.acceleration)); // prevent any negative or zero step update speed
      _motionEndSpeed = 0.0; // will be updated later if needed, see below
      _targetSpeed = _updateMotion.speed;
      _updateStepsToTargetSpeedUpdate = 0; // will trigger its computation on the next update, see below
      // Serial.println("  update steps to target: "+String(_updateStepsToTarget));
      // Speed handling
    };
    while (stepBuffer.available() && _updateStepsToTarget != 0) {  // while there is some room on the step buffer...
      // Push a new step to the step buffer
      StepperBools needsStep = bresenhamStep();  // which motor needs to be stepped ?
      _updateStep.step = StepperBool2Byte(needsStep);
      _updateStep.setSpeed(_updateSpeed);
      // Serial.print("update step: ");_updateStep.print();Serial.println();
      noInterrupts();
      stepBuffer.push(_updateStep);
      interrupts();
      // An update step has been added
      _updateStepsToTarget--;
      // update speed
        // unsigned long nStepsToMotionSpeed = nStepsToReachSpeed(_targetPosition.speed,_updateSpeed,_targetPosition.acceleration);
        // Serial.println("nStepsToMotionSpeed: "+String(nStepsToMotionSpeed));
      if (!motionBuffer.isEmpty() && _motionEndSpeed==0.0) { // Has a new motion been added to the buffer ?
        if (motionBuffer.first().speed>0.0) {
          // Serial.println("motionBuffer.first(): "+motionBuffer.first().to_String());
          _motionEndSpeed = 0.5*(motionBuffer.first().speed+_updateMotion.speed); 
          // Serial.println("_motionEndSpeed:"+String(_updateMotion.speed)+"|"+String(_motionEndSpeed)+"|"+String(motionBuffer.first().speed));
          _updateStepsToTargetSpeedUpdate = 0; // this needs to be updated below
          // Serial.println("targetspeed update needed!");
        }
        // Serial.println("_updateStepsToTarget:"+String(_updateStepsToTarget));
      };
      // Serial.println();
      // Serial.println("updateStepsToTargetSpeedUpdate:"+String(_updateStepsToTargetSpeedUpdate));
      if (_updateStepsToTargetSpeedUpdate==0) { // acceleration & step numbers need to be updated
        // Serial.println("== target speed update ==");
        // Serial.println("_updateSpeed:"+String(_updateSpeed));
        unsigned long nStepsToTargetSpeed = nStepsToReachSpeed(_targetSpeed,_updateSpeed,_updateMotion.acceleration);
        unsigned long nStepsToMotionEndSpeed = nStepsToReachSpeed(_motionEndSpeed,_updateSpeed,_updateMotion.acceleration);
        // Serial.println("motionEndSpeed:"+String(_motionEndSpeed));
        // Serial.println("nStepsToTargetSpeed:"+String(nStepsToTargetSpeed));
        // Serial.println("nStepsToMotionEndSpeed:"+String(nStepsToMotionEndSpeed));
        // Serial.println("_updateStepsToTarget:"+String(_updateStepsToTarget));
        if (nStepsToMotionEndSpeed>=_updateStepsToTarget) { // we are getting close to the target
          // Serial.println("  to motionEndSpeed");
          _targetSpeed = _motionEndSpeed;
          _updateStepsToTargetSpeedUpdate = nStepsToMotionEndSpeed;
        } else if (nStepsToTargetSpeed<=1) { // we reached the motion speed, let's stop accelerating
          _updateSpeed = _targetSpeed;
          // Serial.println("  keep targetSpeed");
          _updateStepsToTargetSpeedUpdate = _updateStepsToTarget - nStepsToMotionEndSpeed; // next acceleration update will happen when we get close to the target
        }
        else { // we need to accelerate to the targetspeed
          unsigned long nStepsFromTargetToEndMotionSpeeds = nStepsToReachSpeed(_motionEndSpeed,_targetSpeed,_updateMotion.acceleration);
          // Serial.println("nStepsFromTargetToEndMotionSpeeds:"+String(nStepsFromTargetToEndMotionSpeeds));
          if ((nStepsToTargetSpeed+nStepsFromTargetToEndMotionSpeeds)>_updateStepsToTarget) { // we need to adjst the targetspeed so that we will have the time to deccelerate after
            float ts = sqrt( _updateMotion.acceleration*float(_updateStepsToTarget) + _updateSpeed*_updateSpeed + _motionEndSpeed*_motionEndSpeed);
            _targetSpeed = min(_targetSpeed,ts);
            // Serial.println("targetSpeed adjusted to "+String(_targetSpeed));
          }
          // Serial.println("  accelerate to targetSpeed="+String(_targetSpeed));
          _updateStepsToTargetSpeedUpdate = nStepsToReachSpeed(_targetSpeed,_updateSpeed,_updateMotion.acceleration);
        }
        // Serial.println("updateStepsToTargetSpeedUpdate:"+String(_updateStepsToTargetSpeedUpdate));
        float speedDiff = _targetSpeed - _updateSpeed;
        _accelerationDir = int(speedDiff>0.0)-int(speedDiff<0.0); // acceleration direction: compute the sign of speedDiff
      }
      if (_accelerationDir!=0)
        if (_accelerationDir>0) _updateSpeed = min(_updateSpeed+(_updateMotion.acceleration/_updateSpeed),_targetSpeed);
        else _updateSpeed = max(_updateSpeed-(_updateMotion.acceleration/_updateSpeed),_targetSpeed);
      _updateStepsToTargetSpeedUpdate--;
        // Serial.println("updateStepsToTargetSpeedUpdate:"+String(_updateStepsToTargetSpeedUpdate));
        // Serial.println("updateStepsToTarget:"+String(_updateStepsToTarget));
      //   // Serial.println("speedAtTarget: "+String(_motionEndSpeed));
      //   unsigned long nStepsToMotionEndSpeed = nStepsToReachSpeed(_motionEndSpeed,_updateSpeed,_targetPosition.acceleration);
      //   // Serial.println("nStepsToMotionEndSpeed: "+String(nStepsToMotionEndSpeed));
      //   float nextSpeed = _targetPosition.speed;
      //   if (nStepsToMotionEndSpeed >= _updateStepsToTarget) nextSpeed = _motionEndSpeed;
      //   if (nextSpeed>_updateSpeed) _updateSpeed = min(nextSpeed,_updateSpeed + _targetPosition.acceleration/_updateSpeed);
      //   else _updateSpeed = max(nextSpeed,_updateSpeed - _targetPosition.acceleration/_updateSpeed);
      //   // Serial.println("nextSpeed: "+String(nextSpeed));
      // // Serial.print("UpdateStep:");_updateStep.print(); Serial.println();
    };
    // Serial.println("OUT OF UPDATE LOOP");
  };
};

// Bresenham step
void StepEngine::bresenhamInit(StepperMotion dx) {
  _N_bresenham = dx._abs();
  _max_N_bresenham = _N_bresenham._max();
  _D_bresenham = _N_bresenham * 2 - _max_N_bresenham;
};

// Bresenham step
StepperBools StepEngine::bresenhamStep() {
  StepperBools needsStep = _D_bresenham > 0;
  for (uint8_t i = 0; i < N_MOTORS; i++) if (needsStep[i]) _D_bresenham[i] -= _max_N_bresenham * 2;
  _D_bresenham = _D_bresenham + _N_bresenham * 2;
  return needsStep;
};

// ENGINE RUN FUNCTION
void StepEngine::step() {
  // keep track of the number of interrupts
  // RETRIEVE THE NEXT STEP
  if (stepBuffer.isEmpty()) return;
  step_t nextStep = stepBuffer.shift();  // this might take too much time..
  // SET PORT STATES; DIR before STEP
  PORT_DIR = nextStep.dir;
  if (DEDGE)                     // double edge mode, only toggle the step port
    PORT_STEP ^= nextStep.step;  // toggles stepping pins
  else {                         // step only on RISING edges, so a full PWM needs to be generated
    PORT_STEP = nextStep.step;   // set pins HIGH
    delayMicroseconds(1);        // wait
    PORT_STEP = 0b00000000;      // // set pins LOW
  };
  // MODIFY THE TIMER PERIOD
  Timer1.setPeriod(nextStep.getUSPeriod());
  // IF LOOPING, RE-PUSH the current step at the end of the buffer
  if (stepBuffer.loop) stepBuffer.push(nextStep);
  // Update the current positions
  for (int8_t b = 0; b < 8; b++)
    if (bitRead(nextStep.step, b)) _currentPosition[b] += (bitRead(nextStep.dir, b) ? 1 : -1);
  // Is the buffer empty ?
  if (stepBuffer.isEmpty()) Timer1.setPeriod(DEFAULT_TIMER_PERIOD);
}

// BUFFER FROM SERIAL
void readHEXStepBufferFromSerial() {
  steppers.setStepGenerationMode('S');
  char msg[4];
  while (Serial.available()) {
    int nvalid = Serial.readBytes(msg, 4);
    if (nvalid < 4) return;
    step_t newStep;
    newStep.step = hex2byte(msg[0], msg[1]);  //readByteFrom2hex();
    newStep.dir = hex2byte(msg[2], msg[3]);   //readByteFrom2hex();
    while (!steppers.stepBuffer.available())
      ;  // wait for the buffer to have free space
    steppers.stepBuffer.push(newStep);
  }
};


// SAMPLE STEP BUFFERS
// Ramp buffer (accelerates to target speed then decelerate to stanstill)
void generateRampStepBuffer(float maxspeed = MAX_SPEED, float acceleration = MAX_ACCELERATION, uint16_t length = STEP_BUFFER_SIZE) {
  steppers.setStepGenerationMode('S');
  step_t newStep;
  newStep.step = 0b11111111;         // all motors turn
  newStep.dir = 0b11111111;          // in the same direction
  float speed = sqrt(acceleration/2.0);  // in steps/seconds
  uint16_t rampSteps = 1;            // will keep the number of steps needed to accelerate/deccelerate
  for (unsigned long i = 0; i < length; i++) {
    speed = min(speed, maxspeed);
    newStep.setUSPeriod((unsigned long)(1000000.0 / speed));
    steppers.stepBuffer.push(newStep);
    if (speed < maxspeed or i > length - rampSteps - 1) {
      speed += acceleration / speed;
      rampSteps++;
    }
    if (speed>maxspeed) Serial.println("True Ramp Steps: "+String(rampSteps));
    if (i == length / 2) acceleration *= -1.0;  // start decelerating at half the buffer
  };
  Serial.println("Predicted Ramp Steps: "+String(maxspeed*maxspeed/acceleration/2.0));
};


// SAMPLE POSITION BUFFERS
void generateSinMotion() {
  // Generate sinusoidal motion
  // steppers.motionBuffer.loop = false;
  // while (!steppers.positonsBuffer.isEmpty()) steppers.update(); // empty the current position buffer
  steppers.setStepGenerationMode('M');
  steppers.motionBuffer.clear();  // empty the current motion buffer
  steppers.motionBuffer.loop = true;
  const float maxSpeed = 1000;  //MAX_SPEED;
  const float period = 3.0;
  const float dt = period / float(POS_BUFFER_SIZE);
  StepperMotion nextPos;
  for (uint8_t t = 0; t < POS_BUFFER_SIZE; t++) {
    float speed = maxSpeed * sin(2.0 * 3.141651 * float(t) / float(POS_BUFFER_SIZE));
    nextPos.speed = abs(speed);
    for (uint8_t i = 0; i < N_MOTORS; i++) nextPos[i] += long(speed * dt);
    steppers.moveTo(nextPos);
  }
  steppers.motionBuffer.print();
};
void generateBounceMotion() {
  // Generate sinusoidal motion
  // steppers.motionBuffer.loop = false;
  // while (!steppers.positonsBuffer.isEmpty()) steppers.update(); // empty the current position buffer
  steppers.setStepGenerationMode('M');
  steppers.motionBuffer.clear();  // empty the current motion buffer
  steppers.motionBuffer.loop = true;
  long nSteps = 3000;
  float speed = MAX_SPEED;
  float acceleration = MAX_ACCELERATION;
  steppers.move(StepperMotion(StepperPositions()+nSteps,speed,acceleration));
  steppers.move(StepperMotion(StepperPositions()-nSteps,speed,acceleration));
  steppers.motionBuffer.print();
};
