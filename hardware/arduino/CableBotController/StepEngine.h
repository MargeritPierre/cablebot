// DEFINES A STEPPING ENGINE BASE ON TIMER INTERRUPTS AND PORT MANIPULATION


#define CIRCULAR_BUFFER_INT_SAFE // make circular buffers interrupt-compatible
#define STEP_BUFFER_SIZE 512
#define POS_BUFFER_SIZE 64
#define DEFAULT_TIMER_PERIOD 5000 // in microseconds, approximate!
#define TIMER_PERIOD_BITSHIFT 4 // bit shift for conversion between uint16_t period and unsigned long period in microseconds
#include "CircularBuffer.hpp"

// Stepping structure
class step_t {
  public:
    uint8_t step = 0b00000000; // the STEP PORT state
    uint8_t dir = 0b00000000; // the DIR PORT state
    uint16_t period_bs = DEFAULT_TIMER_PERIOD/(1UL << TIMER_PERIOD_BITSHIFT); // the corresponding bit-shifted period
    unsigned long getUSPeriod() {return ((unsigned long)period_bs) << TIMER_PERIOD_BITSHIFT;}; // function that returns the corresponding period in microseconds
    void setUSPeriod(unsigned long period_us) {period_bs = (uint16_t)(period_us >> TIMER_PERIOD_BITSHIFT);}; // function that sets the period_b256 from a period in micros
    void print();
};

void step_t::print() {
  Serial.print("STEP:");
  printOCTET(step);
  Serial.print("\t DIR:");
  printOCTET(dir);
  Serial.print("\t PERIOD:");
  Serial.print(period_bs,DEC);
  Serial.print("uint16");
  Serial.print('/');
  Serial.print(getUSPeriod(),DEC);
  Serial.print("us");
}

// Data Buffers
template <class T,size_t N>
class DataBuffer : public CircularBuffer<T,N> {
  public:
    bool loop = false; // is the buffer looping ?
    void print();
};

template <class T,size_t N>
void DataBuffer<T,N>::print() {
  Serial.println("=== BUFFER ===");
  for (unsigned long i=0;i<size();i++) {
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
    StepperMotion(StepperData<long int> pos) {for(uint8_t i=0;i<N_MOTORS;i++) this->operator[](i) = pos[i];};
    float speed = 1000000.0/DEFAULT_TIMER_PERIOD ; // the motion speed in steps/sec
};
// Buffer containing a collection of stepper motions (target position and speed)
typedef DataBuffer<StepperMotion, POS_BUFFER_SIZE> MotionBuffer;
typedef DataBuffer<StepperPositions, POS_BUFFER_SIZE> motionBuffer;

// STEPPER ENGINE CLASS
class StepEngine {
  public:
    StepBuffer stepBuffer;
    MotionBuffer motionBuffer;
    void setup();
    void setStepGeneration(char mode); // change the step generation mode
    void step(); // apply the next buffered step; executed by the timer interrupt
    void update(); // update and fill the step buffer using the position buffer
    StepperPositions getCurrentPosition() {noInterrupts(); StepperPositions cp = _currentPosition; interrupts(); return cp;}
    void setCurrentPosition(StepperPositions p) {noInterrupts(); _currentPosition = p; interrupts(); return;}
    StepperMotion getTargetPosition() {return motionBuffer.first();}
    void move(StepperMotion motion);
    void moveTo(StepperMotion positions);
  private:
    StepperPositions _currentPosition; // the current motor step positions (the most up to date, updated by the timer interrupt)
    char _stepGeneration = 'M'; // stepping generation: M:from motion buffer, S:from step buffer
    unsigned long _updateStepsToTarget = 0; // how many update steps need to be pushed on the buffer to plan the target reach ?
    step_t _updateStep; // the current update step structure (allows to keep the direction byte and period)
  private: // bresenham line drawing algorithm used for step generation
    StepperPositions _N_bresenham; // the total number of steps, by motor, corresponding to the current motion
    long _max_N_bresenham; // the maximum number of steps
    StepperPositions _D_bresenham; // the distance to the line (some error function)
    void bresenhamInit(StepperPositions x0, StepperPositions x1);
    StepperBools bresenhamStep();
};

// Define the unique stepper engine
StepEngine steppers;

// Function for the timer interrupt
void stepSteppers() {steppers.step();} ;

void StepEngine::setup() {
  Timer1.initialize(DEFAULT_TIMER_PERIOD);
  Timer1.attachInterrupt(stepSteppers);
}

void StepEngine::setStepGeneration(char mode) {
// change the step generation mode
  if (mode==_stepGeneration) return;
  switch (mode) {
    case 'M': { // to motionBuffer mode
      stepBuffer.loop = false;
      while(!stepBuffer.isEmpty()); // wait for the step buffer to be empty
    }; break;
    case 'S': {
      motionBuffer.loop = false;
      while(!motionBuffer.isEmpty()) update(); // empty the motion buffer
      while(!stepBuffer.isEmpty()); // wait for the step buffer to be empty
    }; break;
  }
  _stepGeneration = mode;
}

void StepEngine::move(StepperMotion motion) {
// move relatively to the last planned positions
  setStepGeneration('M');
  if (motionBuffer.isEmpty()) moveTo(motion);
  else moveTo(motion + motionBuffer.last());
};

void StepEngine::moveTo(StepperMotion positions) {
// move relatively to the last planned positions
  setStepGeneration('M');
  while (!motionBuffer.available()); // wait for the positions buffer to have free space
  motionBuffer.push(positions);
};

void StepEngine::update() {
  if (_stepGeneration=='S') return; // will not update the buffer if step generation is based on it
  while (stepBuffer.available()) { // while there is some room on the step buffer...
    if (_updateStepsToTarget==0) {
      StepperMotion previousMotion = motionBuffer.shift();
      if (motionBuffer.loop) motionBuffer.push(previousMotion); // IF LOOPING, RE-PUSH it at the end of the buffer
      if (motionBuffer.isEmpty()) return; // no next position to go !
      StepperMotion nextMotion = motionBuffer.first(); // extract the next motion from the buffer
      // Compute motion-constant data
      _updateStep.dir = StepperBool2Byte(nextMotion > previousMotion); // constant direction 
      bresenhamInit(previousMotion,nextMotion); // initialize the bresenham stepping scheme
      _updateStepsToTarget = _max_N_bresenham; // initialize the number of updates to perform
    };
    while (stepBuffer.available() && _updateStepsToTarget!=0) { // while there is some room on the step buffer...
      // Push a new step to the step buffer
      StepperBools needsStep = bresenhamStep(); // which motor needs to be stepped ?
      _updateStep.step = StepperBool2Byte(needsStep);
      noInterrupts(); stepBuffer.push(_updateStep); interrupts();
      // An update step has been added
      _updateStepsToTarget--;
    };
  };
};

// Bresenham step
void StepEngine::bresenhamInit(StepperPositions x0, StepperPositions x1) {
  _N_bresenham = (x1-x0)._abs();
  _max_N_bresenham = _N_bresenham._max();
  _D_bresenham = _N_bresenham*2 - _max_N_bresenham;
};

// Bresenham step
StepperBools StepEngine::bresenhamStep() {
  StepperBools needsStep = _D_bresenham > 0 ;
  for (uint8_t i=0;i<N_MOTORS;i++) if (needsStep[i]) _D_bresenham[i] -= _max_N_bresenham*2;
  _D_bresenham = _D_bresenham + _N_bresenham*2;
  return needsStep;
};

// ENGINE RUN FUNCTION
void StepEngine::step() {
  // keep track of the number of interrupts
  // RETRIEVE THE NEXT STEP
  if (stepBuffer.isEmpty()) return;
  step_t nextStep = stepBuffer.shift(); // this might take too much time..
  // SET PORT STATES; DIR before STEP
  PORT_DIR = nextStep.dir;
  if (DEDGE) // double edge mode, only toggle the step port
    PORT_STEP ^= nextStep.step; // toggles stepping pins
  else { // step only on RISING edges, so a full PWM needs to be generated
    PORT_STEP = nextStep.step; // set pins HIGH
    delayMicroseconds(1); // wait
    PORT_STEP = 0b00000000; // // set pins LOW
  }; 
  // MODIFY THE TIMER PERIOD
  Timer1.setPeriod(nextStep.getUSPeriod());
  // IF LOOPING, RE-PUSH the current step at the end of the buffer
  if (stepBuffer.loop) stepBuffer.push(nextStep);
  // Update the current positions
  for (int8_t b=0;b<8;b++) if (bitRead(nextStep.step,b)) _currentPosition[b] += (bitRead(nextStep.dir,b) ? 1 : -1);
  // Is the buffer empty ?
  if (stepBuffer.isEmpty()) Timer1.setPeriod(DEFAULT_TIMER_PERIOD);
}

// BUFFER FROM SERIAL
void readHEXStepBufferFromSerial() {
  char msg[4];
  while (Serial.available()) {
    int nvalid = Serial.readBytes(msg, 4);
    if (nvalid<4) return ;
    step_t newStep ;
    newStep.step = hex2byte(msg[0],msg[1]);//readByteFrom2hex();
    newStep.dir = hex2byte(msg[2],msg[3]);//readByteFrom2hex();
    while (!steppers.stepBuffer.available()); // wait for the buffer to have free space
    steppers.stepBuffer.push(newStep);
  }
};


// SAMPLE STEP BUFFERS
// Ramp buffer (accelerates to target speed then decelerate to stanstill)
void generateRampStepBuffer(float maxspeed=MAX_SPEED, float acceleration=MAX_ACCELERATION, uint16_t length=STEP_BUFFER_SIZE) {
  step_t newStep;
  newStep.step = 0b11111111; // all motors turn
  newStep.dir = 0b11111111; // in the same direction
  float speed = sqrt(acceleration); // in steps/seconds
  uint16_t rampSteps = 0; // will keep the number of steps needed to accelerate/deccelerate
  for (unsigned long i=0;i<length;i++) {
    speed = min(speed,maxspeed);
    newStep.setUSPeriod((unsigned long)(1000000.0/speed));
    steppers.stepBuffer.push(newStep);
    if (speed<maxspeed or i>length-rampSteps-1) {
      speed += acceleration/speed;
      rampSteps++;
    } 
    if (i==length/2) acceleration *= -1.0; // start decelerating at half the buffer
  };
};


// SAMPLE POSITION BUFFERS
void generateSinMotion() {
// Generate sinusoidal motion
  // steppers.motionBuffer.loop = false;
  // while (!steppers.positonsBuffer.isEmpty()) steppers.update(); // empty the current position buffer
  steppers.motionBuffer.clear(); // empty the current motion buffer
  steppers.motionBuffer.loop = true;
  const float dt = 2.0*3.141651/float(POS_BUFFER_SIZE);
  const float nSteps = 100.0;
  StepperMotion nextPos;
  for (uint8_t t=0;t<POS_BUFFER_SIZE;t++) {
    for (uint8_t i=0;i<N_MOTORS;i++) {
      float phi = float(t); //(i+1)*t;
      nextPos[i] = nSteps*sin(dt*phi);
    }
    steppers.moveTo(nextPos);
  }
  steppers.motionBuffer.print();
};

