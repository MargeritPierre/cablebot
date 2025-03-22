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

// Buffer containing a collection of stepper positions
typedef DataBuffer<StepperPositions, POS_BUFFER_SIZE> PositionsBuffer;

// STEPPER ENGINE CLASS
class StepEngine {
  public:
    StepBuffer stepBuffer;
    PositionsBuffer positionsBuffer;
    char stepgen = 'P'; // stepping generation: P:from position buffer, S:from step buffer
    void setup();
    void step(); // apply the next buffered step; executed by the timer interrupt
    void update(); // update and fill the step buffer using the position buffer
    StepperPositions getCurrentPosition() {noInterrupts(); StepperPositions cp = _currentPosition; interrupts(); return cp;}
    void setCurrentPosition(StepperPositions p) {noInterrupts(); _currentPosition = p; interrupts(); return;}
    StepperPositions getTargetPosition() {return positionsBuffer.first();}
    void move(StepperPositions motion);
    void moveTo(StepperPositions positions);
  private:
    StepperPositions _currentPosition; // the current motor step positions
    StepperPositions _targetPosition; // the current motor step target positions
    bool _targetPositionReached = true; // has the target position been reached ?
    StepperPositions _updateMotion; // the currently updated motion vector
    step_t _updateStep; // the current update step template
    StepperPositions _updatePosition; // the last updated positions (that should be reached at the current last buffered step)
    // StepperFloats _updateSpeeds; // the last updated velocity vector
};

// Define the unique stepper engine
StepEngine steppers;

// Function for the timer interrupt
void stepSteppers() {steppers.step();} ;

void StepEngine::setup() {
  Timer1.initialize(DEFAULT_TIMER_PERIOD);
  Timer1.attachInterrupt(stepSteppers);
}

void StepEngine::move(StepperPositions motion) {
// move relatively to the last planned positions
  if (positionsBuffer.isEmpty()) moveTo(_targetPosition + motion);
  else moveTo(positionsBuffer.last() + motion);
};

void StepEngine::moveTo(StepperPositions positions) {
// move relatively to the last planned positions
  while (!positionsBuffer.available()); // wait for the positions buffer to have free space
  positionsBuffer.push(positions);
};

void StepEngine::update() {
  if (stepgen=='S') return; // will not update the buffer if step generation is based on it
  delay(200);
  while (stepBuffer.available()) { // while there is some room on the step buffer...
    Serial.println("Update StepEngine..");
    Serial.println("  current update position: "+_updatePosition.to_String());
    if (_targetPositionReached) {
      if (positionsBuffer.isEmpty()) return; // no next position to go !
      _targetPosition = positionsBuffer.shift(); // extract the next targrt position from he buffer
      if (positionsBuffer.loop) positionsBuffer.push(_targetPosition); // IF LOOPING, RE-PUSH it at the end of the buffer
      Serial.println("  new target position: "+_targetPosition.to_String());
      // Compute motion-constant data
      _updateMotion = _targetPosition - _updatePosition ;
      _updateStep.dir = StepperBool2Byte(_updateMotion > StepperPositions(0)); // constant direction 
      _targetPositionReached = false;
    };
    while (stepBuffer.available() && !_targetPositionReached) { // while there is some room on the step buffer...
      // Push a new step to the step buffer
      StepperBools needsStep = (_updatePosition != _targetPosition);
      _updateStep.step = StepperBool2Byte(needsStep);
      noInterrupts(); stepBuffer.push(_updateStep); interrupts();
      // Update the position
      for (uint8_t i=0;i<N_MOTORS;i++) if (needsStep[i]) _updatePosition[i] += (_updateMotion[i] > 0 ? 1 : -1);
      // Target position reached ?
      _targetPositionReached = !(needsStep.any());
    };
  };
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


// SAMPLE BUFFERS
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
