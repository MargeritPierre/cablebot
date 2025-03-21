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
    bool loop = true; // is the buffer looping ?
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
  public:
    StepBuffer stepBuffer;
    PositionsBuffer positionsBuffer;
    void setup();
    void step();
    void update();
    StepperPositions getCurrentPosition() {noInterrupts(); StepperPositions cp = _currentPosition; interrupts(); return cp;}
    void setCurrentPosition(StepperPositions p) {noInterrupts(); _currentPosition = p; interrupts(); return;}
    StepperPositions targetPosition() {return positionsBuffer.first();}
    void move(StepperPositions motion);
    void moveTo(StepperPositions position);
  private:
    StepperPositions _currentPosition;
};

// Define the unique stepper engine
StepEngine steppers;

// Function for the timer interrupt
void runSteppers() {steppers.step();} ;

void StepEngine::setup() {
  Timer1.initialize(DEFAULT_TIMER_PERIOD);
  Timer1.attachInterrupt(runSteppers);
}

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
  for (int8_t b=0;b<8;b++) _currentPosition[b] += (bitRead(nextStep.dir,b) ? bitRead(nextStep.step,b) : -bitRead(nextStep.step,b));
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
