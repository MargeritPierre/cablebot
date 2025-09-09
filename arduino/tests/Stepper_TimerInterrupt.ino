// Use timer1 to step a stepper motor via a custom PWM waveform

#include <TimerOne.h>

const int stepPin = 9;
const int dirPin = 3 ;
const int modePins[3] = {5, 6, 7};
const int duty = 2;

float speed = 0.0 ;
float maxSpeed = 4000.0*4.0;
float acceleration = maxSpeed/5.0;
volatile long steps = 0;
volatile int dir = 1;
volatile unsigned long period = 0;

long lastUpdateMicros = 0;

void setup(void)
{
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  // Set microstep
  pinMode(modePins[0],OUTPUT) ;
  pinMode(modePins[1],OUTPUT) ;
  pinMode(modePins[2],OUTPUT) ;
  digitalWrite(modePins[0],LOW) ;
  digitalWrite(modePins[1],HIGH) ;
  digitalWrite(modePins[2],LOW) ;
  // Initialize timer
  Timer1.initialize();
  Timer1.attachInterrupt(step);
  Timer1.pwm(stepPin,duty,1024UL);
  // Open Serial
  Serial.begin(115200);
  // Initialize elapsed micros
  lastUpdateMicros = micros() ;
}

void loop(void)
{
  if (abs(speed)>=maxSpeed) acceleration *= -1 ;
  dir = (speed>0.0 ? 1 : -1) ;
  digitalWrite(dirPin,(dir>0 ? HIGH : LOW)) ;
  
  long t = micros();
  speed += (float(t-lastUpdateMicros)/1000000.0)*acceleration;
  lastUpdateMicros = t;

  unsigned long tempPeriod = (unsigned long)(1000000.0/abs(speed)) ;
  noInterrupts();
  period = tempPeriod;
  interrupts();

  Serial.println("Speed: "+ String(speed));// + ", Period: " + String(period));

  delay(5);
}

void step() {
  steps+=dir;
  if (period) {
    Timer1.setPeriod(period);
    period=0;
  }
}
