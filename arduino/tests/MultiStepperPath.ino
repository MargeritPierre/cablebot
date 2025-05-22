#include <AccelStepper.h>
#include <MultiStepper.h>

AccelStepper stepper_array[] = {
  {AccelStepper::DRIVER,1,2},
  {AccelStepper::DRIVER,3,4},
  {AccelStepper::DRIVER,5,6},
  {AccelStepper::DRIVER,7,8},
};
const int nSteppers = sizeof(stepper_array) / sizeof(stepper_array[0]);

long path[][nSteppers] = {
  {0,0,0,0},
  {100,0,0,0},
  {100,100,0,0},
  {100,100,-100,-100},
  {-100,-100,100,100},
};
const int nPath = sizeof(path) / sizeof(path[0]);
int nextPtInPath = 0;

int printPeriod = 100;
unsigned long lastPrintMillis = 0;

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  // Configure each stepper
  for (int s=0;s<nSteppers;s++) stepper_array[s].setMaxSpeed(100);

  // Then give them to MultiStepper to manage
  for (int s=0;s<nSteppers;s++) steppers.addStepper(stepper_array[s]);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool stillMoving = steppers.run();

  if (!stillMoving) {
    steppers.moveTo(path[nextPtInPath]);
    nextPtInPath = (nextPtInPath+1) % nPath;
  }

  if (millis()-lastPrintMillis>printPeriod) {
    Serial.print("positions:\t");
    for (int s=0;s<nSteppers;s++) {
      Serial.print(stepper_array[s].currentPosition());
      Serial.print('\t');
    }
    Serial.println();
    lastPrintMillis = millis();
  }

}
