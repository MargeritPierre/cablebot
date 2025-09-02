#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <AccelStepper.h>

#define BAUDRATE 250000
#define N_MOTORS 4  // number of activated motors
#define MAX_ACCELERATION 1600
#define MAX_SPEED 1600

#include "DriverConfig.h"
Drivers drivers = Drivers();

#include "StepperData.h"

#include "EasySteppers.h"
Steppers steppers = Steppers();


// ==================4=====================================================================
StepperPositions initialMotion = {0,0,0,0}; // helps to apply some pre-tension
StepperPositions initialPositions = {0,0,0,0};
long dL = 1600;  // some path motion length
StepperPositions path[] = {
  {0,dL,0,dL}, // +X
  {0,0,0,0}, // back to origin
  // {-dL,-dL,dL,dL}, // +X
  // {0,0,0,0}, // back to origin
  // {dL,-dL,dL,-dL}, // +Z
  // {0,0,0,0}, // back to origin
  // { -dL, -dL, -dL, -dL },  // pre-tension
  // { 0, 0, 0, 0 },      // back to origin
};
const int nPath = sizeof(path) / sizeof(path[0]);
int nextPtInPath = 0;
bool followPath = false;

// =======================================================================================
int printPeriod = 100; // millisecs
unsigned long lastPrintMillis = 0;// Apply pre-tension


// =======================================================================================
void setup() {

  SPI.begin();

  Serial.begin(BAUDRATE);
  while (!Serial);  // Wait for serial port to connect
  Serial.println();

  // INITIALIZE TMC5160 DRIVERS
  drivers.setup();
  drivers.testConnection(); // optional
  drivers.applyConfig();
  drivers.enable();

  // Initialize the steppers
  steppers.setup();

  // Apply the initial state
  steppers.move(initialMotion);
  while (!steppers.positionsReached()) steppers.run();
  steppers.setCurrentPositions(initialPositions);

//   // Wait for serial trigger
//   while(!Serial.available());
//   Serial.flush();
}



// =======================================================================================
void loop() {

  if (Serial.available()) followPath = !followPath;
  while (Serial.available()) Serial.read();

  if (followPath) {
    // Go to next point on path
    if (steppers.positionsReached()) {
      steppers.moveTo(path[nextPtInPath]);
      nextPtInPath = (nextPtInPath + 1) % nPath;
    }
  } else {
    steppers.moveTo(initialPositions);
  };

  // Run the motors
  steppers.run();

  // Print infos
  if (millis() - lastPrintMillis > printPeriod) {
    // Motor positions
    // Serial.println("Steppers: "+steppers.getCurrentPositions().to_String());
    // Motor Speeds
    // for ( uint8_t i = 0; i<N_MOTORS; i++) {
    //   Serial.print("speed"+String(i)+":");
    //   Serial.print(abs(steppers[i].speed()));
    //   Serial.print(' ');
    // }
    // Serial.println();
    // Stall Guard values
    // printSGValues();
    // update timestamp
    lastPrintMillis = millis();
  }
}
