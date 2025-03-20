#include <Arduino.h>
#include <SPI.h>

#include <TimerOne.h>

#define BAUDRATE 250000
#define BUFFER_TIMEOUT 10 //milliseconds
#define MAX_ACCELERATION 20000
#define MAX_SPEED 4000
#define N_MOTORS 4  // number of activated motors
#define RMS_CURRENT 500
#define DEDGE true // use DEDGE mode ? (STEP on rising AND falling edges)

#include "SerialUtils.h" // defines some usefull functions

#include "DriverConfig.h" // defines the TMC5160 driver controller

#include "StepperData.h" // defines objects that describe the state of the stepper motors

#include "StepEngine.h" // defines the engine that generates the signals for the steppers

// =================================================================== SETUP
void setup() {
  digitalWrite(LED_BUILTIN,LOW) ;

  SPI.begin();
  Serial.begin(BAUDRATE);
  Serial.setTimeout(BUFFER_TIMEOUT);
  while (!Serial);  // Wait for serial port to connect
  Serial.println();

  // INITIALIZE TMC5160 DRIVERS
  drivers.setup();
  // drivers.testConnection(); // optional
  drivers.applyConfig();
  drivers.enable();

  // Wait for Serial to open
  digitalWrite(LED_BUILTIN,HIGH);

  // Initialize stepper engine
  steppers.setup();

  Serial.println("Setup Finished.");
  // testOperationsOnStepperPositions();
}

// =================================================================== LOOP
unsigned long lastMillis = 0;
unsigned long printPeriodMillis = 0;
void loop() {

  if (Serial.available()) { // a new message is available on the serial
    Serial.println("New Message!");
    char header = Serial.read();
    switch (header) {
      case 'M': // MOVE TO
        while (!steppers.positionsBuffer.available()); // wait for the positions buffer to have free space
        long motion[N_MOTORS];
        // for (uint8_t i=0;i<N_MOTORS;i++) {
        //   motion[i] = (long)Serial.parseInt();
        //   Serial.read(); // remove the separator character
        // };
        StepperPositions nextPosition = StepperPositions(motion); 
        // if (steppers.positionsBuffer.isEmpty()) newPosition = newPosition + steppers.getCurrentPosition();
        // else newPosition = newPosition + steppers.positionsBuffer.last();
        steppers.positionsBuffer.push(nextPosition);
        steppers.positionsBuffer.print();
        break;
      case 'E': // ENABLE DRIVERS
        drivers.enable();
        break;
      case 'D': // DISABLE DRIVERS
        drivers.disable();
        break;
      case 'I': // DISABLE DRIVERS
        drivers.rms_current = Serial.parseInt();
        drivers.applyConfig();
        Serial.println("DRIVER::RMS_CURRENT set to "+ String(drivers.rms_current));
        break;
      case 'P': // PRINT BUFFER
        steppers.stepBuffer.print();
        break;
      case 'X': // PRINT stepper positions
        printPeriodMillis = Serial.parseInt();
        break;
      default: // BUFFER-RELATED MESSAGES
        if (1) steppers.stepBuffer.clear();
        else {
          Serial.println("Emptying the buffer...");
          steppers.stepBuffer.loop = false;
          while (!steppers.stepBuffer.isEmpty()); // wait for the buffer to be empty
        };
        Timer1.stop(); delay(100);
        switch (header) {
          case 'B': // Replace the full buffer with an HEX message
            readHEXBufferFromSerial();
            break;
          case 'R': // Ramp buffer
            float maxspeed = MAX_SPEED;
            float acceleration = MAX_ACCELERATION;
            uint16_t length = STEP_BUFFER_SIZE;
            if (Serial.available()) maxspeed = Serial.parseFloat();
            Serial.read();
            if (Serial.available()) acceleration = Serial.parseFloat();
            Serial.read();
            if (Serial.available()) length = Serial.parseInt();
            generateRampBuffer(maxspeed,acceleration,length);
            break;
        };
        steppers.stepBuffer.loop = true;
        Timer1.start();
        break;
    }
    // PRINT BUFFER
    while(Serial.available()) Serial.read(); // empty the serial input
  }

  // Print some info
  unsigned long dmillis = millis()-lastMillis;
  if (printPeriodMillis and dmillis>=printPeriodMillis) {
    // Serial.print("ticks:"+String(steppers.timerTicks));
    Serial.print("pos: ");
    steppers.getCurrentPosition().print();
    Serial.println();
    lastMillis = millis();
  }
}
