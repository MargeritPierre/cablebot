#include <Arduino.h>
#include <SPI.h>

#include <TimerOne.h>

#define BAUDRATE 250000
#define SERIAL_TIMEOUT 10 //milliseconds
#define MICROSTEP 0
#define MAX_ACCELERATION 5000
#define MAX_SPEED 1000
#define N_MOTORS 4  // number of activated motors
#define RMS_CURRENT 200
#define DEDGE true // use DEDGE mode ? (STEP on rising AND falling edges)

#include "Array.h" // defines some usefull functions

#include "SerialUtils.h" // defines some usefull functions

#include "DriverConfig.h" // defines the TMC5160 driver controller

#include "StepperData.h" // defines objects that describe the state of the stepper motors

#include "StepEngine.h" // defines the engine that generates the signals for the steppers

// =================================================================== SETUP
void setup() {
  digitalWrite(LED_BUILTIN,LOW) ;

  SPI.begin();
  Serial.begin(BAUDRATE);
  Serial.setTimeout(SERIAL_TIMEOUT);
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

  // TESTS!
  // testOperationsOnStepperPositions();
  // Array<int,4> testArray = {1,2,3,4};
  // Serial.print("test array: "); testArray.print(); Serial.println();
  // while(1);
}

// =================================================================== LOOP
unsigned long lastMillis = 0;
unsigned long printPeriodMillis = 0;
void loop() {

  if (Serial.available()) { // a new message is available on the serial
    Serial.println("New Message!");
    char header = Serial.read();
    char sep = Serial.read(); // remove the message separator(':')
    switch (header) {
      case 'F': { // SOME FLOATNG POINT NUMBERS...
        StepperFloats floats = StepperFloats(Serial);
        Serial.println("floats: "+floats.to_String());
      }; break;
      case 'M': { // RELATIVE MOTION
        StepperMotion motion = StepperMotion(Serial); // import the motion vector from the serial
        steppers.move(motion);
        steppers.motionBuffer.print();
      }; break;
      case 'G': { // RELATIVE MOTION
        StepperMotion positions = StepperMotion(Serial); // import the motion vector from the serial
        steppers.moveTo(positions);
        steppers.motionBuffer.print();
      }; break;
      case 'E': { // ENABLE DRIVERS
        drivers.enable();
      }; break;
      case 'D': { // DISABLE DRIVERS
        drivers.disable();
      }; break;
      case 'I': { // CHANGE DRIVER RATED COIL CURRENT
        drivers.rms_current = Serial.parseInt();
        drivers.applyConfig();
        Serial.println("DRIVER::RMS_CURRENT set to "+ String(drivers.rms_current));
      }; break;
      case 'P': { // PRINT BUFFER
        steppers.stepBuffer.print();
      }; break;
      case 'X': { // PRINT stepper positions
        printPeriodMillis = Serial.parseInt();
        }; break;
      case 'B': { // BUFFER-RELATED MESSAGES
        header = sep; 
        sep = Serial.read();
        if (1) steppers.stepBuffer.clear();
        else {
          Serial.println("Emptying the buffer...");
          steppers.stepBuffer.loop = false;
          while (!steppers.stepBuffer.isEmpty()); // wait for the buffer to be empty
        };
        Timer1.stop(); delay(100);
        switch (header) {
          case 'S': { // Replace the full buffer with an HEX message
            readHEXStepBufferFromSerial();
          }; break;
          case 'R': { // Ramp buffer
            float maxspeed = MAX_SPEED;
            float acceleration = MAX_ACCELERATION;
            uint16_t length = STEP_BUFFER_SIZE;
            if (Serial.available()) maxspeed = Serial.parseFloat();
            Serial.read();
            if (Serial.available()) acceleration = Serial.parseFloat();
            Serial.read();
            if (Serial.available()) length = Serial.parseInt();
            generateRampStepBuffer(maxspeed,acceleration,length);
          }; break;
        };
        steppers.stepBuffer.loop = true;
        Timer1.start();
        }; break;
      case 'T': { // TESTS!
        String testCase = Serial.readString();
        testCase.trim(); // remove end of line, whitespaces etc..
        if (testCase=="test") Serial.println("TEST!");
        else if(testCase=="sin") generateSinMotion();
        else Serial.println("Unknown Test: "+testCase);
        }; break;
      default: {
        Serial.print("Unknown message: "); Serial.print(header); Serial.print(sep); while (Serial.available()) Serial.write(Serial.read());
        }; break;
    }
    // PRINT BUFFER
    while(Serial.available()) Serial.read(); // empty the serial input
  }

  // Update steppers
  steppers.update();

  // Print some info
  unsigned long dmillis = millis()-lastMillis;
  if (printPeriodMillis and dmillis>=printPeriodMillis) {
    // Serial.print("ticks:"+String(steppers.timerTicks));
    // Serial.print("pos: ");
    steppers.getCurrentPosition().print();
    Serial.println();
    lastMillis = millis();
  };
}
