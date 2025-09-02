#include <AccelStepper.h>

// The stepper pins
#define STEPPER_ENA_PIN 12
#define STEPPER_DIR_PIN 4
#define STEPPER_STEP_PIN 5
#define STEP_BY_DEG (200.0/2.0*4.0) // step_by_turn/deg_by_turn*microstep

#define ZERO_PIN 13


// Define the stepper motor
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

// Serial Communication
#define MAX_MSG_LEN 32
#define SERIAL_BAUD_RATE 115200
char current_msg[MAX_MSG_LEN] ;
bool new_msg = false ;
int msg_idx = 0 ;

void setup()
{  
    stepper.setEnablePin(STEPPER_ENA_PIN);
    stepper.setPinsInverted(true,false,true); // (dir,step,ena) invert the enable pin
    stepper.setMaxSpeed(4000.0);
    stepper.setAcceleration(10000.0);
    stepper.enableOutputs();

    pinMode(ZERO_PIN,INPUT);

    Serial.begin(SERIAL_BAUD_RATE);
}

void loop()
{
    // Process the Serial
    readSerial() ;
    if (new_msg) {
      processMsg();
      resetMsg();
    }
    // Run the motor
    stepper.run();
    
    //Serial.println(digitalRead(ZERO_PIN));
}

void findZero() {
 // FIRST GO FORWARD
  if (digitalRead(ZERO_PIN)) {
    stepper.move(360.0*STEP_BY_DEG);
    while (digitalRead(ZERO_PIN)) stepper.run();
    long pos = stepper.currentPosition();
    stepper.stop();
    while (stepper.speed()!=0) stepper.run();
    stepper.setCurrentPosition(stepper.currentPosition()-pos);
    stepper.runToNewPosition(-1.0*STEP_BY_DEG);
    stepper.runToNewPosition(0);
  } else {
    stepper.runToNewPosition(stepper.currentPosition()-long(15.0*STEP_BY_DEG));
    findZero();
  }
}

void processMsg() {
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(current_msg," ");      // get the first part - the string
    char command[MAX_MSG_LEN];
    strcpy(command, strtokIndx); // copy it to messageFromPC

    strtokIndx = strtok(NULL, " ");
    float val = atof(strtokIndx);     // convert this part to a float

    if (strcmp(command,"move")==0) stepper.move(val*STEP_BY_DEG);
    else if (strcmp(command,"moveto")==0) stepper.moveTo(val*STEP_BY_DEG);
    else if (strcmp(command,"runtonewposition")==0) stepper.runToNewPosition(val*STEP_BY_DEG);
    else if (strcmp(command,"setacceleration")==0) stepper.setAcceleration(val*STEP_BY_DEG);
    else if (strcmp(command,"setmaxspeed")==0) stepper.setMaxSpeed(val*STEP_BY_DEG);
    else if (strcmp(command,"currentposition")==0) Serial.println(float(stepper.currentPosition())/STEP_BY_DEG);
    else if (strcmp(command,"setcurrentposition")==0) stepper.setCurrentPosition(long(val*STEP_BY_DEG));
    else if (strcmp(command,"findzero")==0) findZero();
    else if (strcmp(command,"enable")==0) stepper.enableOutputs();
    else if (strcmp(command,"disable")==0) stepper.disableOutputs();
    else {
      Serial.println("Unknown Command !");
      return;
    }
    Serial.print(command); Serial.print(":"); Serial.println(val);
    
}

void resetMsg() {
  for (int i=0;i<MAX_MSG_LEN;i++) current_msg[i] = '\0' ;
  msg_idx = 0 ;
  new_msg = false ;
}

void readSerial() {
  while(Serial.available()>0) {
    char c = Serial.read() ;
    if (c=='\n') {
      c = '\0' ;
      new_msg = true ;
    } else {
      current_msg[msg_idx++] = c ;
      if (msg_idx==MAX_MSG_LEN) break ; // split long messages
    }
  }
}
