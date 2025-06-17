
#include <SimpleWebSerial.h>

class WebSerial {
  public:
    SimpleWebSerial serial;
    void setup();
    void update();
    void log(String msg);
};

WebSerial web = WebSerial(); // define the object

void toggleLed(JSONVar state) {
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  // StepperPositions pos = {1000,1000,1000,1000};
  // StepperMotion motion = StepperMotion(pos,1000,5000);
  // steppers.move(motion);
}

void move(JSONVar values) {

  // if you're using a common-cathode LED, just use "constrain(color, 0, 255);"
  long r = (long)values["r"];
  long g = (long)values["g"];
  long b = (long)values["b"];
  long a = (long)values["a"];

  StepperPositions pos = {r,g,b,a};
  StepperMotion motion = StepperMotion(pos,4000,20000);
  steppers.move(motion);
  // web.log("move");
  // while (!steppers.motionBuffer.isEmpty()) steppers.update();
  // web.log("emptyMotionBuffer");
  // while (!steppers.stepBuffer.isEmpty()) delay(10);
  // web.log("emptyStepBuffer");

  // String msg = "";
  // msg+=motion.to_String();
  // msg+=" ("+String(steppers.motionBuffer.size())+")";
  // web.log(msg);
}


void WebSerial::setup() {
  serial.on("led", toggleLed);
  serial.on("move", move);
}

void WebSerial::update() {
  serial.check();
}

void WebSerial::log(String msg) {
    char chr[msg.length()+1]; msg.toCharArray(chr,msg.length()+1);
    serial.log(chr);
}