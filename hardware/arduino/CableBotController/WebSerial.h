
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
  long r = constrain((long)values["r"], 0, 255);
  long g = constrain((long)values["g"], 0, 255);
  long b = constrain((long)values["b"], 0, 255);

  StepperPositions pos = {r,g,b,0};
  StepperMotion motion = StepperMotion(pos,1000,5000);
  steppers.move(motion);

  // print the three numbers in one string as hexadecimal:
  char format[8];
  sprintf(format, "#%02X%02X%02X", (int)values["r"], (int)values["g"], (int)values["b"]);
  web.serial.send("hexadecimal", format);
  web.log(String(steppers.motionBuffer.size()));
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