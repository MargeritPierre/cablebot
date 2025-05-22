#include <SimpleWebSerial.h>

SimpleWebSerial WebSerial;

const int ledPin = LED_BUILTIN;
void setup() {
  // initialize serial communications
  Serial.begin(57600);
  pinMode(ledPin, OUTPUT);

  WebSerial.on("led", toggleLed);
}

void toggleLed(JSONVar state) {
  digitalWrite(ledPin, !digitalRead(ledPin));
}

void loop() {
  WebSerial.check();
  delay(5);
}