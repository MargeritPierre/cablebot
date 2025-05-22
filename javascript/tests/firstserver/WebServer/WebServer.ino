void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  Serial.println("Hello");
  delay(300);
  Serial.println("World");
  delay(300);
  Serial.println("");
  delay(300);        // delay 300 ms
}