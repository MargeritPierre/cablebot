
// Convert ascii character to binary
unsigned char ascii2hex(unsigned char c) {
  if (c >= 0x30 and c <= 0x39) return c - 0x30;
  if (c >= 0x41 and c <= 0x46) return c - 0x37;
  return -1;
};

// Convert 2 asciis characters to a byte
unsigned char hex2byte(unsigned char MSB, unsigned char LSB) {
  return ((ascii2hex(MSB) & 0b1111) << 4) + (ascii2hex(LSB) & 0b1111);
};

unsigned char readByteFrom2hex() {
  char MSB = Serial.read();
  char LSB = Serial.read();
  return hex2byte(MSB,LSB);
};

// Print an octet in binary format
void printOCTET(uint8_t octet) {
    for (int b=7;b>=0;b--) Serial.print(bitRead(octet,b),BIN);
}

// Test two hex -> byte conversion
void testHEX2BYTE() {
  char hc[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
  for (int c1=0;c1<16;c1++) for (int c2=0;c2<16;c2++) {
    Serial.print(hc[c1]); Serial.print(hc[c2]);
    Serial.print('|');Serial.print(ascii2hex(hc[c1]),DEC);Serial.print(',');Serial.print(ascii2hex(hc[c2]),DEC);
    Serial.print('|');Serial.print(hex2byte(hc[c1],hc[c2]));
    Serial.println();
  }
};