// Some fun with the motor noise !

class note_t {
  public:
    String name = "A0"; // the note indicator; if need, put "#" or "b" just after the letter
    uint16_t duration = 250; // the note duration in millisecs
    float pitch();
    note_t(String n){name=n;};
    note_t(String n,uint16_t d){name=n;duration=d;};
    note_t(){};
    void play();
};

float note_t::pitch() {
  if (name.length()==0) return 0.0;
  int n;
  switch (name.charAt(0)) {
    case 'C': n=0; break;
    case 'D': n=2; break;
    case 'E': n=4; break;
    case 'F': n=5; break;
    case 'G': n=7; break;
    case 'A': n=9; break;
    case 'B': n=11; break;
  };
  n-=9; // reference is 'A0'
  int o = name.charAt(name.length()-1)-0x30; // octave
  n += o*12;
  if (name.length()>2) 
    switch (name.charAt(1)) {
      case 'b': n-=1; break;
      case '#': n+=1; break;
    };
  return 55.0*pow(2.0,float(n)/12.0);
}

void note_t::play() {
  step_t mstep;
  mstep.step = 0b00000011; // THE MOTORS THAT WILL VIBRATE
  steppers.stepBuffer.clear(); // clear the current buffer
  steppers.stepBuffer.loop = true; // THE STEP BUFFER WILL BE RINGING
  if (name.length()>0) {
    // generate a square wave with a step forward and a step backward
    mstep.setSpeed(2.0*pitch());
    noInterrupts();
    mstep.dir = 0b00000000;
    steppers.stepBuffer.push(mstep);
    mstep.dir = 0b00001111;
    steppers.stepBuffer.push(mstep);
    interrupts();
  }
  delay(duration);
  steppers.stepBuffer.clear(); // clear the current buffer
}

void playCScale() {
  note_t("C3").play();
  note_t("D3").play();
  note_t("E3").play();
  note_t("F3").play();
  note_t("G3").play();
  note_t("A3").play();
  note_t("B3").play();
  note_t("C4").play();
  return;
};

void playChromatic() {
  String names[] = {"C","Db","D","Eb","E","F","Gb","G","Ab","A","Bb","B"};
  note_t thenote;
  thenote.duration = 100;
  for (char o='3';o<='6';o++) for (int n=0;n<12;n++) {
    String str = names[n]+o;
    thenote.name = str;
    thenote.play();
    // Serial.print(str+": "); Serial.println(" "+String(thenote.pitch())+"Hz");
  }
}

