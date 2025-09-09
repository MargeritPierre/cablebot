// DEFINE A TEMPLATE FOR STEPPER DATA (position, motion, speed, acceleration...)

// Template class
template <class T>
class StepperData {
  public:        
    // Constructors
    StepperData() {for (uint8_t i=0;i<N_MOTORS;i++) _array[i] = (T)0;}; // default
    StepperData(T uniform) {for (uint8_t i=0;i<N_MOTORS;i++) _array[i] = uniform;}; // default
    StepperData(T &state) {for (uint8_t i=0;i<N_MOTORS;i++) _array[i] = state[i];}; // construct with array
    StepperData(T s0,T s1,T s2,T s3) {_array[0] = s0;_array[1] = s1;_array[2] = s2;_array[3] = s3;}; // construct with 4 motors
    StepperData(T s0,T s1,T s2,T s3,T s4,T s5,T s6,T s7) {_array[0]=s0;_array[1]=s1;_array[2]=s2;_array[3]=s3;_array[4]=s4;_array[5]=s5;_array[6]=s6;_array[7]=s7;}; // construct with 8 motors
    StepperData(Stream &S); // construct from the serial
    //Overload the indexing operator
    const T& operator [](uint8_t index) const {return _array[index];};
    T& operator [](uint8_t index) {return _array[index];};
    // array-scalar operatons
    StepperData operator+(T other) {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = _array[i]+other;return res;};
    StepperData operator-(T other) {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = _array[i]-other;return res;};
    StepperData operator*(T other) {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = _array[i]*other;return res;};
    StepperData operator/(T other) {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = _array[i]/other;return res;};
    StepperData<bool> operator>(T other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]>other;return res;};
    StepperData<bool> operator>=(T other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]>=other;return res;};
    StepperData<bool> operator<(T other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]<other;return res;};
    StepperData<bool> operator<=(T other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]<=other;return res;};
    StepperData<bool> operator==(T other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]==other;return res;};
    StepperData<bool> operator!=(T other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]!=other;return res;};
    // Element-wise operations
    StepperData operator+(StepperData other) {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = _array[i]+other._array[i];return res;};
    StepperData operator-(StepperData other) {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = _array[i]-other._array[i];return res;};
    StepperData operator*(StepperData other) {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = _array[i]*other._array[i];return res;};
    StepperData operator/(StepperData other) {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = _array[i]/other._array[i];return res;};
    StepperData<bool> operator>(StepperData other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]>other[i];return res;};
    StepperData<bool> operator>=(StepperData other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]>=other[i];return res;};
    StepperData<bool> operator<(StepperData other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]<other[i];return res;};
    StepperData<bool> operator<=(StepperData other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]<=other[i];return res;};
    StepperData<bool> operator==(StepperData other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]==other[i];return res;};
    StepperData<bool> operator!=(StepperData other) {StepperData<bool> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = _array[i]!=other[i];return res;};
    StepperData operator++() {for (uint8_t i=0;i<N_MOTORS;i++) _array[i]++;return *this;};
    StepperData operator--() {for (uint8_t i=0;i<N_MOTORS;i++) _array[i]--;return *this;};
    StepperData sign() {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = T(_array[i]>T(0))-T(_array[i]<T(0)); return res;};
    // Accumulation functions functions
    bool all() {for (uint8_t i=0;i<N_MOTORS;i++) if (!_array[i]) return false; return true;};
    bool any() {for (uint8_t i=0;i<N_MOTORS;i++) if (_array[i]) return true; return false;};
    T sum() {T res=_array[0]; for (uint8_t i=1;i<N_MOTORS;i++) res+=_array[i]; return res;};
    float norm() {float res=pow(_array[0],2); for (uint8_t i=1;i<N_MOTORS;i++) res+=pow(_array[i],2); return sqrt(res);};
    // functons defined as macros in Arduino, so we add a '_' to prevent compiler errors
    StepperData _abs() {StepperData res; for (uint8_t i=0;i<N_MOTORS;i++) res._array[i] = abs(_array[i]); return res;};
    T _min() {T res=_array[0]; for (uint8_t i=1;i<N_MOTORS;i++) res=min(res,_array[i]); return res;};
    T _max() {T res=_array[0]; for (uint8_t i=1;i<N_MOTORS;i++) res=max(res,_array[i]); return res;};
    // Data type conversion
    StepperData<float>  _float() {StepperData<float> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = float(_array[i]);return res;};
    StepperData<int> _int() {StepperData<int> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = int(_array[i]);return res;};
    StepperData<long> _long() {StepperData<long> res; for (uint8_t i=0;i<N_MOTORS;i++) res[i] = long(_array[i]);return res;};
    // Printing function
    String to_String(char lbl='x');
    void print() {Serial.print(to_String());};
  private:
    T _array[N_MOTORS];
};

template <class T>
String StepperData<T>::to_String(char lbl='x') {
// old version
  // String str = "["; 
  // for (uint8_t i=0;i<N_MOTORS;i++) str += String(_array[i]) + ",";
  // return str + "]"; 
// new version
  String str; 
  for (uint8_t i=0;i<N_MOTORS;i++) {
    if (i>0) str += '\t';
    str += lbl;
    str += String(i);
    str += ':';
    str += String(_array[i]);
  }
  return str; 
};

// Retrieve StepperData from serial
template <class T>
StepperData<T>::StepperData(Stream &S) {
  for (uint8_t i=0;i<N_MOTORS;i++) {
    _array[i] = S.parseInt(); 
    S.read();
  };
};
template <> // specialization for floats
StepperData<float>::StepperData(Stream &S) {
  for (uint8_t i=0;i<N_MOTORS;i++) {
    _array[i] = S.parseFloat(); 
    S.read();
  };
};

// Used for integer position, motion, in steps...
typedef StepperData<long> StepperPositions; // easier to do than class derivation!
// class StepperPositions : public StepperData<long> {
//   using StepperData<long int>::StepperData;
// };
// Used for loads, speeds, lengths...
typedef StepperData<float> StepperFloats; // easier to do than class derivation!
// Used for logical states...
typedef StepperData<bool> StepperBools; // easier to do than class derivation!

char StepperBool2Byte(StepperData<bool> sb) {
  char res = 0b0;
  for (uint8_t i=0;i<N_MOTORS;i++) {
    res += sb[i] << i;
  }
  return res;
}


// TESTS!
void testOperationsOnStepperPositions() {
  StepperPositions x0 = {10,-4,3,5};
  StepperPositions x1 = {1,-5,3,6};
  Serial.println("Operations on Stepper Positions:");
  Serial.println(x0.to_String() + " + " + x1.to_String() + " = " + (x0+x1).to_String());
  Serial.println(x0.to_String() + " - " + x1.to_String() + " = " + (x0-x1).to_String());
  Serial.println(x0.to_String() + " * " + x1.to_String() + " = " + (x0*x1).to_String());
  Serial.println(x0.to_String() + " / " + x1.to_String() + " = " + (x0/x1).to_String());
  Serial.println(x0.to_String() + " == " + x0.to_String() + " = " + (x0==x0).to_String());
  Serial.println(x0.to_String() + " == " + x1.to_String() + " = " + (x0==x1).to_String());
  Serial.println(x0.to_String() + " != " + x1.to_String() + " = " + (x0!=x1).to_String());
  Serial.println("all(" + x0.to_String() + " == " + x0.to_String() + ") = " + String((x0==x0).all()));
  Serial.println("any(" + x0.to_String() + " == " + x0.to_String() + ") = " + String((x0==x0).any()));
  Serial.println("all(" + x0.to_String() + " == " + x1.to_String() + ") = " + String((x0==x1).all()));
  Serial.println("any(" + x0.to_String() + " == " + x1.to_String() + ") = " + String((x0==x1).any()));
  Serial.print("byte(" + x0.to_String() + " > " + x1.to_String() + ") = ");  printOCTET(StepperBool2Byte(x0>x1)) ; Serial.println();
  Serial.println("sign(" + x0.to_String() + ") = " + x0.sign().to_String());
  Serial.println("_abs(" + x0.to_String() + ") = " + x0._abs().to_String());
  Serial.println("_min(" + x0.to_String() + ") = " + String(x0._min()));
  Serial.println("_max(" + x0.to_String() + ") = " + String(x0._max()));
  Serial.println("sum(" + x0.to_String() + ") = " + String(x0.sum()));
  Serial.println("norm(" + x0.to_String() + ") = " + String(x0.norm()));
};
