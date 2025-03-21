// DEFINE A TEMPLATE FOR STEPPER DATA (position, motion, speed, acceleration...)

// Template class
template <class T>
class StepperData {
  public:        
    // Constructors
    StepperData() {for (uint8_t i=0;i<N_MOTORS;i++) _array[i] = (T)0;}; // default
    StepperData(T state[N_MOTORS]) {for (uint8_t i=0;i<N_MOTORS;i++) _array[i] = state[i];}; // construct with array
    StepperData(T s0,T s1,T s2,T s3) {_array[0] = s0;_array[1] = s1;_array[2] = s2;_array[3] = s3;}; // construct with 4 motors
    //Overload the indexing operator
    const T& operator [](uint8_t index) const {return _array[index];};
    T& operator [](uint8_t index) {return _array[index];};
    // Printing function
    String to_String();
  private:
    T _array[N_MOTORS];
};

template <class T>
String StepperData<T>::to_String() {
  String str = "["; 
  for (uint8_t i=0;i<N_MOTORS;i++) str += String(_array[i]) + ",";
  return str + "]"; 
};

// Used for position, motion, speeds...
class StepperPositions : public StepperData<long> {
  using StepperData<long>::StepperData; // inherits the template constructor
};

