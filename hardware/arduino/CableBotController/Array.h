// Define a template class for 1D Arrays

template <class T,size_t N>
class Array {
  public:
    T _array[N]; // the data
  public:        
    //Overload the indexing operator
    inline const T& operator [](uint8_t index) const {return _array[index];};
    inline T& operator [](uint8_t index) {return _array[index];};
    // Element-wise operations
    Array operator+(Array other) {Array res; for (uint8_t i=0;i<N;i++) res._array[i] = _array[i]+other._array[i];return res;};
    Array operator-(Array other) {Array res; for (uint8_t i=0;i<N;i++) res._array[i] = _array[i]-other._array[i];return res;};
    Array operator*(Array other) {Array res; for (uint8_t i=0;i<N;i++) res._array[i] = _array[i]*other._array[i];return res;};
    Array operator/(Array other) {Array res; for (uint8_t i=0;i<N;i++) res._array[i] = _array[i]/other._array[i];return res;};
    Array<bool,N> operator>(Array other) {Array<bool,N> res; for (uint8_t i=0;i<N;i++) res[i] = _array[i]>other[i];return res;};
    Array<bool,N> operator==(Array other) {Array<bool,N> res; for (uint8_t i=0;i<N;i++) res[i] = _array[i]==other[i];return res;};
    Array<bool,N> operator!=(Array other) {Array<bool,N> res; for (uint8_t i=0;i<N;i++) res[i] = _array[i]!=other[i];return res;};
    Array sign() {Array res; for (uint8_t i=0;i<N;i++) res._array[i] = T(_array[i]>T(0))-T(_array[i]<T(0)); return res;};
    // Other functions
    bool all() {for (uint8_t i=0;i<N;i++) if (!_array[i]) return false; return true;};
    bool any() {for (uint8_t i=0;i<N;i++) if (_array[i]) return true; return false;};
    // Printing function
    String to_String();
    void print() {Serial.print(to_String());};
};

// Print function
template <class T,size_t N>
String Array<T,N>::to_String() {
  String str = "["; 
  for (uint8_t i=0;i<N;i++) str += String(_array[i]) + ",";
  return str + "]"; 
};


// Constructor Functions
template <class T,size_t N> Array<T,N> uniformArray(T uniform) {Array<T,N> res; for (uint8_t i=0;i<N;i++) res._array[i] = uniform; return res;}; // default
template <class T,size_t N> Array<T,N> to_Array(T &array) {Array<T,N> res; for (uint8_t i=0;i<N;i++) res._array[i] = array[i]; return res;}; // construct with array
// construct from serial message
template <class T,size_t N> Array<T,N> arrayFromSerial() {Array<T,N> res; for (uint8_t i=0;i<N;i++) {res._array[i] = Serial.parseInt(); Serial.read();} return res;}; // works for integer data
template <size_t N> Array<float,N> arrayFromSerial() {Array<float,N> res; for (uint8_t i=0;i<N;i++) {res._array[i] = Serial.parseFloat(); Serial.read();} return res;}; // specializes to float data
template <size_t N> Array<char,N> arrayFromSerial() {Array<char,N> res; for (uint8_t i=0;i<N;i++) {res._array[i] = Serial.read();} return res;}; // specializes to char data
