// DEFINE THE CONFIGURATION OF THE EIGHT TMC5160 DRIVERS
#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>

// Some necessary info/includes
#define R_SENSE 0.075 // current sense resistance of 75 milli Ohms of the Breakout board
using namespace TMC2130_n; // for stall guard measurements

// DRIVER WIRING: see MEGA_PORTS_PINS.txt
//Driver DIR: port F (pins A0:54 to A7:61) -> DRV_REFR pins of (drivers 0 to 7)
#define PORT_DIR PORTF      //pin
#define PORT_DIR_MODE DDRF  //mode
extern const uint8_t DIR_PIN_array[] = { 54, 55, 56, 57, 58, 59, 60, 61 };
//Driver STEP: port K (pins A8:62 to A15:69) -> DRV_REFL pins of (drivers 0 to 7)
#define PORT_STEP PORTK      //pin
#define PORT_STEP_MODE DDRK  //mode
extern const uint8_t STEP_PIN_array[] = { 62, 63, 64, 65, 66, 67, 68, 69 };
//Driver ENABLE: port C (pins 37 to 30) -> DRV_ENN pins of (drivers 0 to 7)
#define PORT_ENABLE PORTC      //pin
#define PORT_ENABLE_MODE DDRC  //mode
extern const uint8_t EN_PIN_array[] = { 37, 36, 35, 34, 33, 32, 31, 30 };
//SPI Chip select: port A !inverted! (pins 29 to 22) -> CSN pins of (drivers 0 to 7)
#define PORT_CS PORTA      //pin
#define PORT_CS_MODE DDRA  //mode
extern const uint8_t CS_PIN_array[] = { 29, 28, 27, 26, 25, 24, 23, 22 };

class Drivers {
  public:
  // Driver configuration
    uint8_t blank_time = 24;      // [16, 24, 36, 54]
    uint8_t off_time = 4;         // [1..15]
    int8_t hysteresis_start = 1;  // [1..8]
    int8_t hysteresis_end = 12;   // [-3..12]
    uint16_t rms_current = RMS_CURRENT;  // [0..5000]
    uint8_t microstep = MICROSTEP;        // [0,1,2,4,8,16,32,64,128,256]
    uint8_t dedge = DEDGE;        // [0,1,2,4,8,16,32,64,128,256]
    bool en_pwm_mode = true;     // Enable extremely quiet stepping
    bool pwm_autoscale = true;   // run automatic setup of PWM mode
    uint16_t TCOOLTHRS = 0xFFFFF;
    uint16_t TPWMTHRS = 0x0;
    uint16_t THIGH = 0x0;
    int8_t semin = 15;
    int8_t semax = 15;
    int8_t seup = 0b0;
    int8_t sedn = 0b0;
    int8_t sgt = 6;
  // Driver functions
    void setup();
    void applyConfig();
    void testConnection();
    void enable();
    void disable();
  public:
    const TMC5160Stepper _array[8] = {
      { CS_PIN_array[0], R_SENSE },
      { CS_PIN_array[1], R_SENSE },
      { CS_PIN_array[2], R_SENSE },
      { CS_PIN_array[3], R_SENSE },
      { CS_PIN_array[4], R_SENSE },
      { CS_PIN_array[52], R_SENSE },
      { CS_PIN_array[6], R_SENSE },
      { CS_PIN_array[7], R_SENSE },
    };
};

// Define the unique driver controller
Drivers drivers; 

void Drivers::setup() {
  // sets up PINs port in OUTPUT mode
  PORT_DIR_MODE = 0b11111111;
  PORT_STEP_MODE = 0b11111111;
  PORT_CS_MODE = 0b11111111;
  PORT_ENABLE_MODE = 0b1111111;
  // initialize the state of PINs
  PORT_DIR = 0b0;            // init all STEP as LOW
  PORT_STEP = 0b0;           // init all STEP as HIGH
  PORT_CS = 0b11111111;      //disable all SPI CS PINs
  PORT_ENABLE = 0b11111111;  //disable all drivers
}

void Drivers::testConnection() {
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    Serial.print("Testing connection with driver " + String(i) + "...");
    uint8_t result = _array[i].test_connection();
    if (result) {
      Serial.print("failed! ");
      Serial.print("Likely cause: ");
      switch (result) {
        case 1: Serial.println("loose connection"); break;
        case 2: Serial.println("no power"); break;
      }
      abort();
    }
    Serial.println("OK");
  }
}

void Drivers::applyConfig() {
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    _array[i].push();                  // reset the driver to defaults
    _array[i].toff(off_time);
    _array[i].rms_current(rms_current);
    _array[i].microsteps(microstep);
    _array[i].dedge(dedge);
    _array[i].en_pwm_mode(en_pwm_mode);
    _array[i].pwm_autoscale(pwm_autoscale);
    _array[i].TCOOLTHRS(TCOOLTHRS);
    _array[i].TPWMTHRS(TPWMTHRS);
    _array[i].THIGH(THIGH);
    _array[i].semin(semin);
    _array[i].semax(semax);
    _array[i].seup(seup);
    _array[i].sedn(sedn);
    _array[i].sgt(sgt);
  }
}

void Drivers::enable() {
  PORT_ENABLE = 0b00000000;
}

void Drivers::disable() {
  PORT_ENABLE = 0b11111111;
}