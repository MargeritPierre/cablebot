// DEFINE THE CONFIGURATION OF THE EIGHT TMC5160 DRIVERS
#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>

// Some necessary info/includes
#define R_SENSE 0.075 // current sense resistance of 75 milli Ohms of the Breakout board
using namespace TMC2130_n; // for stall guard measurements

// DRIVER WIRING: see MEGA_PORTS_PINS.txt
//Driver DIR: port F (pins A0:54 to A7:61) -> DRV_REFR pins of (drivers 0 to 7)
#define DIRregister PORTF      //pin
#define DIRsetupRegister DDRF  //mode
extern const uint8_t DIR_PIN_array[] = { 54, 55, 56, 57, 58, 59, 60, 61 };
//Driver STEP: port K (pins A8:62 to A15:69) -> DRV_REFL pins of (drivers 0 to 7)
#define STEPregister PORTK      //pin
#define STEPsetupRegister DDRK  //mode
extern const uint8_t STEP_PIN_array[] = { 62, 63, 64, 65, 66, 67, 68, 69 };
//Driver ENABLE: port C (pins 37 to 30) -> DRV_ENN pins of (drivers 0 to 7)
#define ENABLEregister PORTC      //pin
#define ENABLEsetupRegister DDRC  //mode
extern const uint8_t EN_PIN_array[] = { 37, 36, 35, 34, 33, 32, 31, 30 };
//SPI Chip select: port A !inverted! (pins 29 to 22) -> CSN pins of (drivers 0 to 7)
#define CSregister PORTA      //pin
#define CSsetupRegister DDRA  //mode
extern const uint8_t CS_PIN_array[] = { 29, 28, 27, 26, 25, 24, 23, 22 };

class Drivers {
  public:
  // Driver configuration
    uint8_t blank_time = 24;      // [16, 24, 36, 54]
    uint8_t off_time = 4;         // [1..15]
    int8_t hysteresis_start = 1;  // [1..8]
    int8_t hysteresis_end = 12;   // [-3..12]
    uint16_t rms_current = 1000;  // [0..5000]
    uint8_t microstep = 16;        // [0,1,2,4,8,16,32,64,128,256]
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
  private:
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

void Drivers::setup() {
  // sets up PINs port in OUTPUT mode
  DIRsetupRegister = 0b11111111;
  STEPsetupRegister = 0b11111111;
  CSsetupRegister = 0b11111111;
  ENABLEsetupRegister = 0b1111111;
  // initialize the state of PINs
  DIRregister = 0b0;            // init all STEP as LOW
  STEPregister = 0b0;           // init all STEP as HIGH
  CSregister = 0b11111111;      //disable all SPI CS PINs
  ENABLEregister = 0b11111111;  //disable all drivers
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
  ENABLEregister = 0b00000000;
}

void Drivers::disable() {
  ENABLEregister = 0b11111111;
}

// =======================================================================================
// void testDriverConnection(int drv) {
//   Serial.print("Testing connection with driver " + String(drv) + "...");
//   uint8_t result = drivers[drv].test_connection();
//   if (result) {
//     Serial.print("failed! ");
//     Serial.print("Likely cause: ");
//     switch (result) {
//       case 1: Serial.println("loose connection"); break;
//       case 2: Serial.println("no power"); break;
//     }
//     abort();
//   }
//   Serial.println("OK");
// }

// void setupMotorDrivers() {
//   // sets up PINs port in OUTPUT mode
//   DIRsetupRegister = 0b11111111;
//   STEPsetupRegister = 0b11111111;
//   CSsetupRegister = 0b11111111;
//   ENABLEsetupRegister = 0b1111111;
//   // initialize the state of PINs
//   DIRregister = 0b0;            // init all STEP as LOW
//   STEPregister = 0b0;           // init all STEP as HIGH
//   CSregister = 0b11111111;      //disable all SPI CS PINs
//   ENABLEregister = 0b11111111;  //disable all drivers
//   // Begin the SPI communication & set the driver configuration
//   SPI.begin(); delay(100);
//   for (uint8_t i = 0; i < N_MOTORS; i++) {
//     testDriverConnection(i);
//     drivers[i].push();                  // reset the driver to defaults
//     drivers[i].toff(TMCDriverConfig.off_time);
//     drivers[i].rms_current(TMCDriverConfig.rms_current);       // Set stepper current.
//     drivers[i].microsteps(TMCDriverConfig.microstep);           // set microstep to FULLSTEP
//     drivers[i].en_pwm_mode(TMCDriverConfig.en_pwm_mode);          // Enable extremely quiet stepping
//     drivers[i].pwm_autoscale(TMCDriverConfig.pwm_autoscale);        // run automatic setup of PWM mode
//     drivers[i].TCOOLTHRS(TMCDriverConfig.TCOOLTHRS);  // 
//     drivers[i].TPWMTHRS(TMCDriverConfig.TPWMTHRS);  // FFFFF
//     drivers[i].THIGH(TMCDriverConfig.THIGH);  // FFFFF
//     drivers[i].semin(TMCDriverConfig.semin);
//     drivers[i].semax(TMCDriverConfig.semax);
//     drivers[i].seup(TMCDriverConfig.seup);
//     drivers[i].sedn(TMCDriverConfig.sedn);
//     drivers[i].sgt(TMCDriverConfig.sgt);
//   }
// }

// void printSGValues() {
//   for (uint8_t i = 0; i < N_MOTORS; i++) {
//     DRV_STATUS_t drv_status{ 0 };
//     drv_status.sr = drivers[i].DRV_STATUS();
//     Serial.print("sg");
//     Serial.print(i);
//     Serial.print(":");
//     Serial.print(drv_status.sg_result, DEC);
//     Serial.print("/");
//     // Serial.print(drivers[i].cs2rms(drv_status.cs_actual), DEC);
//     Serial.print(" ");
//   }
//   Serial.println();

//   // Serial.print("0 ");
//   // Serial.print(drv_status.sg_result, DEC);
//   // Serial.print(" ");
//   // Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);
// }
