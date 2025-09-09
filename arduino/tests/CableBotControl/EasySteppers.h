// RUN STEPPERS USING ACCELSTEPPER LIBRARY


class Steppers {
  private:
    AccelStepper _array[8] = {
      { AccelStepper::DRIVER, STEP_PIN_array[0], DIR_PIN_array[0] },
      { AccelStepper::DRIVER, STEP_PIN_array[1], DIR_PIN_array[1] },
      { AccelStepper::DRIVER, STEP_PIN_array[2], DIR_PIN_array[2] },
      { AccelStepper::DRIVER, STEP_PIN_array[3], DIR_PIN_array[3] },
      { AccelStepper::DRIVER, STEP_PIN_array[4], DIR_PIN_array[4] },
      { AccelStepper::DRIVER, STEP_PIN_array[5], DIR_PIN_array[5] },
      { AccelStepper::DRIVER, STEP_PIN_array[6], DIR_PIN_array[6] },
      { AccelStepper::DRIVER, STEP_PIN_array[7], DIR_PIN_array[7] },
    };
    uint16_t _acceleration = MAX_ACCELERATION;
    uint16_t _maxspeed = MAX_SPEED;
  public:
    void setup();
    void run();
    void move(StepperPositions motion);
    void moveTo(StepperPositions positions);
    bool positionsReached();
    void setCurrentPositions(StepperPositions positions);
    StepperPositions getCurrentPositions();
};

void Steppers::setup() {
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    _array[i].setMaxSpeed(_maxspeed);
    _array[i].setAcceleration(_acceleration);
    _array[i].setEnablePin(EN_PIN_array[i]);
    _array[i].setPinsInverted(false, false, true);
    _array[i].enableOutputs();
    _array[i].setCurrentPosition(0);
  }
}

void Steppers::run() {
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    _array[i].run();
  }
}

void Steppers::moveTo(StepperPositions positions) {
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    _array[i].moveTo(positions[i]);
  }
}

void Steppers::move(StepperPositions motion) {
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    _array[i].move(motion[i]);
  }
}

void Steppers::setCurrentPositions(StepperPositions positions) {
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    _array[i].setCurrentPosition(positions[i]);
  }
}

StepperPositions Steppers::getCurrentPositions() {
  StepperPositions positions;
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    positions[i] = _array[i].currentPosition();
  }
  return positions;
}

bool Steppers::positionsReached() {
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    if (_array[i].distanceToGo() != 0) return false;
  }
  return true;
}

