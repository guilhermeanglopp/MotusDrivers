#include "GenericStepDir.h"

GenericStepDir::GenericStepDir(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                               uint32_t stepPulseUs)
    : Motor(stepPin, dirPin, enPin) {
  _stepPulseUs = stepPulseUs;
}

