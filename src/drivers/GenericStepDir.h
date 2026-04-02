#pragma once

#include "../Motor.h"

class GenericStepDir : public Motor {
public:
  explicit GenericStepDir(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                          uint32_t stepPulseUs = 3);
};

