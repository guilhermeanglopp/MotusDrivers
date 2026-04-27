#pragma once

#include "../Motor.h"

class GenericStepDir : public Motor {
public:
  // Motor só STEP/DIR/EN com largura de pulso configurável (TB6600, DM556, etc.).
  explicit GenericStepDir(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                          uint32_t stepPulseUs = 1);
};

