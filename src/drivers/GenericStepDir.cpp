#include "GenericStepDir.h"

// Inicializa Motor base e grava duração mínima do pulso STEP em microssegundos.
GenericStepDir::GenericStepDir(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                               uint32_t stepPulseUs)
    : Motor(stepPin, dirPin, enPin) {
  _stepPulseUs = stepPulseUs;
}

