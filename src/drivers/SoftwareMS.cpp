#include "SoftwareMS.h"

namespace {

const uint8_t kA4988Valid[] = {1, 2, 4, 8, 16};
const uint8_t kDrvValid[] = {1, 2, 4, 8, 16, 32};

// Escolhe o elemento da lista mais próximo do valor pedido (lista ordenada de divisores válidos).
uint8_t nearestInList(uint8_t ms, const uint8_t *vals, uint8_t n) {
  uint8_t best = vals[0];
  uint16_t bestDiff = 0xFFFF;
  for (uint8_t i = 0; i < n; i++) {
    const uint16_t d = (ms > vals[i]) ? (ms - vals[i]) : (vals[i] - ms);
    if (d < bestDiff) {
      bestDiff = d;
      best = vals[i];
    }
  }
  return best;
}

} // namespace

// Grava pinos MS, variante do chip e largura de pulso típica (A4988 vs DRV8825).
SoftwareMS::SoftwareMS(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                       uint8_t ms0, uint8_t ms1, uint8_t ms2,
                       DriverVariant variant)
    : Motor(stepPin, dirPin, enPin), _variant(variant) {
  _msPins[0] = ms0;
  _msPins[1] = ms1;
  _msPins[2] = ms2;
  _currentMs = 1;

  if (variant == DriverVariant::A4988) {
    _stepPulseUs = 1;
  } else {
    _stepPulseUs = 2;
  }
}

// pinMode nos MS, chama begin() da classe base e aplica micropassos por defeito (1).
void SoftwareMS::begin() {
  for (uint8_t i = 0; i < 3; i++) {
    if (_msPins[i] != 255) {
      pinMode(_msPins[i], OUTPUT);
    }
  }
  Motor::begin();
  _begun = true;
  applyMicrostepPins(_currentMs);
}

// digitalWrite HIGH/LOW num pino de modo, ignorando pinos não usados (255).
void SoftwareMS::writeMsPin(uint8_t pin, bool high) {
  if (pin == 255)
    return;
  digitalWrite(pin, high ? HIGH : LOW);
}

// Normaliza ms≠0 e escolhe o divisor permitido mais próximo para o driver atual.
uint8_t SoftwareMS::snapMicrosteps(uint8_t ms) const {
  if (ms == 0)
    ms = 1;
  if (_variant == DriverVariant::A4988) {
    return nearestInList(ms, kA4988Valid, sizeof(kA4988Valid));
  }
  return nearestInList(ms, kDrvValid, sizeof(kDrvValid));
}

// Atualiza _currentMs e escreve a combinação de MS1/2/3 ou MODE0/1/2 da folha de dados.
void SoftwareMS::applyMicrostepPins(uint8_t ms) {
  ms = snapMicrosteps(ms);
  _currentMs = ms;

  if (_variant == DriverVariant::A4988) {
    // MS1, MS2, MS3 -> _msPins[0..2]
    switch (ms) {
    case 1:
      writeMsPin(_msPins[0], false);
      writeMsPin(_msPins[1], false);
      writeMsPin(_msPins[2], false);
      break;
    case 2:
      writeMsPin(_msPins[0], true);
      writeMsPin(_msPins[1], false);
      writeMsPin(_msPins[2], false);
      break;
    case 4:
      writeMsPin(_msPins[0], false);
      writeMsPin(_msPins[1], true);
      writeMsPin(_msPins[2], false);
      break;
    case 8:
      writeMsPin(_msPins[0], true);
      writeMsPin(_msPins[1], true);
      writeMsPin(_msPins[2], false);
      break;
    case 16:
    default:
      writeMsPin(_msPins[0], true);
      writeMsPin(_msPins[1], true);
      writeMsPin(_msPins[2], true);
      break;
    }
    return;
  }

  // DRV8825 MODE0, MODE1, MODE2 -> _msPins[0..2]
  switch (ms) {
  case 1:
    writeMsPin(_msPins[0], false);
    writeMsPin(_msPins[1], false);
    writeMsPin(_msPins[2], false);
    break;
  case 2:
    writeMsPin(_msPins[0], true);
    writeMsPin(_msPins[1], false);
    writeMsPin(_msPins[2], false);
    break;
  case 4:
    writeMsPin(_msPins[0], false);
    writeMsPin(_msPins[1], true);
    writeMsPin(_msPins[2], false);
    break;
  case 8:
    writeMsPin(_msPins[0], true);
    writeMsPin(_msPins[1], true);
    writeMsPin(_msPins[2], false);
    break;
  case 16:
    writeMsPin(_msPins[0], false);
    writeMsPin(_msPins[1], false);
    writeMsPin(_msPins[2], true);
    break;
  case 32:
  default:
    writeMsPin(_msPins[0], true);
    writeMsPin(_msPins[1], false);
    writeMsPin(_msPins[2], true);
    break;
  }
}

// Ajusta micropassos ao valor suportado e, se begin() já correu, atualiza os pinos.
void SoftwareMS::setMicrosteps(uint8_t ms) {
  const uint8_t snapped = snapMicrosteps(ms);
  _currentMs = snapped;
  if (_begun)
    applyMicrostepPins(snapped);
}

// Leitor simples do divisor de micropassos em uso.
uint8_t SoftwareMS::getMicrosteps() const { return _currentMs; }
