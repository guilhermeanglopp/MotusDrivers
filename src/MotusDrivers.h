#pragma once

#include <Arduino.h>

#include "drivers/GenericStepDir.h"
#include "drivers/SoftwareMS.h"

/** Gerenciador da biblioteca Motus Drivers (motores de passo). */
class MotusDrivers {
public:
  static constexpr uint8_t MAX_MOTORS = 8;

  GenericStepDir &setupTB6600(uint8_t step, uint8_t dir, uint8_t en);
  GenericStepDir &setupDM556(uint8_t step, uint8_t dir, uint8_t en);

  SoftwareMS &setupA4988(uint8_t step, uint8_t dir, uint8_t en,
                         uint8_t ms1 = 255, uint8_t ms2 = 255,
                         uint8_t ms3 = 255);
  SoftwareMS &setupDRV8825(uint8_t step, uint8_t dir, uint8_t en,
                            uint8_t ms0 = 255, uint8_t ms1 = 255,
                            uint8_t ms2 = 255);

  /** true: `run()` só atualiza o planner; pulsos vêm de `tick()` (ISR/Task). */
  void setIsrMode(bool isrMode) { _isrMode = isrMode; }
  bool isrMode() const { return _isrMode; }

  void beginAll();

  void run();
  bool tick();

  uint8_t motorCount() const { return _motorCount; }

private:
  alignas(GenericStepDir) uint8_t _genericRaw[MAX_MOTORS][sizeof(GenericStepDir)];
  alignas(SoftwareMS) uint8_t _softRaw[MAX_MOTORS][sizeof(SoftwareMS)];

  uint8_t _genericCount = 0;
  uint8_t _softCount = 0;

  Motor *_motors[MAX_MOTORS] = {};
  uint8_t _motorCount = 0;

  bool _isrMode = false;

  GenericStepDir &_genericOverflow();
  SoftwareMS &_softOverflow();
};
