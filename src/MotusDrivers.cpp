#include "MotusDrivers.h"

#include <new>

GenericStepDir &MotusDrivers::_genericOverflow() {
  return *reinterpret_cast<GenericStepDir *>(_genericRaw[0]);
}

SoftwareMS &MotusDrivers::_softOverflow() {
  return *reinterpret_cast<SoftwareMS *>(_softRaw[0]);
}

GenericStepDir &MotusDrivers::setupTB6600(uint8_t step, uint8_t dir, uint8_t en) {
  if (_genericCount >= MAX_MOTORS)
    return _genericOverflow();
  void *p = _genericRaw[_genericCount];
  GenericStepDir *m = new (p) GenericStepDir(step, dir, en);
  _motors[_motorCount++] = m;
  _genericCount++;
  return *m;
}

GenericStepDir &MotusDrivers::setupDM556(uint8_t step, uint8_t dir, uint8_t en) {
  if (_genericCount >= MAX_MOTORS)
    return _genericOverflow();
  void *p = _genericRaw[_genericCount];
  GenericStepDir *m = new (p) GenericStepDir(step, dir, en);
  _motors[_motorCount++] = m;
  _genericCount++;
  return *m;
}

SoftwareMS &MotusDrivers::setupA4988(uint8_t step, uint8_t dir, uint8_t en,
                                 uint8_t ms1, uint8_t ms2, uint8_t ms3) {
  if (_softCount >= MAX_MOTORS)
    return _softOverflow();
  void *p = _softRaw[_softCount];
  SoftwareMS *m =
      new (p) SoftwareMS(step, dir, en, ms1, ms2, ms3, DriverVariant::A4988);
  _motors[_motorCount++] = m;
  _softCount++;
  return *m;
}

SoftwareMS &MotusDrivers::setupDRV8825(uint8_t step, uint8_t dir, uint8_t en,
                                    uint8_t ms0, uint8_t ms1, uint8_t ms2) {
  if (_softCount >= MAX_MOTORS)
    return _softOverflow();
  void *p = _softRaw[_softCount];
  SoftwareMS *m =
      new (p) SoftwareMS(step, dir, en, ms0, ms1, ms2, DriverVariant::DRV8825);
  _motors[_motorCount++] = m;
  _softCount++;
  return *m;
}

void MotusDrivers::beginAll() {
  for (uint8_t i = 0; i < _motorCount; i++) {
    if (_motors[i])
      _motors[i]->begin();
  }
}

void MotusDrivers::run() {
  for (uint8_t i = 0; i < _motorCount; i++) {
    if (_motors[i])
      _motors[i]->updatePlanner();
  }

  if (_isrMode)
    return;

  const uint32_t now = micros();
  for (uint8_t i = 0; i < _motorCount; i++) {
    Motor *m = _motors[i];
    if (!m)
      continue;
    if (m->isStepDue(now)) {
      m->pulseStep();
      m->stepApplied();
    }
  }
}

bool MotusDrivers::tick() {
  const uint32_t now = micros();
  bool stepped = false;
  for (uint8_t i = 0; i < _motorCount; i++) {
    Motor *m = _motors[i];
    if (!m)
      continue;
    if (m->isStepDue(now)) {
      m->pulseStepIsr();
      m->stepApplied();
      stepped = true;
    }
  }
  return stepped;
}
