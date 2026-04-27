#include "MotusDrivers.h"

#include <new>

#if defined(ESP32)
#include "esp_attr.h"
#define FK_TICK_IRAM IRAM_ATTR
#else
#define FK_TICK_IRAM
#endif

// Devolve referência ao primeiro motor genérico quando a lista está cheia (uso interno de segurança).
GenericStepDir &MotusDrivers::_genericOverflow() {
  return *reinterpret_cast<GenericStepDir *>(_genericRaw[0]);
}

// Devolve referência ao primeiro SoftwareMS quando a lista está cheia (uso interno de segurança).
SoftwareMS &MotusDrivers::_softOverflow() {
  return *reinterpret_cast<SoftwareMS *>(_softRaw[0]);
}

// Instancia TB6600 em memória fixa, regista-o e devolve a referência.
GenericStepDir &MotusDrivers::setupTB6600(uint8_t step, uint8_t dir, uint8_t en) {
  if (_genericCount >= MAX_MOTORS)
    return _genericOverflow();
  void *p = _genericRaw[_genericCount];
  GenericStepDir *m = new (p) GenericStepDir(step, dir, en);
  _motors[_motorCount++] = m;
  _genericCount++;
  return *m;
}

// Instancia DM556 (GenericStepDir) em memória fixa, regista-o e devolve a referência.
GenericStepDir &MotusDrivers::setupDM556(uint8_t step, uint8_t dir, uint8_t en) {
  if (_genericCount >= MAX_MOTORS)
    return _genericOverflow();
  void *p = _genericRaw[_genericCount];
  GenericStepDir *m = new (p) GenericStepDir(step, dir, en);
  _motors[_motorCount++] = m;
  _genericCount++;
  return *m;
}

// Instancia SoftwareMS para A4988, regista-o e devolve a referência.
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

// Instancia SoftwareMS para DRV8825, regista-o e devolve a referência.
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

// Inicializa hardware (pinos) de todos os motores na ordem de registo.
void MotusDrivers::beginAll() {
  const uint8_t n = _motorCount;
  for (uint8_t i = 0; i < n; i++) {
    if (_motors[i])
      _motors[i]->begin();
  }
}

// Atualiza planeamento de todos; fora do modo ISR também aplica passos com delay cooperativo.
void MotusDrivers::run() {
  const uint8_t n = _motorCount;
  for (uint8_t i = 0; i < n; i++) {
    if (_motors[i])
      _motors[i]->updatePlanner();
  }

  if (_isrMode)
    return;

  uint32_t now = micros();
  for (uint8_t i = 0; i < n; i++) {
    Motor *m = _motors[i];
    if (!m)
      continue;
    if (m->isStepDue(now)) {
      m->pulseStep();
      m->stepApplied();
      now = micros();
    }
  }
}

// Varre os motores e, se for altura do próximo passo, emite pulso ISR-safe e avança estado.
bool FK_TICK_IRAM MotusDrivers::tick() {
  const uint8_t n = _motorCount;
  uint32_t now = micros();
  bool stepped = false;

  for (uint8_t i = 0; i < n; i++) {
    Motor *m = _motors[i];
    if (!m)
      continue;
    if (m->isStepDue(now)) {
      m->pulseStepIsr();
      m->stepApplied();
      stepped = true;
      // `pulseStepIsr()` consome alguns µs (e com múltiplos motores soma).
      // Reamostrar `micros()` aqui reduz viés temporal entre eixos no mesmo scan.
      now = micros();
    }
  }

  return stepped;
}
