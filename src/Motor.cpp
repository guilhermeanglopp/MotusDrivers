#include "Motor.h"

static inline float _clampf01(float v) {
  if (v < 0.0f)
    return 0.0f;
  if (v > 1.0f)
    return 1.0f;
  return v;
}

static float _sigmoid01(float x01) {
  // Aproximação simples (ease-in-out): x^2 * (3 - 2x)
  x01 = _clampf01(x01);
  return x01 * x01 * (3.0f - 2.0f * x01);
}

Motor::Motor(uint8_t stepPin, uint8_t dirPin, uint8_t enPin)
    : _stepPin(stepPin), _dirPin(dirPin), _enPin(enPin) {}

void Motor::setSpeedStepsPerSec(float stepsPerSec) { _speedStepsPerSec = stepsPerSec; }

void Motor::setAccelerationTimeMs(uint32_t ms) { _accelMs = ms; }

void Motor::setDecelerationTimeMs(uint32_t ms) { _decelMs = ms; }

void Motor::setAccelerationType(AccelType type) { _accelType = type; }

void Motor::setCustomAccelFn(CustomAccelerationFn fn) { _customFn = fn; }

void Motor::begin() {
  if (_stepPin != 255) {
    pinMode(_stepPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
  }

  if (_dirPin != 255) {
    pinMode(_dirPin, OUTPUT);
    _applyDir(_dirForward);
  }

  if (_enPin != 255) {
    pinMode(_enPin, OUTPUT);
    disable(); // seguro por padrão
  }
}

void Motor::enable() { _applyEnable(true); }

void Motor::disable() { _applyEnable(false); }

void Motor::enable(bool on) {
  if (on)
    enable();
  else
    disable();
}

void Motor::invertDir(bool invert) {
  _dirInverted = invert;
  setDir(_dirForward);
}

void Motor::invertEnable(bool invert) {
  _enInverted = invert;
  _applyEnable(_enabled);
}

void Motor::setDir(bool forward) {
  _dirForward = forward;
  _applyDir(forward);
}

void Motor::_applyEnable(bool on) {
  _enabled = on;
  if (_enPin == 255)
    return;

  // Padrão: enable ativo em LOW (muito comum em drivers de passo)
  // _enInverted permite inverter caso o hardware seja ativo em HIGH.
  const uint8_t level = (on ? LOW : HIGH);
  digitalWrite(_enPin, _enInverted ? !level : level);
}

void Motor::_applyDir(bool forward) {
  if (_dirPin == 255)
    return;

  const uint8_t level = forward ? HIGH : LOW;
  digitalWrite(_dirPin, _dirInverted ? !level : level);
}

bool Motor::isRunning() const { return _state != State::IDLE; }

int32_t Motor::currentPosition() const { return _currentPos; }

int32_t Motor::targetPosition() const { return _targetPos; }

void Motor::moveSteps(int32_t steps) {
  if (steps == 0) {
    stop();
    return;
  }

  _targetPos = _currentPos + steps;
  _stepsRemaining = (steps > 0) ? steps : -steps;
  _moveTotalSteps = (uint32_t)_stepsRemaining;
  _stepsDone = 0;

  setDir(steps > 0);

  // ── Calcula _accelSteps e _decelSteps UMA ÚNICA VEZ ──────────────────────
  // Estes valores NÃO devem ser recalculados em updatePlanner().
  const float speedMax = (_speedStepsPerSec > 1.0f) ? _speedStepsPerSec : 1.0f;
  const float accelS = (float)_accelMs / 1000.0f;
  const float decelS = (float)_decelMs / 1000.0f;

  uint32_t accelSteps = (uint32_t)(0.5f * speedMax * accelS);
  uint32_t decelSteps = (uint32_t)(0.5f * speedMax * decelS);

  const uint32_t half = _moveTotalSteps / 2;
  if (accelSteps > half)
    accelSteps = half;
  if (decelSteps > half)
    decelSteps = half;

  _accelSteps = accelSteps;
  _decelSteps = decelSteps;
  // ─────────────────────────────────────────────────────────────────────────

  _state = (_accelMs == 0) ? State::CRUISE : State::ACCEL;
  _currentInterval = 0;
  _plannerReady = false;
  _nextStepTime = micros();
}

void Motor::stop() {
  _stepsRemaining = 0;
  _moveTotalSteps = 0;
  _stepsDone = 0;
  _targetPos = _currentPos;
  _currentInterval = 0;
  _plannerReady = false;
  _state = State::IDLE;
}

void Motor::stopSmooth() {
  if (_state == State::IDLE)
    return;
  _state = State::DECEL;
}

uint32_t Motor::nextStepInterval() const { return _currentInterval; }

bool Motor::isStepDue() const { return isStepDue(micros()); }

bool Motor::isStepDue(uint32_t nowUs) const {
  if (_state == State::IDLE)
    return false;
  if (_currentInterval == 0)
    return false;

  // Comparação segura com overflow: (int32_t)(now - target) >= 0
  return (int32_t)(nowUs - _nextStepTime) >= 0;
}

void Motor::pulseStep() {
  if (_stepPin == 255)
    return;
  digitalWrite(_stepPin, HIGH);
  if (_stepPulseUs > 0)
    delayMicroseconds(_stepPulseUs);
  digitalWrite(_stepPin, LOW);
}

void Motor::pulseStepIsr() {
  if (_stepPin == 255)
    return;
  digitalWrite(_stepPin, HIGH);
  if (_stepPulseUs > 0) {
    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < _stepPulseUs) {
    }
  }
  digitalWrite(_stepPin, LOW);
}

void Motor::stepApplied() {
  if (_state == State::IDLE)
    return;
  if (_stepsRemaining <= 0) {
    stop();
    return;
  }

  // Atualiza posição e contadores (inteiro, ISR-safe)
  _currentPos += _dirForward ? 1 : -1;
  _stepsRemaining--;
  _stepsDone++;

  // Fim do movimento
  if (_stepsRemaining <= 0) {
    stop();
    return;
  }

  // Transições de estado baseadas em passos
  if (_state == State::ACCEL && _accelSteps > 0 && _stepsDone >= _accelSteps) {
    _state = State::CRUISE;
  }

  if (_decelSteps > 0 && (uint32_t)_stepsRemaining <= _decelSteps) {
    _state = State::DECEL;
  }

  // Agenda o próximo passo usando o intervalo já calculado pelo Planner
  // (se ainda não calculado, o loop cooperativo/Planner vai preencher antes do próximo tick)
  if (_currentInterval == 0)
    _currentInterval = 1000000UL; // fallback bem lento para não travar
  _nextStepTime += _currentInterval;
}

void Motor::updatePlanner() {
  if (_state == State::IDLE)
    return;
  if (_moveTotalSteps == 0)
    return;

  // ── Leitura atômica das variáveis compartilhadas com a ISR/Task ───────────
  noInterrupts();
  const uint32_t stepsRemaining = (uint32_t)_stepsRemaining;
  const uint32_t stepsDone = _stepsDone;
  const State state = _state;
  interrupts();
  // ─────────────────────────────────────────────────────────────────────────

  const float speedMax = (_speedStepsPerSec > 1.0f) ? _speedStepsPerSec : 1.0f;
  const float speedMin = (speedMax >= 10.0f) ? (speedMax / 10.0f) : 1.0f;
  const float intervalMinF = 1000000.0f / speedMax;
  const float intervalMaxF = 1000000.0f / speedMin;

  // _accelSteps e _decelSteps já foram calculados em moveSteps() — não recalcular aqui.

  float factor = 1.0f;

  if (state == State::ACCEL && _accelSteps > 0) {
    const float progress = (float)stepsDone / (float)_accelSteps;
    if (_accelType == AccelType::Sigmoidal) {
      factor = _sigmoid01(_clampf01(progress));
    } else if (_accelType == AccelType::Custom) {
      factor = _customFn ? _clampf01(_customFn(_clampf01(progress))) : _clampf01(progress);
    } else {
      factor = _clampf01(progress);
    }
  } else if (state == State::DECEL && _decelSteps > 0) {
    const uint32_t doneInDecel =
        (_decelSteps > stepsRemaining) ? (_decelSteps - stepsRemaining) : 0;
    const float progress = (float)doneInDecel / (float)_decelSteps;
    float shaped = _clampf01(progress);
    if (_accelType == AccelType::Sigmoidal) {
      shaped = _sigmoid01(shaped);
    } else if (_accelType == AccelType::Custom) {
      shaped = _customFn ? _clampf01(_customFn(shaped)) : shaped;
    }
    factor = 1.0f - shaped;
  } else {
    factor = 1.0f; // CRUISE
  }

  const float intervalF =
      intervalMaxF - (intervalMaxF - intervalMinF) * _clampf01(factor);
  uint32_t interval = (uint32_t)(intervalF + 0.5f);
  if (interval < 1)
    interval = 1;
  _currentInterval = interval;

  // ── Inicializa _nextStepTime apenas na primeira execução do Planner ──────
  // Usa flag _plannerReady em vez de checar _nextStepTime == 0,
  // evitando ambiguidade com overflow do micros().
  if (!_plannerReady) {
    _nextStepTime = micros() + _currentInterval;
    _plannerReady = true;
  }
}

void Motor::run() { updatePlanner(); }

