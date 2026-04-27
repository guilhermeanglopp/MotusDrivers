#include "Motor.h"

#if defined(ESP32)
#include "driver/gpio.h"
#include "esp_attr.h"
#include "rom/ets_sys.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#define FK_MOTOR_IRAM IRAM_ATTR
#else
#define FK_MOTOR_IRAM
#endif

#if defined(ESP32)
/** STEP/DIR costumam ser GPIO < 32: escrita direta no registo. Demais pinos: driver. */
// Escreve nível GPIO o mais rápido possível no ESP32 (registo vs driver API).
static inline void FK_MOTOR_IRAM fk_gpio_set_fast(uint8_t pin, uint32_t level) {
  if (pin == 255)
    return;
  if (pin < 32) {
    if (level)
      REG_WRITE(GPIO_OUT_W1TS_REG, 1u << pin);
    else
      REG_WRITE(GPIO_OUT_W1TC_REG, 1u << pin);
  } else {
    gpio_set_level((gpio_num_t)pin, (int)level);
  }
}
#endif

// Limita um float ao intervalo [0, 1] para perfis de rampa.
static inline float _clampf01(float v) {
  if (v < 0.0f)
    return 0.0f;
  if (v > 1.0f)
    return 1.0f;
  return v;
}

// Curva suave tipo ease-in-out entre 0 e 1 (aproximação polinomial simples).
static float _sigmoid01(float x01) {
  // Aproximação simples (ease-in-out): x^2 * (3 - 2x)
  x01 = _clampf01(x01);
  return x01 * x01 * (3.0f - 2.0f * x01);
}

// Construtor: apenas associa pinos; pinMode e níveis iniciais ficam para begin().
Motor::Motor(uint8_t stepPin, uint8_t dirPin, uint8_t enPin)
    : _stepPin(stepPin), _dirPin(dirPin), _enPin(enPin) {}

// Define a velocidade de cruzeiro em passos por segundo.
void Motor::setSpeedStepsPerSec(float stepsPerSec) { _speedStepsPerSec = stepsPerSec; }

void Motor::setRampStartSpeedStepsPerSec(float vminStepsPerSec) {
  if (vminStepsPerSec < 0.0f)
    vminStepsPerSec = 0.0f;
  _rampStartSpeedStepsPerSec = vminStepsPerSec;
}

// Define a duração alvo da rampa de aceleração em milissegundos.
void Motor::setAccelerationTimeMs(uint32_t ms) { _accelMs = ms; }

// Define a duração alvo da rampa de desaceleração em milissegundos.
void Motor::setDecelerationTimeMs(uint32_t ms) { _decelMs = ms; }

// Escolhe linear, sigmoidal ou rampa com função customizada.
void Motor::setAccelerationType(AccelType type) { _accelType = type; }

// Associa função externa usada quando o tipo de aceleração é Custom.
void Motor::setCustomAccelFn(CustomAccelerationFn fn) { _customFn = fn; }

// Configura pinos como saída, STEP em LOW e motor desativado por defeito.
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

// Ativa o driver (ENABLE) e regista estado ligado.
void Motor::enable() { _applyEnable(true); }

// Desativa o driver e baixa o STEP.
void Motor::disable() { _applyEnable(false); }

// enable(true/false) como atalho único.
void Motor::enable(bool on) {
  if (on)
    enable();
  else
    disable();
}

// Inverte o mapeamento lógico do pino DIR e reaplica o sentido atual.
void Motor::invertDir(bool invert) {
  _dirInverted = invert;
  setDir(_dirForward);
}

// Inverte polaridade do ENABLE e reaplica o estado atual ligado/desligado.
void Motor::invertEnable(bool invert) {
  _enInverted = invert;
  _applyEnable(_enabled);
}

// Memoriza sentido “frente” e escreve DIR com inversão opcional.
void Motor::setDir(bool forward) {
  _dirForward = forward;
  _applyDir(forward);
}

// Escreve ENABLE (ativo em LOW por defeito) e ao desligar força STEP a 0.
void Motor::_applyEnable(bool on) {
  if (!on && _stepPin != 255) {
#if defined(ESP32)
    fk_gpio_set_fast(_stepPin, 0);
#else
    digitalWrite(_stepPin, LOW);
#endif
  }
  _enabled = on;
  if (_enPin == 255)
    return;

  // Padrão: enable ativo em LOW (muito comum em drivers de passo)
  // _enInverted permite inverter caso o hardware seja ativo em HIGH.
  const uint8_t level = (on ? LOW : HIGH);
  digitalWrite(_enPin, _enInverted ? !level : level);
}

// Escreve HIGH/LOW em DIR conforme frente/trás e flag de inversão.
void Motor::_applyDir(bool forward) {
  if (_dirPin == 255)
    return;

  const uint8_t level = forward ? HIGH : LOW;
  digitalWrite(_dirPin, _dirInverted ? !level : level);
}

// Soma frações de intervalo (Q8) e produz _currentInterval em microssegundos inteiros.
void Motor::_bresenhamEmitIntervalQ8(uint32_t intervalQ8) {
  _intervalAccumQ8 += intervalQ8;
  uint32_t whole = _intervalAccumQ8 >> 8;
  if (whole < 1)
    whole = 1;
  _currentInterval = whole;
  _intervalAccumQ8 -= (whole << 8);
}

// Indica se o estado da máquina não é IDLE (há ou haverá passos).
bool Motor::isRunning() const { return _state != State::IDLE; }

// Lê contador de posição em passos.
int32_t Motor::currentPosition() const { return _currentPos; }

// Lê objetivo do movimento relativo atual.
int32_t Motor::targetPosition() const { return _targetPos; }

// Arranca movimento: calcula rampa uma vez, define alvo e estado ACCEL ou CRUISE.
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

  // Passos na rampa: alinhar duração ao perfil real do planner (período linear em µs
  // entre intervalMin/Max), não à área 0.5*v*T do triângulo v–t (isso alongava ~2,75×).
  float speedMin = _rampStartSpeedStepsPerSec;
  if (speedMin <= 0.0f) {
    speedMin = (speedMax >= 10.0f) ? (speedMax / 10.0f) : 1.0f; // default compatível
  }
  if (speedMin < 1.0f)
    speedMin = 1.0f;
  if (speedMin > speedMax)
    speedMin = speedMax;
  const float intervalMinF = 1000000.0f / speedMax;
  const float intervalMaxF = 1000000.0f / speedMin;
  const float rampPeriodAvgUs = 0.5f * (intervalMinF + intervalMaxF);

  uint32_t accelSteps = 0;
  uint32_t decelSteps = 0;
  if (rampPeriodAvgUs > 0.0f) {
    accelSteps = (uint32_t)((accelS * 1000000.0f) / rampPeriodAvgUs + 0.5f);
    decelSteps = (uint32_t)((decelS * 1000000.0f) / rampPeriodAvgUs + 0.5f);
  }

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
  _intervalAccumQ8 = 0;
  _bresenhamSyncStepsDone = 0;

#if defined(MOTUS_PROFILE_INSTRUMENT)
  profileClear();
#endif
}

#if defined(MOTUS_PROFILE_INSTRUMENT)
void Motor::profileClear() { _profile_n = 0; }

bool Motor::profileEventAt(uint32_t index, ProfileEvent *out) const {
  if (!out || index >= _profile_n || index >= kProfileRingCap)
    return false;
  *out = _profile_ring[index];
  return true;
}

void Motor::_profilePush(State from, State to, uint32_t stepsDone, uint32_t stepsRem,
                         uint32_t intervalAppliedUs) {
  if (_profile_n >= kProfileRingCap)
    return;
  ProfileEvent &e = _profile_ring[_profile_n++];
  e.t_us = micros();
  e.from_state = (uint8_t)from;
  e.to_state = (uint8_t)to;
  e.steps_done = stepsDone;
  e.steps_remaining = stepsRem;
  e.interval_applied_us = intervalAppliedUs;
}
#endif

// Paragem imediata: limpa movimento e sincroniza alvo com posição atual.
void Motor::stop() {
  if (_stepPin != 255) {
#if defined(ESP32)
    fk_gpio_set_fast(_stepPin, 0);
#else
    digitalWrite(_stepPin, LOW);
#endif
  }
  _stepsRemaining = 0;
  _moveTotalSteps = 0;
  _stepsDone = 0;
  _targetPos = _currentPos;
  _currentInterval = 0;
  _plannerReady = false;
  _state = State::IDLE;
  _intervalAccumQ8 = 0;
  _bresenhamSyncStepsDone = 0;
}

// Se estiver em movimento, passa só a fase DECEL até parar naturalmente.
void Motor::stopSmooth() {
  if (_state == State::IDLE)
    return;
  _state = State::DECEL;
}

// Devolve o último intervalo µs entre passos calculado pelo planner.
uint32_t Motor::nextStepInterval() const { return _currentInterval; }

// Usa micros() para saber se já chegou a hora do próximo passo.
bool Motor::isStepDue() const { return isStepDue(micros()); }

// Compara nowUs com _nextStepTime com aritmética segura em caso de overflow do timer.
bool FK_MOTOR_IRAM Motor::isStepDue(uint32_t nowUs) const {
  if (_state == State::IDLE)
    return false;
  if (_currentInterval == 0)
    return false;

  // Comparação segura com overflow: (int32_t)(now - target) >= 0
  return (int32_t)(nowUs - _nextStepTime) >= 0;
}

// Pulso STEP com digitalWrite e delayMicroseconds (não usar dentro de ISR).
void Motor::pulseStep() {
  if (_stepPin == 255)
    return;
  digitalWrite(_stepPin, HIGH);
  if (_stepPulseUs > 0)
    delayMicroseconds(_stepPulseUs);
  digitalWrite(_stepPin, LOW);
}

// Pulso STEP rápido para ISR (GPIO direto no ESP32, espera mínima do pulso).
void FK_MOTOR_IRAM Motor::pulseStepIsr() {
  if (_stepPin == 255)
    return;

#if defined(ESP32)
  fk_gpio_set_fast(_stepPin, 1);
  if (_stepPulseUs > 0)
    ets_delay_us(_stepPulseUs);
  fk_gpio_set_fast(_stepPin, 0);
#else
  digitalWrite(_stepPin, HIGH);
  if (_stepPulseUs > 0) {
    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < _stepPulseUs) {
    }
  }
  digitalWrite(_stepPin, LOW);
#endif
}

// Após cada passo: atualiza posição, transições ACCEL/CRUISE/DECEL e agenda próximo instante.
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

#if defined(MOTUS_PROFILE_INSTRUMENT)
  const uint32_t intervalAppliedUs = _currentInterval;
  const State stateBeforeTransitions = _state;
#endif

  // Transições de estado baseadas em passos
  if (_state == State::ACCEL && _accelSteps > 0 && _stepsDone >= _accelSteps) {
    _state = State::CRUISE;
    _intervalAccumQ8 = 0;
  }

  if (_decelSteps > 0 && (uint32_t)_stepsRemaining <= _decelSteps) {
    _state = State::DECEL;
    _intervalAccumQ8 = 0;
  }

#if defined(MOTUS_PROFILE_INSTRUMENT)
  if (_state != stateBeforeTransitions) {
    _profilePush(stateBeforeTransitions, _state, _stepsDone, (uint32_t)_stepsRemaining,
                 intervalAppliedUs);
  }
#endif

  // Agenda o próximo passo usando o intervalo já calculado pelo Planner
  // (se ainda não calculado, o loop cooperativo/Planner vai preencher antes do próximo tick)
  if (_currentInterval == 0)
    _currentInterval = 1000000UL; // fallback bem lento para não travar
  _nextStepTime += _currentInterval;
}

// Fora da ISR: lê progresso com noInterrupts, calcula fator da rampa e atualiza intervalos Bresenham.
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
  float speedMin = _rampStartSpeedStepsPerSec;
  if (speedMin <= 0.0f) {
    speedMin = (speedMax >= 10.0f) ? (speedMax / 10.0f) : 1.0f; // default compatível
  }
  if (speedMin < 1.0f)
    speedMin = 1.0f;
  if (speedMin > speedMax)
    speedMin = speedMax;
  const float intervalMinF = 1000000.0f / speedMax;
  const float intervalMaxF = 1000000.0f / speedMin;

  // Dado um índice de passo (stepsDoneSim), calcula o intervalo Q8 correspondente
  // ao perfil atual (inclui transições ACCEL/CRUISE/DECEL por limiar de passos).
  const uint32_t total = _moveTotalSteps;
  const uint32_t accelN = _accelSteps;
  const uint32_t decelN = _decelSteps;
  const uint32_t decelStartStep = (decelN > 0 && total > decelN) ? (total - decelN) : total;

  auto shapedFactor01 = [&](float x01) -> float {
    x01 = _clampf01(x01);
    if (_accelType == AccelType::Sigmoidal)
      return _sigmoid01(x01);
    if (_accelType == AccelType::Custom)
      return _customFn ? _clampf01(_customFn(x01)) : x01;
    return x01; // Linear
  };

  auto intervalQ8ForStepsDone = [&](uint32_t stepsDoneSim, State *outState) -> uint32_t {
    State s = State::CRUISE;
    float factor = 1.0f;

    if (accelN > 0 && stepsDoneSim < accelN) {
      s = State::ACCEL;
      const float progress = (float)stepsDoneSim / (float)accelN; // 0..(N-1)/N
      factor = shapedFactor01(progress);
    } else if (decelN > 0 && stepsDoneSim >= decelStartStep) {
      s = State::DECEL;
      const uint32_t doneInDecel = stepsDoneSim - decelStartStep;
      const float progress = (float)doneInDecel / (float)decelN;
      factor = 1.0f - shapedFactor01(progress);
    } else {
      s = State::CRUISE;
      factor = 1.0f;
    }

    if (outState)
      *outState = s;

    const float intervalF =
        intervalMaxF - (intervalMaxF - intervalMinF) * _clampf01(factor);
    uint32_t q8 = (uint32_t)(intervalF * 256.0f + 0.5f);
    if (q8 < 256u)
      q8 = 256u;
    return q8;
  };

  // Bresenham deve ser "consumido" no ritmo dos PASSOS, não no ritmo do loop().
  // `updatePlanner()` pode rodar centenas de vezes entre dois steps, então
  // sincronizamos com `stepsDone` (lido atomicamente acima).

  if (!_plannerReady) {
    // Primeira inicialização do movimento: emite 1 intervalo e agenda o primeiro step.
    _bresenhamSyncStepsDone = stepsDone;
    _bresenhamEmitIntervalQ8(intervalQ8ForStepsDone(stepsDone, nullptr));
    _nextStepTime = micros() + _currentInterval;
    _plannerReady = true;
    return;
  }

  // Emite exatamente 1 intervalo por passo efetivamente aplicado.
  // IMPORTANTE: se vários steps ocorrerem entre duas chamadas ao planner, não podemos
  // reutilizar o mesmo `intervalQ8` para todos — pode haver transição de fase no meio.
  while (_bresenhamSyncStepsDone < stepsDone) {
    _bresenhamSyncStepsDone++;
    _bresenhamEmitIntervalQ8(intervalQ8ForStepsDone(_bresenhamSyncStepsDone, nullptr));
  }

  // Antes do 1º passo físico, a rampa pode mudar entre chamadas do planner.
  // Atualiza _currentInterval seguindo intervalF sem acumular várias vezes.
  if (stepsDone == 0) {
    uint32_t intervalQ8 = intervalQ8ForStepsDone(0, nullptr);
    uint32_t iv = (intervalQ8 + 128u) >> 8;
    if (iv < 1)
      iv = 1;
    _currentInterval = iv;
  }
}

// No modo cooperativo, só corre o planeamento (o loop principal pode chamar isto em ciclo).
void Motor::run() { updatePlanner(); }

