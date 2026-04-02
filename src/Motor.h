#pragma once

#include <Arduino.h>

enum class AccelType : uint8_t { Linear, Sigmoidal, Custom };

using CustomAccelerationFn = float (*)(float progress01);

class Motor {
public:
  Motor(uint8_t stepPin, uint8_t dirPin, uint8_t enPin);

  // Configuração
  void setSpeedStepsPerSec(float stepsPerSec);
  void setAccelerationTimeMs(uint32_t ms);
  void setDecelerationTimeMs(uint32_t ms);
  void setAccelerationType(AccelType type);
  void invertDir(bool invert = true);
  void invertEnable(bool invert = true);
  void setCustomAccelFn(CustomAccelerationFn fn);

  // Enable / Disable
  void enable();
  void disable();
  void enable(bool on);

  // Direção
  void setDir(bool forward);

  // Movimento
  void moveSteps(int32_t steps);
  void stop();
  void stopSmooth();
  bool isRunning() const;
  int32_t currentPosition() const;
  int32_t targetPosition() const;

  // Interface Maestro (ISR/Task)
  bool isStepDue() const;
  bool isStepDue(uint32_t nowUs) const;
  void pulseStep();
  /** Pulso STEP sem delayMicroseconds (adequado a tick/ISR de alta frequência). */
  void pulseStepIsr();
  void stepApplied();
  uint32_t nextStepInterval() const;

  // Modo cooperativo
  void run();

  // Inicialização de hardware
  virtual void begin();

  // Interno (planejamento). Deve rodar fora de ISR.
  void updatePlanner();

protected:
  enum class State : uint8_t { IDLE, ACCEL, CRUISE, DECEL };

  uint8_t _stepPin;
  uint8_t _dirPin;
  uint8_t _enPin;

  bool _dirInverted = false;
  bool _enInverted = false;

  float _speedStepsPerSec = 0.0f;
  uint32_t _accelMs = 0;
  uint32_t _decelMs = 0;
  AccelType _accelType = AccelType::Linear;

  int32_t _currentPos = 0;
  int32_t _targetPos = 0;
  int32_t _stepsRemaining = 0;

  uint32_t _currentInterval = 0; // µs
  uint32_t _nextStepTime = 0;    // timestamp em µs (micros())
  bool _plannerReady = false;

  State _state = State::IDLE;

  CustomAccelerationFn _customFn = nullptr;

  // Planejamento do movimento (tudo em inteiros para ISR)
  uint32_t _moveTotalSteps = 0;
  uint32_t _stepsDone = 0;
  uint32_t _accelSteps = 0;
  uint32_t _decelSteps = 0;

  // Parâmetros específicos de driver (ex.: pulso mínimo em µs)
  uint32_t _stepPulseUs = 3;

  // Direção "lógica" atual
  bool _dirForward = true;
  bool _enabled = false;

  // Helpers internos
  void _applyEnable(bool on);
  void _applyDir(bool forward);
};

