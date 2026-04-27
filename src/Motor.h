#pragma once

#include <Arduino.h>

enum class AccelType : uint8_t { Linear, Sigmoidal, Custom };

using CustomAccelerationFn = float (*)(float progress01);

class Motor {
public:
  // Guarda os pinos STEP/DIR/ENABLE; a inicialização real faz-se em begin().
  Motor(uint8_t stepPin, uint8_t dirPin, uint8_t enPin);

  // Configuração
  // Define a velocidade de cruzeiro em passos por segundo.
  void setSpeedStepsPerSec(float stepsPerSec);
  // Define a velocidade mínima (steps/s) usada no início/fim das rampas.
  // - vmin == 0: usa o default compatível (vpico/10 quando vpico>=10; senão 1).
  // - vmin > 0: limita o intervalo máximo (início da rampa) e altera a duração/“tempo extra” do perfil.
  void setRampStartSpeedStepsPerSec(float vminStepsPerSec);
  // Duração alvo da aceleração (ms): em moveSteps() traduz-se em passos de rampa de
  // forma a coincidir com o planner (período µs variando linearmente entre o mínimo
  // e o máximo por passo; não é a área triangular ½·v·T de aceleração clássica).
  void setAccelerationTimeMs(uint32_t ms);
  // Duração alvo da desaceleração (ms); mesma semântica que setAccelerationTimeMs.
  void setDecelerationTimeMs(uint32_t ms);
  // Escolhe a forma da rampa: linear, sigmoidal ou função personalizada.
  void setAccelerationType(AccelType type);
  // Inverte o sentido lógico do pino DIR em relação a “frente/trás”.
  void invertDir(bool invert = true);
  // Inverte a polaridade do sinal ENABLE (ativo em HIGH vs LOW).
  void invertEnable(bool invert = true);
  // Regista callback para perfil de aceleração quando o tipo é Custom.
  void setCustomAccelFn(CustomAccelerationFn fn);

  // Enable / Disable
  // Liga o motor (ENABLE conforme convenção do driver).
  void enable();
  // Desliga o motor e força STEP a LOW por segurança.
  void disable();
  // Liga ou desliga conforme o booleano.
  void enable(bool on);

  // Direção
  // Ajusta o pino DIR para o sentido lógico pedido (frente = forward).
  void setDir(bool forward);

  // Movimento
  // Inicia movimento relativo de N passos, com rampa calculada uma vez.
  void moveSteps(int32_t steps);
  // Para imediatamente: zera fila, posição alvo = atual, estado IDLE.
  void stop();
  // Força entrada em desaceleração suave até parar no fim da rampa.
  void stopSmooth();
  // true se ainda há movimento planeado (não está em IDLE).
  bool isRunning() const;
  // Posição acumulada em passos desde o último reset lógico do movimento.
  int32_t currentPosition() const;
  // Posição para onde o motor está a ir com o moveSteps atual.
  int32_t targetPosition() const;

  // Interface Maestro (ISR/Task)
  // Verifica com micros() atual se já passou a hora do próximo passo.
  bool isStepDue() const;
  // Igual ao anterior mas com instante explícito (útil em ISR para evitar múltiplas leituras).
  bool isStepDue(uint32_t nowUs) const;
  // Gera um pulso STEP com delayMicroseconds (uso fora de ISR).
  void pulseStep();
  /** Pulso STEP síncrono (HIGH → delay → LOW); no ESP32 usa `esp_rom_delay_us` + `gpio_set_level`. */
  // Versão adequada a ISR: GPIO rápido no ESP32 e espera curta do pulso.
  void pulseStepIsr();

  // Chamado após um passo físico: atualiza posição, decrementa restantes e agenda o próximo instante.
  void stepApplied();
  // Intervalo em µs até ao próximo passo (último valor calculado pelo planner).
  uint32_t nextStepInterval() const;

  // Leitura do plano (útil para testes / diagnóstico)
  uint32_t plannerAccelSteps() const { return _accelSteps; }
  uint32_t plannerDecelSteps() const { return _decelSteps; }
  uint32_t plannerMoveTotalSteps() const { return _moveTotalSteps; }
  uint32_t plannerStepsDone() const { return _stepsDone; }
  uint32_t plannerStepsRemaining() const { return (uint32_t)_stepsRemaining; }
  /** 0=IDLE, 1=ACCEL, 2=CRUISE, 3=DECEL — só para depuração. */
  uint8_t plannerStateOrdinal() const { return (uint8_t)_state; }

#if defined(MOTUS_PROFILE_INSTRUMENT)
  static constexpr uint8_t kProfileRingCap = 32;
  struct ProfileEvent {
    uint32_t t_us;
    uint8_t from_state;
    uint8_t to_state;
    uint32_t steps_done;
    uint32_t steps_remaining;
    uint32_t interval_applied_us;
  };
  void profileClear();
  uint32_t profileEventCount() const { return _profile_n; }
  bool profileEventAt(uint32_t index, ProfileEvent *out) const;
#endif

  // Modo cooperativo
  // Atalho para updatePlanner() no loop cooperativo.
  void run();

  // Inicialização de hardware
  // Configura pinos STEP/DIR/EN e estado seguro (motor desligado).
  virtual void begin();

  // Interno (planejamento). Deve rodar fora de ISR.
  // Recalcula intervalos da rampa (float) e alimenta Bresenham para o próximo passo.
  void updatePlanner();

protected:
  /** Ordinais fixos (0–3) para logs de perfil e testes. */
  enum class State : uint8_t { IDLE = 0, ACCEL = 1, CRUISE = 2, DECEL = 3 };

  uint8_t _stepPin;
  uint8_t _dirPin;
  uint8_t _enPin;

  bool _dirInverted = false;
  bool _enInverted = false;

  float _speedStepsPerSec = 0.0f;
  // Velocidade mínima das rampas (steps/s). 0 = “auto” (compatível com versões antigas).
  float _rampStartSpeedStepsPerSec = 0.0f;
  // Bresenham em Q8: acumula micros × 256 (sem float no acumulador)
  uint32_t _intervalAccumQ8 = 0;
  uint32_t _bresenhamSyncStepsDone = 0;

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
  uint32_t _stepPulseUs = 1;

  // Direção "lógica" atual
  bool _dirForward = true;
  bool _enabled = false;

  // Helpers internos
  // Escreve ENABLE e, ao desligar, garante STEP em LOW.
  void _applyEnable(bool on);
  // Escreve DIR com inversão lógica opcional.
  void _applyDir(bool forward);
  // Acumula intervalo em Q8 e extrai µs inteiros para o próximo passo (suavização Bresenham).
  void _bresenhamEmitIntervalQ8(uint32_t intervalQ8);

#if defined(MOTUS_PROFILE_INSTRUMENT)
  void _profilePush(State from, State to, uint32_t stepsDone, uint32_t stepsRem,
                    uint32_t intervalAppliedUs);
  uint32_t _profile_n = 0;
  ProfileEvent _profile_ring[kProfileRingCap] = {};
#endif
};

