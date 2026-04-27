#pragma once

#include <Arduino.h>

#include "drivers/GenericStepDir.h"
#include "drivers/SoftwareMS.h"

/** Gerenciador da biblioteca Motus Drivers (motores de passo). */
class MotusDrivers {
public:
  static constexpr uint8_t MAX_MOTORS = 8;

  // Cria e regista um motor com driver genérico STEP/DIR (TB6600) nos pinos indicados.
  GenericStepDir &setupTB6600(uint8_t step, uint8_t dir, uint8_t en);
  // Igual ao TB6600: motor STEP/DIR para DM556 (mesma classe genérica).
  GenericStepDir &setupDM556(uint8_t step, uint8_t dir, uint8_t en);

  // Cria motor com micropassos por software para A4988 (pinos MS opcionais).
  SoftwareMS &setupA4988(uint8_t step, uint8_t dir, uint8_t en,
                         uint8_t ms1 = 255, uint8_t ms2 = 255,
                         uint8_t ms3 = 255);
  // Cria motor com micropassos por software para DRV8825 (MODE0–2 opcionais).
  SoftwareMS &setupDRV8825(uint8_t step, uint8_t dir, uint8_t en,
                            uint8_t ms0 = 255, uint8_t ms1 = 255,
                            uint8_t ms2 = 255);

  /** true: `run()` só atualiza o planner; pulsos vêm de `tick()` (ISR/Task). */
  // Ativa ou desativa modo em que os passos são gerados só por tick() (ISR), não por run().
  void setIsrMode(bool isrMode) { _isrMode = isrMode; }
  // Indica se está em modo ISR (planner no loop, pulsos no tick).
  bool isrMode() const { return _isrMode; }

  // Chama begin() em todos os motores registados (configura pinos).
  void beginAll();

  // Atualiza o planeamento de todos; em modo cooperativo também envia pulsos STEP quando a hora chega.
  void run();
  // Um “passo” do maestro: em modo ISR atualiza o planner de todos os motores antes de
  // testar isStepDue(), para evitar dois passos sem updatePlanner() (perfil com rampa).
  bool tick();

  // Número de motores atualmente registados.
  uint8_t motorCount() const { return _motorCount; }

private:
  alignas(GenericStepDir) uint8_t _genericRaw[MAX_MOTORS][sizeof(GenericStepDir)];
  alignas(SoftwareMS) uint8_t _softRaw[MAX_MOTORS][sizeof(SoftwareMS)];

  uint8_t _genericCount = 0;
  uint8_t _softCount = 0;

  Motor *_motors[MAX_MOTORS] = {};
  uint8_t _motorCount = 0;

  bool _isrMode = false;

  // Fallback interno quando há demasiados motores genéricos: devolve o primeiro bloco (evita nullptr).
  GenericStepDir &_genericOverflow();
  // Idem para motores com micropassos por software.
  SoftwareMS &_softOverflow();
};

#include "MotusTickTimer.h"
