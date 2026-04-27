#pragma once

/**
 * Maestro por timer (ESP32: hw_timer; outros: esp_timer) para `MotusDrivers::tick()`.
 * Parte da biblioteca Motus Drivers — inclua após `<MotusDrivers.h>`.
 */

#include <Arduino.h>
#include <stdint.h>

class MotusDrivers;

/** Configuração do maestro periódico que chama `MotusDrivers::tick()`. */
struct MotusTickTimerConfig {
  /** Período entre chamadas a `tick()`, em microssegundos. */
  uint32_t period_us = 10;
  /**
   * GPIO para osciloscópio: fica HIGH durante `tick()`.
   * Use `255` para desativar (não configura pino).
   */
  uint8_t probe_pin = 255;
#if defined(ESP32)
  /** Índice do hw_timer (0–3 no ESP32 clássico). */
  uint8_t hw_timer_index = 0;
  /** Prescaler: com 80 e APB 80 MHz → contagem em µs. */
  uint16_t hw_prescaler = 80;
#endif
};

/**
 * Inicia o timer periódico (ESP32: hw_timer; outros: esp_timer).
 * @return true se OK, false em falha (ex.: esp_timer_create).
 */
// Arranca hardware timer que chama tick() nos drivers em cada período (maestro de passos).
bool motusTickTimerBegin(MotusDrivers &drivers, const MotusTickTimerConfig &cfg);

/** Para e liberta o timer criado por `motusTickTimerBegin`. */
// Desliga o timer e limpa estado global do maestro.
void motusTickTimerEnd();

/** Indica se o maestro está ativo. */
// true se motusTickTimerBegin teve sucesso e ainda não foi feito End.
bool motusTickTimerIsActive();
