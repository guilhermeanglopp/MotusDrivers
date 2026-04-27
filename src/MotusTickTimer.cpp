#include "MotusTickTimer.h"

#include "MotusDrivers.h"

#if defined(ESP32)
#include "driver/gpio.h"
#endif

static MotusDrivers *s_drivers = nullptr;
static MotusTickTimerConfig s_cfg;
static bool s_active = false;

#if defined(ESP32)
static hw_timer_t *s_hwTimer = nullptr;

// Rotina de interrupção: opcionalmente toggla pino de prova e chama drivers->tick().
static void IRAM_ATTR motus_tick_isr() {
  if (s_cfg.probe_pin != 255)
    gpio_set_level((gpio_num_t)s_cfg.probe_pin, 1);
  if (s_drivers)
    s_drivers->tick();
  if (s_cfg.probe_pin != 255)
    gpio_set_level((gpio_num_t)s_cfg.probe_pin, 0);
}
#endif

// Configura pino de sonda, cria hw_timer no ESP32 e liga alarme periódico; noutras plataformas devolve false.
bool motusTickTimerBegin(MotusDrivers &drivers, const MotusTickTimerConfig &cfg) {
  motusTickTimerEnd();

  if (cfg.period_us == 0)
    return false;

#if defined(ESP32)
  s_drivers = &drivers;
  s_cfg = cfg;
  s_active = false;

  if (s_cfg.probe_pin != 255) {
    pinMode(s_cfg.probe_pin, OUTPUT);
    gpio_set_level((gpio_num_t)s_cfg.probe_pin, 0);
  }

  s_hwTimer = timerBegin(s_cfg.hw_timer_index, s_cfg.hw_prescaler, true);
  if (!s_hwTimer) {
    s_drivers = nullptr;
    return false;
  }

  timerAttachInterrupt(s_hwTimer, &motus_tick_isr, true);
  timerAlarmWrite(s_hwTimer, s_cfg.period_us, true);
  timerAlarmEnable(s_hwTimer);
  s_active = true;
  return true;
#else
  (void)drivers;
  (void)cfg;
  return false;
#endif
}

// Desativa alarme, remove ISR e liberta o timer; repõe ponteiros e flag ativo.
void motusTickTimerEnd() {
#if defined(ESP32)
  if (s_hwTimer) {
    timerAlarmDisable(s_hwTimer);
    timerDetachInterrupt(s_hwTimer);
    timerEnd(s_hwTimer);
    s_hwTimer = nullptr;
  }
#endif
  s_drivers = nullptr;
  s_active = false;
}

// Consulta se o maestro por timer está considerado ligado.
bool motusTickTimerIsActive() { return s_active; }
