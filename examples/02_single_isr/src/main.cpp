/**
 * Passo 7 — 02_single_isr
 *
 * - loop(): apenas drivers.run() → Planner (setIsrMode(true)).
 * - ESP32: hw_timer periódico → drivers.tick() (maestro; melhor que esp_timer para 10µs).
 * - Outros alvos: esp_timer periódico → drivers.tick().
 *
 * Ajuste STEP_PIN / DIR_PIN / EN_PIN. LED_BUILTIN pisca no loop (validação de loop livre).
 */

#include <Arduino.h>
#include <MotusDrivers.h>
#if !defined(ESP32)
#include <esp_timer.h>
#endif

#define LED_BUILTIN 2

static constexpr uint8_t STEP_PIN = 18;
static constexpr uint8_t DIR_PIN = 19;
static constexpr uint8_t EN_PIN = 21;

/** Período do tick em µs (10 = boa resolução; subir para 25–50 se CPU apertar). */
static constexpr uint64_t TIMER_PERIOD_US = 10;

static MotusDrivers drivers;
static Motor *motor = nullptr;

#if defined(ESP32)
static hw_timer_t *s_hwTimer = nullptr;

static void IRAM_ATTR fk_timer_isr() {
  drivers.tick();
}
#else
static esp_timer_handle_t s_fkTimer = nullptr;

static void fk_timer_cb(void *arg) {
  (void)arg;
  drivers.tick();
}
#endif

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_BUILTIN, OUTPUT);

  motor = &drivers.setupTB6600(STEP_PIN, DIR_PIN, EN_PIN);
  drivers.setIsrMode(true);
  drivers.beginAll();

  motor->setSpeedStepsPerSec(2000.0f);
  motor->setAccelerationTimeMs(300);
  motor->setDecelerationTimeMs(300);
  motor->setAccelerationType(AccelType::Linear);

  motor->enable();
  motor->moveSteps(6400);

#if defined(ESP32)
  s_hwTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(s_hwTimer, &fk_timer_isr, true);
  timerAlarmWrite(s_hwTimer, TIMER_PERIOD_US, true);
  timerAlarmEnable(s_hwTimer);
#else
  esp_timer_create_args_t cfg = {};
  cfg.callback = &fk_timer_cb;
  cfg.name = "fk_tick";

  if (esp_timer_create(&cfg, &s_fkTimer) != ESP_OK ||
      esp_timer_start_periodic(s_fkTimer, TIMER_PERIOD_US) != ESP_OK) {
    Serial.println(F("esp_timer falhou"));
    while (true) {
      delay(1000);
    }
  }
#endif
}

void loop() {
  drivers.run();

  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (now - lastMs >= 500) {
    lastMs = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  static bool reportedDone = false;
  if (!reportedDone && motor && !motor->isRunning()) {
    Serial.println(F("Movimento concluído (isRunning == false)."));
    reportedDone = true;
  }
}
