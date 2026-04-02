/**
 * Passo 7 — 02_single_isr
 *
 * - loop(): apenas drivers.run() → atualiza o Planner (setIsrMode(true)).
 * - esp_timer periódico: drivers.tick() → aplica pulsos STEP.
 *
 * Ajuste STEP_PIN / DIR_PIN / EN_PIN. LED_BUILTIN pisca no loop (validação de loop livre).
 */

#include <Arduino.h>
#include <MotusDrivers.h>
#include <esp_timer.h>

static constexpr uint8_t STEP_PIN = 18;
static constexpr uint8_t DIR_PIN = 19;
static constexpr uint8_t EN_PIN = 21;

/** Período do timer em µs: quanto menor, mais resolução para isStepDue (custo de CPU). */
static constexpr uint64_t TIMER_PERIOD_US = 50;

static MotusDrivers drivers;
static Motor *motor = nullptr;
static esp_timer_handle_t s_fkTimer = nullptr;

static void fk_timer_cb(void *arg) {
  (void)arg;
  drivers.tick();
}

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

  esp_timer_create_args_t cfg = {};
  cfg.callback = &fk_timer_cb;
  cfg.name = "fk_tick";

  esp_err_t err = esp_timer_create(&cfg, &s_fkTimer);
  if (err != ESP_OK) {
    Serial.println(F("esp_timer_create falhou"));
    while (true) {
      delay(1000);
    }
  }

  err = esp_timer_start_periodic(s_fkTimer, TIMER_PERIOD_US);
  if (err != ESP_OK) {
    Serial.println(F("esp_timer_start_periodic falhou"));
    while (true) {
      delay(1000);
    }
  }
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
