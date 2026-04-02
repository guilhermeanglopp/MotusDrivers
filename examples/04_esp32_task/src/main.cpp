/**
 * Passo 9 — 04_esp32_task
 *
 * ESP32 (Arduino-ESP32): a task do `loop()` normalmente roda no Core 1 com
 * prioridade baixa. Uma segunda task no MESMO core com prioridade 24 (como no
 * modelo antigo) pode **matar de fome** o `loop()` → `drivers.run()` nunca roda
 * → `_currentInterval` fica 0 → **nenhum pulso em STEP**.
 *
 * Arquitetura corrigida (alinhada ao ISR_vs_TASK_Comparison.md):
 * - **esp_timer** periódico → `drivers.tick()` (executor / maestro)
 * - **loop()** → `drivers.run()` (planner, float)
 * - **Task no Core 1** (baixa prioridade) → telemetria + `uxTaskGetStackHighWaterMark`
 *
 * Ajuste os pinos conforme seu hardware.
 */

#include <Arduino.h>
#include <MotusDrivers.h>
#include <esp_timer.h>

static constexpr uint8_t X_STEP = 18, X_DIR = 19, X_EN = 21;
static constexpr uint8_t Y_STEP = 22, Y_DIR = 23, Y_EN = 25;
static constexpr uint8_t Z_STEP = 26, Z_DIR = 27, Z_EN = 32;

static constexpr uint64_t TIMER_PERIOD_US = 50;

static MotusDrivers drivers;
static Motor *mx = nullptr;
static Motor *my = nullptr;
static Motor *mz = nullptr;

static esp_timer_handle_t s_fkTimer = nullptr;
static TaskHandle_t s_monitorTask = nullptr;

static void fk_timer_cb(void *arg) {
  (void)arg;
  drivers.tick();
}

static void monitorTask(void *param) {
  (void)param;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!mx || !my || !mz || !s_monitorTask)
      continue;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.printf(
        "X:%d Y:%d Z:%d run:%d%d%d | monitorStack:%u\n",
        (int)mx->currentPosition(), (int)my->currentPosition(),
        (int)mz->currentPosition(), (int)mx->isRunning(),
        (int)my->isRunning(), (int)mz->isRunning(),
        (unsigned)uxTaskGetStackHighWaterMark(s_monitorTask));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(LED_BUILTIN, OUTPUT);

  mx = &drivers.setupTB6600(X_STEP, X_DIR, X_EN);
  my = &drivers.setupTB6600(Y_STEP, Y_DIR, Y_EN);
  mz = &drivers.setupTB6600(Z_STEP, Z_DIR, Z_EN);

  drivers.setIsrMode(true);
  drivers.beginAll();

  mx->setSpeedStepsPerSec(3200.0f);
  my->setSpeedStepsPerSec(3200.0f);
  mz->setSpeedStepsPerSec(4500.0f);

  mx->setAccelerationTimeMs(250);
  my->setAccelerationTimeMs(250);
  mz->setAccelerationTimeMs(250);

  mx->setDecelerationTimeMs(250);
  my->setDecelerationTimeMs(250);
  mz->setDecelerationTimeMs(250);

  mx->setAccelerationType(AccelType::Linear);
  my->setAccelerationType(AccelType::Linear);
  mz->setAccelerationType(AccelType::Linear);

  mx->enable();
  my->enable();
  mz->enable();

  mx->moveSteps(6400);
  my->moveSteps(6400);
  mz->moveSteps(9600);

  esp_timer_create_args_t tcfg = {};
  tcfg.callback = &fk_timer_cb;
  tcfg.name = "fk_tick";
  if (esp_timer_create(&tcfg, &s_fkTimer) != ESP_OK ||
      esp_timer_start_periodic(s_fkTimer, TIMER_PERIOD_US) != ESP_OK) {
    Serial.println(F("esp_timer falhou"));
    while (true) {
      delay(1000);
    }
  }

  // Task só para monitoramento (prioridade baixa — não compete com o loop/planner)
  xTaskCreatePinnedToCore(monitorTask, "fk_monitor", 4096, nullptr, 1,
                          &s_monitorTask, 1);
}

void loop() {
  drivers.run();
}
