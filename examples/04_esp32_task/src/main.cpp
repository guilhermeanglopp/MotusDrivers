/**
 * Passo 9 — 04_esp32_task
 *
 * ESP32 (Arduino-ESP32): o `loop()` costuma ser prioridade baixa no core 1.
 * O maestro dos passos deve ser periódico e previsível: **hw_timer** (não esp_timer
 * se precisares de 10µs estáveis) chama `drivers.tick()`.
 *
 * Arquitetura:
 * - **hw_timer** periódico → `drivers.tick()` (executor)
 * - **loop()** → `drivers.run()` (planner, float)
 * - **Task no Core 1** (prioridade baixa) → telemetria + stack watermark
 */

#include <Arduino.h>
#include <MotusDrivers.h>
#if !defined(ESP32)
#include <esp_timer.h>
#endif

#define LED_BUILTIN 2

static constexpr uint8_t X_STEP = 18, X_DIR = 19, X_EN = 21;
static constexpr uint8_t Y_STEP = 22, Y_DIR = 23, Y_EN = 25;
static constexpr uint8_t Z_STEP = 26, Z_DIR = 27, Z_EN = 32;

static constexpr uint64_t TIMER_PERIOD_US = 10;

static MotusDrivers drivers;
static Motor *mx = nullptr;
static Motor *my = nullptr;
static Motor *mz = nullptr;

#if defined(ESP32)
static hw_timer_t *s_hwTimer = nullptr;

// Executor de passos no tempo: tick() a cadência fixa.
static void IRAM_ATTR fk_timer_isr() {
  drivers.tick();
}
#else
static esp_timer_handle_t s_fkTimer = nullptr;

// Equivalente ao hw_timer em plataformas sem ESP32 clássico.
static void fk_timer_cb(void *arg) {
  (void)arg;
  drivers.tick();
}
#endif

static TaskHandle_t s_monitorTask = nullptr;

// Tarefa de baixa prioridade: telemetria periódica, LED e watermark de stack.
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

// Inicia três eixos, timer de tick e cria a task de monitorização num core fixo.
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

#if defined(ESP32)
  s_hwTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(s_hwTimer, &fk_timer_isr, true);
  timerAlarmWrite(s_hwTimer, TIMER_PERIOD_US, true);
  timerAlarmEnable(s_hwTimer);
#else
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
#endif

  xTaskCreatePinnedToCore(monitorTask, "fk_monitor", 4096, nullptr, 1,
                          &s_monitorTask, 1);
}

// Apenas atualiza o planeamento; execução física fica na ISR e telemetria na task.
void loop() {
  drivers.run();
}
