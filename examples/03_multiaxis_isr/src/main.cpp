/**
 * Passo 8 — 03_multiaxis_isr
 *
 * Três motores, um único esp_timer → drivers.tick() (maestro).
 * loop(): apenas drivers.run() (planner) + LED.
 *
 * X/Y proporcionais: relação passos/velocidade igual → tempo de cruzeiro parecido.
 * Z independente: perfil e passos diferentes (tende a terminar em outro instante).
 *
 * Medição de tempo de varredura (< 20 µs): use osciloscópio num pino auxiliar
 * ou temporariamente faça digitalWrite alto/baixo em volta de drivers.tick() no callback.
 */

#include <Arduino.h>
#include <MotusDrivers.h>
#include <esp_timer.h>

// --- Eixos (ajuste ao hardware) ---
static constexpr uint8_t X_STEP = 18, X_DIR = 19, X_EN = 21;
static constexpr uint8_t Y_STEP = 22, Y_DIR = 23, Y_EN = 25;
static constexpr uint8_t Z_STEP = 26, Z_DIR = 27, Z_EN = 32;

static constexpr uint64_t TIMER_PERIOD_US = 50;

static MotusDrivers drivers;
static Motor *motorX = nullptr;
static Motor *motorY = nullptr;
static Motor *motorZ = nullptr;
static esp_timer_handle_t s_fkTimer = nullptr;

static void fk_timer_cb(void *arg) {
  (void)arg;
  drivers.tick();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_BUILTIN, OUTPUT);

  motorX = &drivers.setupTB6600(X_STEP, X_DIR, X_EN);
  motorY = &drivers.setupTB6600(Y_STEP, Y_DIR, Y_EN);
  motorZ = &drivers.setupTB6600(Z_STEP, Z_DIR, Z_EN);

  drivers.setIsrMode(true);
  drivers.beginAll();

  // X/Y: mesma razão passos/velocidade → deslocamento proporcional e fim ~alinhado
  const int32_t stepsX = 3200;
  const int32_t stepsY = 1600;
  const float vX = 1600.0f;
  const float vY = 800.0f;

  motorX->setSpeedStepsPerSec(vX);
  motorY->setSpeedStepsPerSec(vY);
  motorZ->setSpeedStepsPerSec(1200.0f);

  motorX->setAccelerationTimeMs(280);
  motorY->setAccelerationTimeMs(280);
  motorZ->setAccelerationTimeMs(200);

  motorX->setDecelerationTimeMs(280);
  motorY->setDecelerationTimeMs(280);
  motorZ->setDecelerationTimeMs(200);

  motorX->setAccelerationType(AccelType::Linear);
  motorY->setAccelerationType(AccelType::Linear);
  motorZ->setAccelerationType(AccelType::Linear);

  motorX->enable();
  motorY->enable();
  motorZ->enable();

  motorX->moveSteps(stepsX);
  motorY->moveSteps(stepsY);
  // Z: menos passos — costuma parar antes (eixo “independente”)
  motorZ->moveSteps(480);

  esp_timer_create_args_t cfg = {};
  cfg.callback = &fk_timer_cb;
  cfg.name = "fk_tick_maestro";

  if (esp_timer_create(&cfg, &s_fkTimer) != ESP_OK ||
      esp_timer_start_periodic(s_fkTimer, TIMER_PERIOD_US) != ESP_OK) {
    Serial.println(F("esp_timer falhou"));
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

  static bool saidZ = false;
  static bool saidXY = false;
  if (!saidZ && motorZ && !motorZ->isRunning()) {
    Serial.println(F("Z parou (independente)."));
    saidZ = true;
  }
  if (!saidXY && motorX && motorY && !motorX->isRunning() && !motorY->isRunning()) {
    Serial.println(F("X e Y pararam (~proporcional)."));
    saidXY = true;
  }
}
