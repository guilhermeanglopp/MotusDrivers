/**
 * Passo 6 — 01_single_cooperative
 *
 * Motor único, sem ISR: apenas drivers.run() no loop().
 * Ajuste STEP_PIN / DIR_PIN / EN_PIN conforme sua ligação.
 */

#include <Arduino.h>
#include <MotusDrivers.h>

// ESP32 — troque pelos pinos do seu hardware
static constexpr uint8_t STEP_PIN = 18;
static constexpr uint8_t DIR_PIN = 19;
static constexpr uint8_t EN_PIN = 21;

static MotusDrivers drivers;
static Motor *motor = nullptr;

void setup() {
  Serial.begin(115200);
  delay(200);

  motor = &drivers.setupTB6600(STEP_PIN, DIR_PIN, EN_PIN);
  drivers.setIsrMode(false);
  drivers.beginAll();

  motor->setSpeedStepsPerSec(800.0f);
  motor->setAccelerationTimeMs(400);
  motor->setDecelerationTimeMs(400);
  motor->setAccelerationType(AccelType::Linear);

  motor->enable();
  motor->moveSteps(3200);
}

void loop() {
  drivers.run();

  static bool reportedDone = false;
  if (!reportedDone && motor && !motor->isRunning()) {
    Serial.println(F("Movimento concluído (isRunning == false)."));
    reportedDone = true;
  }
}
