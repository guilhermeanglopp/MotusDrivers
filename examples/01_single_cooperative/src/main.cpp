/**
 * Passo 6 — 01_single_cooperative
 *
 * Motor único, sem timer: setIsrMode(false) e apenas drivers.run() no loop()
 * (planner + pulseStep com delayMicroseconds). Não usa hw_timer nem esp_timer.
 *
 * Ajuste STEP_PIN / DIR_PIN / EN_PIN conforme sua ligação.
 */

#include <Arduino.h>
#include <MotusDrivers.h>

#define LED_BUILTIN 2

// ESP32 — troque pelos pinos do seu hardware
static constexpr uint8_t STEP_PIN = 18;
static constexpr uint8_t DIR_PIN = 19;
static constexpr uint8_t EN_PIN = 21;

static MotusDrivers drivers;
static Motor *motor = nullptr;

// Inicia Serial, regista TB6600 em modo cooperativo, configura rampa e arranca movimento.
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_BUILTIN, OUTPUT);

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

// Chama run() para planner + pulsos; pisca LED e avisa no Serial quando o movimento termina.
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
