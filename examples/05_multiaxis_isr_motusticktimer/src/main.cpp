/**
 * Exemplo: um motor + maestro por timer (MotusTickTimer na biblioteca MotusDrivers).
 */

#include <Arduino.h>
#include <MotusDrivers.h>

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif
// Pinos STEP / DIR / ENABLE do driver
static constexpr uint8_t STEP_PIN = 18;
static constexpr uint8_t DIR_PIN = 19;
static constexpr uint8_t EN_PIN = 21;

static MotusDrivers drivers;
static Motor *motor = nullptr;

// Serial, motor, timer que chama tick() a cada 10 µs; passos não vêm do loop
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_BUILTIN, OUTPUT);

  motor = &drivers.setupTB6600(STEP_PIN, DIR_PIN, EN_PIN);
  drivers.setIsrMode(true); // só planeamento aqui; pulsos no timer
  drivers.beginAll();

  motor->setSpeedStepsPerSec(2000.0f);
  motor->setAccelerationTimeMs(300);
  motor->setDecelerationTimeMs(300);
  motor->setAccelerationType(AccelType::Linear);

  motor->enable();
  motor->moveSteps(6400);

  MotusTickTimerConfig tickCfg{};
  tickCfg.period_us = 10;
  tickCfg.probe_pin = 33; // sem pino de prova no osciloscópio

  if (!motusTickTimerBegin(drivers, tickCfg)) {
    Serial.println(F("motusTickTimerBegin falhou"));
    while (true) {
      delay(1000);
    }
  }
}

// Planner contínuo; LED a piscar; ao parar: mensagem, inverte DIR, desliga motor
void loop() {
  drivers.run();

  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (now - lastMs >= 500) {
    lastMs = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  static bool saidDone = false;
  if (!saidDone && motor && !motor->isRunning()) {
    Serial.println(F("Movimento concluído."));
    saidDone = true;
    motor->invertDir();
    motor->disable();
  }
}
