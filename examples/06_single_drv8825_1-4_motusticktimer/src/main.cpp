/**
 * Exemplo: 1 motor com DRV8825 em modo ISR via MotusTickTimer.
 * Microstepping configurado em 1/4 (4 microsteps) via SoftwareMS.
 */
 
 #include <Arduino.h>
 #include <MotusDrivers.h>
 
 #ifndef LED_BUILTIN
   #define LED_BUILTIN 2
 #endif
 
 // STEP / DIR / ENABLE do DRV8825
 static constexpr uint8_t STEP_PIN = 18;
 static constexpr uint8_t DIR_PIN  = 19;
 static constexpr uint8_t EN_PIN   = 21;
 
 // MODE0 / MODE1 / MODE2 (microstepping) do DRV8825
 static constexpr uint8_t MS0_PIN = 25;
 static constexpr uint8_t MS1_PIN = 26;
 static constexpr uint8_t MS2_PIN = 27;
 
 static MotusDrivers drivers;
 static SoftwareMS *motor = nullptr;
 
 void setup() {
   Serial.begin(115200);
   delay(200);
 
   pinMode(LED_BUILTIN, OUTPUT);
 
   motor = &drivers.setupDRV8825(STEP_PIN, DIR_PIN, EN_PIN, MS0_PIN, MS1_PIN, MS2_PIN);
   drivers.setIsrMode(true); // planeamento no loop; pulsos no tick() (timer)
   drivers.beginAll();       // chama SoftwareMS::begin() e configura pinos
 
   motor->setMicrosteps(4); // 1/4 de passo (4 microsteps)
 
   motor->setSpeedStepsPerSec(2000.0f);
   motor->setAccelerationTimeMs(300);
   motor->setDecelerationTimeMs(300);
   motor->setAccelerationType(AccelType::Linear);
 
   motor->enable();
   motor->moveSteps(6400);
 
   MotusTickTimerConfig tickCfg{};
   tickCfg.period_us = 10;
   tickCfg.probe_pin = 33; // opcional (pino de prova)
 
   if (!motusTickTimerBegin(drivers, tickCfg)) {
     Serial.println(F("motusTickTimerBegin falhou"));
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
 
   static bool saidDone = false;
   if (!saidDone && motor && !motor->isRunning()) {
     Serial.println(F("Movimento concluído."));
     saidDone = true;
     motor->invertDir();
     motor->disable();
   }
 }

