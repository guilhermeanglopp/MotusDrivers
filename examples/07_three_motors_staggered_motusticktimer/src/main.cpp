/**
 * Exemplo: 3 motores (STEP/DIR/EN) com DRV8825 em modo ISR via MotusTickTimer.
 * Os três começam ao mesmo tempo, mas com parâmetros/duração diferentes (terminam em tempos distintos).
 *
 * Observação de hardware:
 * - Neste exemplo os pinos MODE0/1/2 (microstepping) são "compartilhados" entre os 3 drivers
 *   (ou seja, amarrados fisicamente em paralelo) para manter o exemplo simples.
 * - Se cada driver tiver seus próprios MODE0/1/2 independentes, defina MS0/1/2 por motor.
 */
 
 #include <Arduino.h>
 #include <MotusDrivers.h>
 
 #ifndef LED_BUILTIN
   #define LED_BUILTIN 2
 #endif
 
 // MODE0 / MODE1 / MODE2 compartilhados (microstepping)
 static constexpr uint8_t MS0_PIN = 25;
 static constexpr uint8_t MS1_PIN = 26;
 static constexpr uint8_t MS2_PIN = 27;
 
 // Motor A
 static constexpr uint8_t A_STEP = 18;
 static constexpr uint8_t A_DIR  = 19;
 static constexpr uint8_t A_EN   = 21;
 
 // Motor B (ajuste para pinos livres no seu ESP32)
 static constexpr uint8_t B_STEP = 23;
 static constexpr uint8_t B_DIR  = 22;
 static constexpr uint8_t B_EN   = 32;
 
 // Motor C (ajuste para pinos livres no seu ESP32)
 static constexpr uint8_t C_STEP = 5;
 static constexpr uint8_t C_DIR  = 17;
 static constexpr uint8_t C_EN   = 16;
 
 static MotusDrivers drivers;
 static SoftwareMS *mA = nullptr;
 static SoftwareMS *mB = nullptr;
 static SoftwareMS *mC = nullptr;
 
 static void configMotor(SoftwareMS *m, float speed, uint32_t accelMs, uint32_t decelMs) {
   m->setSpeedStepsPerSec(speed);
   m->setAccelerationTimeMs(accelMs);
   m->setDecelerationTimeMs(decelMs);
   m->setAccelerationType(AccelType::Linear);
 }
 
 void setup() {
   Serial.begin(115200);
   delay(200);
   pinMode(LED_BUILTIN, OUTPUT);
 
   mA = &drivers.setupDRV8825(A_STEP, A_DIR, A_EN, MS0_PIN, MS1_PIN, MS2_PIN);
   mB = &drivers.setupDRV8825(B_STEP, B_DIR, B_EN, MS0_PIN, MS1_PIN, MS2_PIN);
   mC = &drivers.setupDRV8825(C_STEP, C_DIR, C_EN, MS0_PIN, MS1_PIN, MS2_PIN);
 
   drivers.setIsrMode(true);
   drivers.beginAll();
 
   // Microstepping (1/4). Como os MODE pins são compartilhados, basta chamar em um,
   // mas chamar nos três não tem problema (vai escrever a mesma combinação).
   mA->setMicrosteps(0);
   mB->setMicrosteps(0);
   mC->setMicrosteps(0);
 
   // Parâmetros diferentes (cada um vai "terminar" em um tempo diferente)
   configMotor(mA, 2200.0f, 250, 250);
   configMotor(mB, 1400.0f, 500, 500);
   configMotor(mC, 900.0f,  800, 800);
 
   mA->enable();
   mB->enable();
   mC->enable();
 
   // Todos começam ao mesmo tempo (chamadas sequenciais no setup, diferença é desprezível)
   mA->moveSteps(6400);  // mais rápido
   mB->moveSteps(9000);  // intermediário
   mC->moveSteps(12000); // mais lento / mais longo
 
   MotusTickTimerConfig tickCfg{};
   tickCfg.period_us = 10;
   tickCfg.probe_pin = 33;
 
   if (!motusTickTimerBegin(drivers, tickCfg)) {
     Serial.println(F("motusTickTimerBegin falhou"));
     while (true) {
       delay(1000);
     }
   }
 
   Serial.println(F("3 motores iniciados."));
 }
 
 void loop() {
   drivers.run();
 
   static uint32_t lastBlink = 0;
   const uint32_t now = millis();
   if (now - lastBlink >= 250) {
     lastBlink = now;
     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
   }
 
   static bool saidA = false, saidB = false, saidC = false;
   if (!saidA && mA && !mA->isRunning()) {
     saidA = true;
     Serial.println(F("Motor A terminou."));
     mA->disable();
   }
   if (!saidB && mB && !mB->isRunning()) {
     saidB = true;
     Serial.println(F("Motor B terminou."));
     mB->disable();
   }
   if (!saidC && mC && !mC->isRunning()) {
     saidC = true;
     Serial.println(F("Motor C terminou."));
     mC->disable();
   }
 }

