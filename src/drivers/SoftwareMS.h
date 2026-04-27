#pragma once

#include "../Motor.h"

enum class DriverVariant : uint8_t { A4988, DRV8825 };

class SoftwareMS : public Motor {
public:
  // Associa pinos STEP/DIR/EN e os três pinos de modo de micropassos; escolhe variante A4988 ou DRV8825.
  SoftwareMS(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, uint8_t ms0,
             uint8_t ms1, uint8_t ms2, DriverVariant variant);

  // Configura pinos MS e delega ao Motor::begin(); aplica tabela de micropassos atual.
  void begin() override;

  // Pede um valor de micropassos; ajusta à tabela suportada e atualiza pinos se já iniciado.
  void setMicrosteps(uint8_t ms);
  // Devolve o divisor de micropassos efetivo (1, 2, 4, …).
  uint8_t getMicrosteps() const;

private:
  uint8_t _msPins[3];
  uint8_t _currentMs = 1;
  DriverVariant _variant;
  bool _begun = false;

  // Arredonda o pedido para o valor válido mais próximo conforme o chip.
  uint8_t snapMicrosteps(uint8_t ms) const;
  // Escreve níveis MS0–2 conforme a tabela do A4988 ou DRV8825.
  void applyMicrostepPins(uint8_t ms);
  // Escreve um pino MS se não for 255 (desligado).
  static void writeMsPin(uint8_t pin, bool high);
};
