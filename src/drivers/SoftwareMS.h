#pragma once

#include "../Motor.h"

enum class DriverVariant : uint8_t { A4988, DRV8825 };

class SoftwareMS : public Motor {
public:
  SoftwareMS(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, uint8_t ms0,
             uint8_t ms1, uint8_t ms2, DriverVariant variant);

  void begin() override;

  void setMicrosteps(uint8_t ms);
  uint8_t getMicrosteps() const;

private:
  uint8_t _msPins[3];
  uint8_t _currentMs = 1;
  DriverVariant _variant;
  bool _begun = false;

  uint8_t snapMicrosteps(uint8_t ms) const;
  void applyMicrostepPins(uint8_t ms);
  static void writeMsPin(uint8_t pin, bool high);
};
