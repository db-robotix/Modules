#include "servoMotor.h"

ServoX::ServoX(const uint8_t pin_geek) {
  // initialisation
  PIN_GEEK = pin_geek;
  pinMode(PIN_GEEK, OUTPUT);
  digitalWrite(PIN_GEEK, LOW);
  isActive = false;
}

void ServoX::writeMicroseconds(const uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulsewidth);  // pulse width
  digitalWrite(pin, LOW);
}

void ServoX::setPulsewidth(const int32_t _pulsewidth) {
  pulsewidth = _pulsewidth;
  isActive = true;
}

void ServoX::coast() {
  digitalWrite(PIN_GEEK, LOW);  // servo disabled
  isActive = false;
}