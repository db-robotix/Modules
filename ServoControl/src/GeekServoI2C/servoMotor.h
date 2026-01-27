#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>

class ServoX {
public:
  ServoX(const uint8_t pin_geek);  // constructor
private:
  volatile int32_t pulsewidth;
public:
  volatile bool isActive = false;
  volatile uint8_t PIN_GEEK;
  void writeMicroseconds(const uint8_t pin);
  void setPulsewidth(const int32_t _pulsewidth);
  void coast();
};

#endif