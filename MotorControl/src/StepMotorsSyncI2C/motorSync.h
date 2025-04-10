// (C) db robotix 2025

#ifndef MOTORSYNC_H
#define MOTORSYNC_H

#include <Arduino.h>

struct steppingMotor {
  uint8_t PIN_STEP;
  uint8_t PIN_DIR;
  int32_t v0;
  int32_t v_actual;
  const bool FORWARD = true;
  const bool BACKWARD = false;
  bool direction;
};

static inline int32_t sgn(int32_t val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

class MotorSync {
public:
  MotorSync(const uint8_t pinL_step, const uint8_t pinL_dir, const uint8_t pinR_step, const uint8_t pinR_dir, const uint8_t pin_en);  // constructor
private:
  volatile uint8_t PIN_EN;
  volatile const int32_t VSTOP = 30;
  volatile int32_t s_target, v_given, steering;
  volatile int32_t accel = 2000, decel = 2000;  // steps/s^2
  void sendStep(const uint8_t pin);
  void setDirection();
public:
  volatile bool isRunning = false;
  volatile bool isIdle = true;
  volatile int32_t sL_real, sR_real;  // steps sent to motors
  void setAccel(const uint32_t _accel);
  void setDecel(const uint32_t _decel);
  void setTarget(const int32_t _s_target);
  void setSpeed(const int32_t _v_given);
  void setSteering(const int32_t _steering);
  void go();
  void stop();
  void coast();
  void brake();
  void getStatus();
  // void debugMonitor(int32_t value1, int32_t value2, int32_t value3, int32_t value4);
  // void debugMonitor(int32_t value1, int32_t value2, int32_t value3);
  // void debugMonitor(int32_t value1, int32_t value2);
  // void debugMonitor(int32_t value1);
};

#endif