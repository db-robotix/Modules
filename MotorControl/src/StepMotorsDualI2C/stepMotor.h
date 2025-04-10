#ifndef STEPMOTOR_H
#define STEPMOTOR_H

#include <Arduino.h>

static inline int32_t sgn(int32_t val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

class MotorX {
public:
  MotorX(const uint8_t pin_step, const uint8_t pin_dir, const uint8_t pin_en);  // constructor
private:
  volatile uint8_t PIN_STEP;
  volatile uint8_t PIN_DIR;
  volatile uint8_t PIN_EN;
  const int32_t VSTOP = 30;
  const bool FORWARD = true;
  const bool BACKWARD = false;
  volatile int32_t s_target, v_given;
  volatile int32_t accel = 2000, decel = 2000;  // steps/s^2
  volatile int32_t v_actual;
  volatile bool direction;
  unsigned long t0, t1, t2;  // microseconds
  int32_t td;  // loop time in microseconds
  int32_t count;
  int32_t target; // steps
  int32_t s_given;  // steps
  uint64_t s_given_long;
  int32_t s_decel;  // steps to brake
  int32_t v_max, v_act;
  bool br_phase = false;  // is not in braking phase
  void setDirection();
  void sendStep(const uint8_t pin);
public:
  volatile bool isRunning = false;
  volatile int32_t s_real;  // steps sent to motors
  void setSpeed(const int32_t _v_given);
  void setAccel(const uint32_t _accel);
  void setDecel(const uint32_t _decel);
  void setTarget(const int32_t _s_target);
  void stop();
  void coast();
  void brake();
  void getStatus();
  void start();
  void goloop();
  void debugMonitor(int32_t value1, int32_t value2, int32_t value3, int32_t value4);
  void debugMonitor(int32_t value1, int32_t value2, int32_t value3);
  void debugMonitor(int32_t value1, int32_t value2);
  void debugMonitor(int32_t value1);
};

#endif