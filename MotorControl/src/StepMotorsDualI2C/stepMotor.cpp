#include "stepMotor.h"

MotorX::MotorX(const uint8_t pin_step, const uint8_t pin_dir, const uint8_t pin_en) {
  // initialisation
  PIN_STEP = pin_step;
  PIN_DIR  = pin_dir;
  PIN_EN   = pin_en;
  pinMode(PIN_STEP, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);  // disabled
  stop();
}

void MotorX::sendStep(const uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(4);  // pulse width
  digitalWrite(pin, LOW);
}

void MotorX::setDirection() {
  digitalWrite(PIN_DIR, direction);
}

void MotorX::setAccel(const uint32_t _accel) {  // in steps/s^2
  accel =  constrain(_accel, 1, 10000);
}

void MotorX::setDecel(const uint32_t _decel) {  // in steps/s^2
  decel =  constrain(_decel, 1, 10000);
}

void MotorX::setTarget(const int32_t _s_target) {
  s_target = _s_target;
}

void MotorX::setSpeed(const int32_t _v_given) {
  v_given =  _v_given;
}

void MotorX::start() {
  s_real = 0;
  s_given_long = 0UL;
  v_act = 0;
  count = 0;
  direction = (s_target * v_given > 0);
  setDirection();
  target = abs(s_target);
  v_max = (int32_t)sqrt(2.0 * target * accel * decel / (accel + decel));
  if (v_max > abs(v_given)) v_max = abs(v_given);

  //debugMonitor(target);
  //debugMonitor(accel, decel);

  digitalWrite(PIN_EN, LOW);  // enable motors
  t0 = micros();
  t1 = 0;
  isRunning = true;
}

void MotorX::goloop() {
  if(!isRunning) return;
  delayMicroseconds(400);  // additional loop time
  t2 = micros() - t0;
  td = (int32_t)(t2 - t1);
  //calculation of velocities:
  s_decel = 1 + v_act * v_act / decel / 2;  // estimated steps to stop
  br_phase = br_phase || ((target - s_real) < s_decel);
  if (br_phase) {  // braking phase
    v_act = (int32_t) sqrt(2.0 * decel * (target - s_real));
    v_act = max(v_act, VSTOP);
  }
  else if ((t2 / 1000) < (1050 * v_max / accel)) v_act = t2 / 10 * accel / 100000;  // acceleration phase (avoid overflow!)
  else v_act = v_max;  // linear phase
  v_act = min(v_act, v_max);  // for all phases
  s_given_long += abs(v_act) * td;
  s_given = (int32_t)(s_given_long / 1000000);
  // debugMonitor(t2, v_act, s_given, s_real);
  if (s_real < s_given) {
    sendStep(PIN_STEP);
    s_real++;
  }
  t1 = t2;
  count++;
  if(s_real >= target) isRunning = false;  // finish loop
}

void MotorX::stop() {
  isRunning = false;
}

void MotorX::coast() {
  stop();
  digitalWrite(PIN_EN, HIGH);  // motors disabled
}

void MotorX::brake() {
  stop();
  digitalWrite(PIN_EN, LOW);  // motors enabled
}

void MotorX::debugMonitor(int32_t value1, int32_t value2 = 0, int32_t value3 = 0, int32_t value4 = 0) {
  Serial.print("Debug ");
  Serial.print(value1);
  Serial.print(" : ");
  Serial.print(value2);
  Serial.print(" : ");
  Serial.print(value3);
  Serial.print(" : ");
  Serial.println(value4);
}

void MotorX::debugMonitor(int32_t value1, int32_t value2 = 0, int32_t value3 = 0) {
  Serial.print("Debug ");
  Serial.print(value1);
  Serial.print(" : ");
  Serial.print(value2);
  Serial.print(" : ");
  Serial.println(value3);
}

void MotorX::debugMonitor(int32_t value1, int32_t value2) {
  Serial.print("Debug ");
  Serial.print(value1);
  Serial.print(" : ");
  Serial.println(value2);
}

void MotorX::debugMonitor(int32_t value1) {
  Serial.print("Debug ");
  Serial.println(value1);
}
