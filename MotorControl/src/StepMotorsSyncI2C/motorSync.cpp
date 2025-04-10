// (C) db robotix 2025

#include "motorSync.h"

steppingMotor motorL;
steppingMotor motorR;

MotorSync::MotorSync(const uint8_t pinL_step, const uint8_t pinL_dir, const uint8_t pinR_step, const uint8_t pinR_dir, const uint8_t pin_en) {
  // initialisation
  motorL.PIN_STEP = pinL_step;
  motorL.PIN_DIR = pinL_dir;
  motorR.PIN_STEP = pinR_step;
  motorR.PIN_DIR = pinR_dir;
  PIN_EN = pin_en;
  pinMode(motorL.PIN_STEP, OUTPUT);
  digitalWrite(motorL.PIN_STEP, LOW);
  pinMode(motorR.PIN_STEP, OUTPUT);
  digitalWrite(motorR.PIN_STEP, LOW);
  pinMode(motorL.PIN_DIR, OUTPUT);
  pinMode(motorR.PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);  // disabled
  motorL.v0 = 0;
  motorR.v0 = 0;
  stop();
}

void MotorSync::sendStep(const uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(4);  // pulse width
  digitalWrite(pin, LOW);
}

void MotorSync::setDirection() {
  digitalWrite(motorL.PIN_DIR, motorL.direction);
  digitalWrite(motorR.PIN_DIR, !motorR.direction);
}

void MotorSync::setAccel(const uint32_t _accel) {  // in steps/s^2
  accel =  constrain(_accel, 1, 10000);
}

void MotorSync::setDecel(const uint32_t _decel) {  // in steps/s^2
  decel =  constrain(_decel, 1, 10000);
}

void MotorSync::setTarget(const int32_t _s_target) {
  s_target = _s_target;
}

void MotorSync::setSpeed(const int32_t _v_given) {
  v_given =  _v_given;
}

void MotorSync::setSteering(const int32_t _steering) {
  steering = constrain(_steering, -100, 100);
}

void MotorSync::go() {
  unsigned long t0, t1, t2;  // microseconds
  int32_t td;  // loop time in microseconds
  int32_t count;
  int32_t sL_target, sR_target; // steps
  int32_t sL_given,  sR_given, s_act;  // steps
  uint64_t sL_given_long, sR_given_long;
  int32_t s_decel;  // steps to brake
  int32_t v_act;
  int32_t vL_given,  vR_given;  // steps per second
  
  while (isIdle) delay(1);  // wait until activated by main program

  bool br_phase = false;  // is not in braking phase
  sL_real = 0;
  sR_real = 0;
  v_act = 0;
  count = 0;

  motorL.direction = (steering > -50);
  motorR.direction = (steering <  50);
  if (s_target * v_given < 0) {
    motorL.direction = !motorL.direction;
    motorR.direction = !motorR.direction;
  }
  s_target = abs(s_target);
  v_given = abs(v_given);

  float v_max = sqrt(2.0 * s_target * accel * decel / (accel + decel));
  if (v_given > v_max) v_given = (int32_t)v_max;

  if (steering < 0) {  // turn to left
    sR_target = s_target;
    sL_target = abs(s_target + s_target * steering / 50) ;
  }
  else {               // turn to right
    sL_target = s_target;
    sR_target = abs(s_target - s_target * steering / 50) ;
  }
  //debugMonitor(sL_target, sR_target);
  //debugMonitor(accel, decel);

  sL_given_long = 0UL;
  sR_given_long = 0UL;
  setDirection();
  digitalWrite(PIN_EN, LOW);  // enable motors
  isRunning = true;
  t0 = micros();
  t1 = 0;
  
  // start of loop
  while (isRunning && ((sL_real < sL_target) || (sR_real < sR_target))) {
    delayMicroseconds(100);  // additional loop time
    t2 = micros() - t0;
    td = (int32_t)(t2 - t1);
    //calculation of velocities:
    s_decel = 1 + v_act * v_act / decel / 2;  // estimated steps to stop
    if (sL_real > sR_real) s_act = sL_real; else s_act = sR_real;
    br_phase = br_phase || ((s_target - s_act) < s_decel);
    if (br_phase) {  // braking phase
      v_act = (int32_t) sqrt(2.0 * decel * (s_target - s_act));
      if (v_act < VSTOP) v_act = VSTOP;
    }
    else if ((t2 / 1000) < (1050 * v_given / accel)) v_act = t2 / 10 * accel / 100000;  // acceleration phase (avoid overflow)
    else v_act = v_given;  // linear phase
    if (v_act > v_given) v_act = v_given;

    if (steering < 0) {  // turn to left
      vR_given = v_act;
      vL_given = v_act + v_act * steering / 50;
    }
    else {  // turn to right
      vL_given = v_act;
      vR_given = v_act - v_act * steering / 50;
    }
    sL_given_long += abs(vL_given) * td;
    sR_given_long += abs(vR_given) * td;
    sL_given = (int32_t)(sL_given_long / 1000000);
    sR_given = (int32_t)(sR_given_long / 1000000);
    // debugMonitor(t2, sL_given, sR_given);  
    // debugMonitor(sL_real, sR_real);

    if (sL_real < sL_given) {
      sendStep(motorL.PIN_STEP);
      sL_real++;
    }
    if (sR_real < sR_given) {
      sendStep(motorR.PIN_STEP);
      sR_real++;
    }
    t1 = t2;
    count++;
  }
  stop();
  // debugMonitor(sL_real, sR_real);
  // char serialBuffer[128];
  // sprintf(serialBuffer, "Time: %lu us  Steps: %d of %d  %d of %d\n", t1, (int)sL_real, (int)sL_given, (int)sR_real, (int)sR_given); Serial.print(serialBuffer);  
}

void MotorSync::stop() {
  isRunning = false;
  isIdle = true;
  motorL.v0 = 0; motorR.v0 = 0;
}

void MotorSync::coast() {
  stop();
  digitalWrite(PIN_EN, HIGH);  // motors disabled
}

void MotorSync::brake() {
  stop();
  digitalWrite(PIN_EN, LOW);  // motors enabled
}

// void MotorSync::debugMonitor(int32_t value1, int32_t value2 = 0, int32_t value3 = 0, int32_t value4 = 0) {
//   Serial.print("Debug ");
//   Serial.print(value1);
//   Serial.print(" : ");
//   Serial.print(value2);
//   Serial.print(" : ");
//   Serial.print(value3);
//   Serial.print(" : ");
//   Serial.println(value4);
// }

// void MotorSync::debugMonitor(int32_t value1, int32_t value2 = 0, int32_t value3 = 0) {
//   Serial.print("Debug ");
//   Serial.print(value1);
//   Serial.print(" : ");
//   Serial.print(value2);
//   Serial.print(" : ");
//   Serial.println(value3);
// }

// void MotorSync::debugMonitor(int32_t value1, int32_t value2) {
//   Serial.print("Debug ");
//   Serial.print(value1);
//   Serial.print(" : ");
//   Serial.println(value2);
// }

// void MotorSync::debugMonitor(int32_t value1) {
//   Serial.print("Debug ");
//   Serial.println(value1);
// }
