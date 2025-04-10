#include "stepMotor.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

MotorX motorA = MotorX(5, 4, 1);
MotorX motorB = MotorX(3, 2, 1);

// MotorSync robot = MotorSync(5, 4, 3, 2, 1);  // pins Esp32 S3 Mini
TaskHandle_t Task0;
TaskHandle_t Task1;

Adafruit_NeoPixel rgb(1, 21, NEO_RGB + NEO_KHZ800);  // ESP32 S3 Mini Waveshare onboard RGB LED

volatile uint8_t command = 0;
volatile int16_t value = 0;
enum motorDCommand { NONE_X, GO_A, STOP_A, SPEED_A, ACCEL_A, DECEL_A, TARGET_A, COAST_A, BRAKE_A, GO_B, STOP_B, SPEED_B, ACCEL_B, DECEL_B, TARGET_B, COAST_B, BRAKE_B };  // do not change !

#define I2C_ADDRESS 5

void core1(void* pvParameters) {  // motor control
  for (;;) {  // infinite loop
    while(!motorA.isRunning && !motorB.isRunning) delay(1); // wait until any motor is activated by main program
    // while(motorA.isRunning || motorB.isRunning) {
      if(motorA.isRunning) motorA.goloop();
      if(motorB.isRunning) motorB.goloop();
    // }
  }
}

void setup() {
  Serial.begin(115200);
  rgb.begin(); // initialize NeoPixel object
  rgb.setPixelColor(0, rgb.Color(0, 0, 32));  // blue
  rgb.show();   // send the updated pixel colors to the hardware
  xTaskCreatePinnedToCore(core0, "Task0", 10000, NULL,  1, &Task0, 0);  // task on core 0
  xTaskCreatePinnedToCore(core1, "Task1", 10000, NULL, 10, &Task1, 1);  // task on core 1
  Wire.begin(I2C_ADDRESS);  // join i2c bus with address
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  delay(1000);
  rgb.setPixelColor(0, rgb.Color(0, 4, 0));  // green
  rgb.show();   // send the updated pixel colors to the hardware
}

void requestEvent() {
  int16_t value = 0;
  if (motorA.isRunning) value += 1;
  if (motorB.isRunning) value += 2;
  Wire.write(lowByte(value)); // respond with message as expected by master
  Wire.write(highByte(value)); // 2nd byte
  Wire.endTransmission();
}

void receiveEvent(int howMany) {
  uint8_t lo, hi;
  rgb.setPixelColor(0, rgb.Color(255, 0, 0));  // red
  rgb.show();   // send the updated pixel colors to the hardware
  command = Wire.read();
  lo = Wire.read();
  hi = Wire.read();
  value = ((0x0000 | hi) << 8) | (0x0000 | lo);
  switch (command) {
    case SPEED_A:    motorA.setSpeed(value);    break;
    case ACCEL_A:    motorA.setAccel(value);    break;
    case DECEL_A:    motorA.setDecel(value);    break;
    case TARGET_A:   motorA.setTarget(value);   break;
    case GO_A:       motorA.start();            break;
    case STOP_A:     motorA.stop();             break;
    case COAST_A:    motorA.coast();            break;
    case BRAKE_A:    motorA.brake();            break;
    case SPEED_B:    motorB.setSpeed(value);    break;
    case ACCEL_B:    motorB.setAccel(value);    break;
    case DECEL_B:    motorB.setDecel(value);    break;
    case TARGET_B:   motorB.setTarget(value);   break;
    case GO_B:       motorB.start();            break;
    case STOP_B:     motorB.stop();             break;
    case COAST_B:    motorB.coast();            break;
    case BRAKE_B:    motorB.brake();            break;
    default: Serial.println("unknown ");     break;
  }
  delay(1);
  rgb.setPixelColor(0, rgb.Color(0, 4, 0));  // green
  rgb.show();   // send the updated pixel colors to the hardware
// Serial.println(xPortGetCoreID());}  // = 0
}

void loop() { /* empty */ }

void core0(void* pvParameters) {  // I2C communication
  for (;;) {         // infinite loop
    delay(1000);
  }
}