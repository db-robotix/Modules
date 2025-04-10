// (C) db robotix 2025

#include "motorSync.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

//MotorSync robot = MotorSync(26, 18, 19, 23, 5);  // pins Esp32 D1 Mini
MotorSync robot = MotorSync(5, 4, 3, 2, 1);  // pins Esp32 S3 Mini
TaskHandle_t Task0;
TaskHandle_t Task1;

Adafruit_NeoPixel rgb(1, 21, NEO_RGB + NEO_KHZ800);  // ESP32 S3 Mini Waveshare onboard RGB LED

volatile uint8_t command = 0;
volatile int16_t value = 0;
enum motorCommand { NONE, GO, STOP, SPEED, STEERING, ACCEL, DECEL, TARGET, COAST, BRAKE };  // do not change !

#define I2C_ADDRESS 4

void core1(void* pvParameters) {  // motor control
  for (;;) robot.go();  // infinite loop
}

void setup() {
//  Serial.begin(115200);
  rgb.begin(); // initialize NeoPixel object
  rgb.setPixelColor(0, rgb.Color(0, 0, 32));  // blue
  rgb.show();   // send the updated pixel colors to the hardware
  xTaskCreatePinnedToCore(core0, "Task0", 10000, NULL,  1, &Task0, 0);  // task on core 0
  xTaskCreatePinnedToCore(core1, "Task1", 10000, NULL, 10, &Task1, 1);  // task on core 1
  Wire.begin(I2C_ADDRESS);  // join i2c bus with address
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  delay(500);
  rgb.setPixelColor(0, rgb.Color(0, 2, 0));  // green
  rgb.show();   // send the updated pixel colors to the hardware
}

void requestEvent() {
  int16_t value;
  if (!robot.isRunning) value = -1;
  else value = max(robot.sL_real, robot.sR_real);
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
    case SPEED:    robot.setSpeed(value);    break;
    case STEERING: robot.setSteering(value); break;
    case ACCEL:    robot.setAccel(value);    break;
    case DECEL:    robot.setDecel(value);    break;
    case TARGET:   robot.setTarget(value);   break;
    case GO:       robot.isIdle = false;     break;
    case STOP:     robot.stop();             break;
    case COAST:    robot.coast();            break;
    case BRAKE:    robot.brake();            break;
//    default: Serial.println("unknown ");     break;
  }
  //Serial.print(command);
  //Serial.print(" : ");
  //Serial.println(value);
  delay(1);
  rgb.setPixelColor(0, rgb.Color(0, 2, 0));  // green
  rgb.show();   // send the updated pixel colors to the hardware
  // Serial.println(xPortGetCoreID());}  // = 0
}

void loop() { }

void core0(void* pvParameters) {  // I2C communication
  for (;;) {         // infinite loop
    delay(1000);
  }
}