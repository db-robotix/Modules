#include "servoMotor.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

ServoX servoA = ServoX(5);
ServoX servoB = ServoX(3);

TaskHandle_t Task0;
TaskHandle_t Task1;

Adafruit_NeoPixel rgb(1, 21, NEO_RGB + NEO_KHZ800);  // ESP32 S3 Mini Waveshare onboard RGB LED

volatile uint8_t command = 0;
volatile int16_t value = 0;
enum servoCommand { NONE_G, ANGLE_A, COAST_A, ANGLE_B, COAST_B };  // do not change !

#define I2C_ADDRESS 6

void core1(void* pvParameters) {  // servo control
  for (;;) {  // infinite loop
    while(!servoA.isActive && !servoB.isActive) delay(1); // wait until any servo is activated by main program
    if(servoA.isActive) servoA.writeMicroseconds(servoA.PIN_GEEK);
    if(servoB.isActive) servoB.writeMicroseconds(servoB.PIN_GEEK);
    delay(17);  // ca. 50 Hz
  }
}

void setup() {
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

void requestEvent() {   // not used
  int16_t value = 0;
  if (servoA.isActive) value += 1;
  if (servoB.isActive) value += 2;
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
    case ANGLE_A:  servoA.setPulsewidth(value);  break;
    case COAST_A:  servoA.coast();               break;
    case ANGLE_B:  servoB.setPulsewidth(value);  break;
    case COAST_B:  servoB.coast();               break;
    default: Serial.println("unknown ");         break;
  }
  delay(1);
  rgb.setPixelColor(0, rgb.Color(0, 4, 0));  // green
  rgb.show();   // send the updated pixel colors to the hardware
}

void loop() { /* empty */ }

void core0(void* pvParameters) {  // I2C communication
  for (;;) {         // infinite loop
    delay(1000);
  }
}