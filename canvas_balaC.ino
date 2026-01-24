#include <M5StickC.h>
#include "gyro.h"

void setup() {
  M5.begin();
  M5.IMU.Init(); // per far funzionare gyro
}

void loop() {
  delay(500);
  readGyro();
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("gyroX: %7.2f", gyroX);
} 