#include "functions.h"
#include "globals.h"
#include <M5Unified.h> 
#include <Wire.h>

void readEncoders(int32_t *encL, int32_t *encR) {
  uint8_t data[8];
  Wire.beginTransmission(0x3A);
  Wire.write(0x10); 
  Wire.endTransmission();
  
  Wire.requestFrom(0x3A, 8);
  if (Wire.available() == 8) {
    for (int i = 0; i < 8; i++) data[i] = Wire.read();
    *encL = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    *encR = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
  }
}

int32_t getAveragePosition() {
  int32_t eL = 0, eR = 0;
  readEncoders(&eL, &eR);
  return ((eL + eR) / 2) - offsetEnc;
}

void setMotors(int16_t speedL, int16_t speedR) { 
  Wire.beginTransmission(0x3A); 
  Wire.write(0x00); 
  Wire.write((uint8_t)(speedL >> 8));
  Wire.write((uint8_t)(speedL & 0xFF));
  Wire.endTransmission(); 

  Wire.beginTransmission(0x3A); 
  Wire.write(0x02); 
  Wire.write((uint8_t)(speedR >> 8));
  Wire.write((uint8_t)(speedR & 0xFF));
  Wire.endTransmission(); 
} 


void calibrateZero() { 
  setMotors(0, 0); 
  M5.Display.fillScreen(TFT_BLACK); 
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK); 
  M5.Display.setCursor(20, 30); 
  M5.Display.setTextSize(3); 
  M5.Display.println("Calibrazione"); 
  delay(1500); 

  float pitchSum = 0; 
  float gyroSum = 0; 
  int samples = 50; 
  for(int i = 0; i < samples; i++) { 
    M5.Imu.getAccel(&accX, &accY, &accZ); 
    M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ); 
    pitchSum += -atan2(accY, accZ) * 180.0 / PI; 
    gyroSum += gyroX; 
    delay(10); 
  } 
  
  targetAngle = pitchSum / (float)samples; 
  gyroXBias = gyroSum / (float)samples; 
  filteredPitch = targetAngle; 
  previousError = 0.0; 

  // Controllo posizione
  int32_t eL=0, eR=0;
  readEncoders(&eL, &eR);
  offsetEnc = (eL + eR) / 2;
  lastPosition = 0;

  M5.Display.fillScreen(TFT_BLACK); 
  M5.Display.setCursor(20, 40); 
  M5.Display.setTextSize(3); 
  M5.Display.printf("Zero: %.1f", targetAngle); 
  delay(1000); 
  
  M5.Display.fillScreen(TFT_BLACK); 
  M5.Display.setCursor(20, 60); 
  M5.Display.println("Operativo"); 
} 