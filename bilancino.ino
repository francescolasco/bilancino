#include <M5Unified.h> 
#include <Wire.h> 
#include <math.h> 
// #include <SD.h>

float Kp = 40.0; 
float Kd = 0.1; 

float previousError = 0.0; 

float accX = 0, accY = 0, accZ = 0; 
float gyroX = 0, gyroY = 0, gyroZ = 0; 
float u = 0;

float filteredPitch = 0.0; 
float targetAngle = 0.0; 
float gyroXBias = 0.0; 

int32_t offsetEnc = 0; 
int32_t lastPosition = 0;

// Controllo posizione
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
  speedL = constrain(speedL, -1023, 1023);
  speedR = constrain(speedR, -1023, 1023);

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
  M5.Display.setCursor(20, 70); 
  M5.Display.setTextSize(2); 
  M5.Display.println("Tienilo fermo..."); 
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

void setup() { 
  auto cfg = M5.config(); 
  M5.begin(cfg); 
  Serial.begin(115200); 
  delay(500); 
  
  M5.Power.setExtOutput(true); 
  Wire.begin(21, 22, 400000UL); 
  calibrateZero(); 
  // Scrittura CSV
  /*
  if (!SD.begin(4, SPI)) {
    Serial.println("SD card initialization failed!");
    return;
  }
	csv = SD.open("/telemetria.csv", FILE_WRITE);
	if (!csv) {
    Serial.println("Failed to open example.txt for writing.");
    return;
  } else {
		csv.println("u,theta");
		csv.close();
	}
  */
} 

void loop() { 
  M5.update(); 

  // Sleep
  if (M5.BtnC.wasPressed()) { 
    /*
    setMotors(0, 0); 
    M5.Display.sleep(); 
    M5.Power.setExtOutput(false); 
    M5.Power.deepSleep(); 
    */
    Kd = Kd+0.1;
  } 
  
  // Reset
  if (M5.BtnA.wasPressed()) { 
    calibrateZero(); 
  }

  // KP
  if (M5.BtnB.wasPressed()) { 
    Kp = Kp+0.5; 
  }  

  M5.Imu.getAccel(&accX, &accY, &accZ); 
  M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ); 
  
  float trueGyroX = gyroX - gyroXBias; 
  float accPitch = -atan2(accY, accZ) * 180.0 / PI; 
  
  filteredPitch = 0.98 * (filteredPitch - trueGyroX * 0.01) + 0.02 * accPitch; 
  
  int32_t currentPos = getAveragePosition();
  float currentSpeed = (float)(currentPos - lastPosition); 
  lastPosition = currentPos;
  float Kp_enc = 0*0.005; 
  float Kd_enc = 0*0.02;    
  float angleCorrection = (currentPos * Kp_enc) + (currentSpeed * Kd_enc);
  angleCorrection = constrain(angleCorrection, -8.0, 8.0);

  float error = targetAngle -angleCorrection - filteredPitch; 
  
  float dt = 0.01; 
  float derivative = (error - previousError) / dt;
  
  float u = (Kp * error) + (Kd * derivative);
  
  previousError = error;
  
  int16_t motorSpeed = constrain((int)u, -1023, 1023); 
  setMotors(motorSpeed, motorSpeed); 
  Serial.printf("Kp: %5.1f | Kd: %5.2f | Err: %5.2f | Motore: %4d\n", Kp, Kd, error, motorSpeed); 
  
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK); 
  M5.Display.setCursor(10, 100); 
  M5.Display.setTextSize(2); 
  M5.Display.printf("Kp: %5.1f \n", Kp);
  M5.Display.setCursor(10, 130);
  M5.Display.printf("Kd: %5.2f \n", Kd);
  M5.Display.setCursor(10, 160);
  M5.Display.printf("Mot: %4d   \n", motorSpeed);

  Serial.printf("P: %5.2f | Err: %5.2f | Out: %4d\n", filteredPitch, error, motorSpeed); 
  // Scrittura CSV
  /*
  String logString = String(u[0]) + "," + String(filteredPitch);
	csv = SD.open("/telemetria.csv", FILE_APPEND);
	if (csv) {
		csv.println(logString);
		csv.close();
	}
  */
  delay(10); 
}
