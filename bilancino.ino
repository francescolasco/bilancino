#include <M5Unified.h> 
#include "functions.h"
#include "globals.h"
// #include <SD.h>

void setup() { 
  auto cfg = M5.config(); 
  M5.begin(cfg); 
  Serial.begin(115200); 
  delay(500); 
  
  M5.Power.setExtOutput(true); 
  Wire.begin(21, 22, 400000UL); 
  calibrateZero();

  loopTimer = micros(); 
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
  
  filteredPitch = 0.99 * (filteredPitch - trueGyroX * 0.005) + 0.01 * accPitch; 
  
  int32_t currentPos = getAveragePosition();
  float currentSpeed = (float)(currentPos - lastPosition)/0.005; 
  lastPosition = currentPos;
  float Kp_enc = 0.005; 
  float Kd_enc = 0.02;    
  float angleCorrection = (currentPos * Kp_enc) + (currentSpeed * Kd_enc);
  angleCorrection = constrain(angleCorrection, -2.0, 2.0);

  float error = targetAngle - angleCorrection - filteredPitch; 
  
  float dt = 0.005; 
  float derivative = (error - previousError) / dt;
  
  float u = (Kp * error) + (Kd * derivative);
  
  previousError = error;
  
  int16_t motorSpeed = constrain((int)u, -1023, 1023); 
  setMotors(motorSpeed, motorSpeed); 
  
  /* LOG */
  //Serial.printf("Kp: %5.1f | Kd: %5.2f | Err: %5.2f | Motore: %4d\n", Kp, Kd, error, motorSpeed); 
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK); 
  M5.Display.setCursor(10, 100); 
  M5.Display.setTextSize(2); 
  M5.Display.printf("Kp: %5.1f \n", Kp);
  M5.Display.setCursor(10, 130);
  M5.Display.printf("Kd: %5.2f \n", Kd);
  M5.Display.setCursor(10, 160);
  M5.Display.printf("Mot: %4d   \n", motorSpeed);

  /* SCRITTURA CSV */
  /*
  String logString = String(u[0]) + "," + String(filteredPitch);
	csv = SD.open("/telemetria.csv", FILE_APPEND);
	if (csv) {
		csv.println(logString);
		csv.close();
	}
  */
  /* TEMPORIZZAZIONE */
  while(micros() - loopTimer < 5000){}
  loopTimer += 5000;
}
