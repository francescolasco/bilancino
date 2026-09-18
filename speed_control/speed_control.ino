#include <M5Unified.h> 
#include <Wire.h>

/* Definizioni Globali */
int32_t eL = 0, eR = 0;  
int motor = 0; // 0 per left, 1 per right
int tick = 0;
uint32_t loopTimer; 
int dt = 1000; // 1 ms loop

// Variabili per il test a gradini
bool testRunning = true;
uint32_t startTime;
int currentPWM = 0;


/* Funzioni */
void setMotor(int16_t PWM, int motor) { 
  Wire.beginTransmission(0x3A);
  if(motor == 0){ Wire.write(0x00); } 
  else { Wire.write(0x02); }
  Wire.write((uint8_t)(PWM >> 8));
  Wire.write((uint8_t)(PWM & 0xFF));
  Wire.endTransmission(); 
}

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

void setup() {
  auto cfg = M5.config(); 
  M5.begin(cfg); 
  Serial.begin(1000000); 
  delay(1000); 
  
  M5.Power.setExtOutput(true); 
  Wire.begin(21, 22, 400000UL);

  readEncoders(&eL, &eR);
  tick = (motor == 0) ? eL : eR;
  
  Serial.println("Time_us,Ticks,PWM");
  
  startTime = micros();
  loopTimer = micros();
}

void loop() {
  if (!testRunning) return;

  uint32_t currentTime = micros() - startTime;
  
  if (currentTime < 500000) {
    currentPWM = 0;
  } 
  else if (currentTime < 2000000) {
    currentPWM = 400;
  }
  else if (currentTime < 3500000) {
    currentPWM = 800;
  }
  else if (currentTime < 5000000) {
    currentPWM = 400;
  }
  else if (currentTime < 6500000) {
    currentPWM = 0;
  }
  else {
    testRunning = false;
    Serial.println("Test concluso.");
    setMotor(0, motor); 
    return;
  }
  
  setMotor(currentPWM, motor);
  readEncoders(&eL, &eR);
  int32_t currentTick = (motor == 0) ? eL : eR;
  
  Serial.printf("%lu,%d,%d\n", currentTime, currentTick, currentPWM);

  while(micros() - loopTimer < dt){}
  loopTimer += dt;
}