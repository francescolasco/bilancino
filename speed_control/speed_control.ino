#include <M5Unified.h> 

/* Definizioni Globali*/
int32_t eL = 0, eR = 0;  // lettura encoder motori settata a 0 tick
int motor = 0; // 0 per left e 1 per right
int PWMSignal = 500;
int tick = 0;
float tickSpeed = 0;
uint32_t loopTimer; // per temporizzazione

/* Funzioni */

// Funzione per settare il segnale PWM a un motore
void setMotor(int16_t PWM, int motor) { 
  Wire.beginTransmission(0x3A);
  if(motor == 0){// motore sinistro
    Wire.write(0x00); 
  } else if(motor == 1){// motore destro
    Wire.write(0x02); 
  }
  Wire.write((uint8_t)(PWM >> 8));
  Wire.write((uint8_t)(PWM & 0xFF));
  Wire.endTransmission(); 
}

// Funzione per leggere encoder motori, legge i 'tick'
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
  /* Setup M5 */
  auto cfg = M5.config(); 
  M5.begin(cfg); 
  Serial.begin(115200); 
  delay(500); 
  
  M5.Power.setExtOutput(true); 
  Wire.begin(21, 22, 400000UL);

  loopTimer = micros();
  Serial.println("Setup finito.");
}

void loop() {
  setMotor(PWMSignal, motor);
  readEncoders(&eL, &eR);
  if(motor == 0){// motore sinistro
    tickSpeed = (eL - tick)/0.005;
    tick = eL;
  } else if(motor == 1){// motore destro
    tickSpeed = (eR - tick)/0.005;
    tick = eR; 
  }

  Serial.printf("tick/s = %5.1f\n", tickSpeed);
  /* Temporizzazione */
  while(micros() - loopTimer < 5000){}
}
