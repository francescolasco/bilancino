#include <M5Unified.h>
#include <Wire.h>
#include <math.h>
#include <SD.h>

double u[1] = {0};
double y[2] = {0., 0.};

float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

float filteredPitch = 0.0;
float targetAngle = 0.0;
float gyroXBias = 0.0;
float speedIntegral = 0.0;

float pwm_scale = 3.5;

File csv;

// --- MATEMATICA PD (Kp = 15, Kd = 0.3) ---
void update_ssm(double *u, double *y) {
	static double x[1] = {0};
	double x1[1];
	y[0] = -15. * x[0] + 30. * u[0];
	y[1] = -15. * x[0] + 30. * u[0];
	x1[0] = u[0];
	x[0] = x1[0];
}

// Invio velocità alla base Bala2 (0x3A) - Modificato per 16 bit
void setMotors(int16_t speedL, int16_t speedR) {
	speedL = constrain(speedL, -1023, 1023);
	speedR = constrain(speedR, -1023, 1023);

	// Motore Sinistro (Registro 0x00, 2 byte)
	Wire.beginTransmission(0x3A);
	Wire.write(0x00);
	Wire.write((uint8_t)(speedL >> 8));
	Wire.write((uint8_t)(speedL & 0xFF));
	Wire.endTransmission();

	// Motore Destro (Registro 0x02, 2 byte)
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
		float p = -atan2(accY, accZ) * 180.0 / PI;
		pitchSum += p;
		gyroSum += gyroX;
		delay(10);
	}

	targetAngle = pitchSum / (float)samples;
	gyroXBias = gyroSum / (float)samples;
	filteredPitch = targetAngle;

	M5.Display.fillScreen(TFT_BLACK);
	M5.Display.setCursor(20, 40);
	M5.Display.setTextSize(3);
	M5.Display.printf("Zero: %.1f", targetAngle);
	M5.Display.setCursor(20, 90);
	M5.Display.printf("GBias: %.1f", gyroXBias);
	delay(1000);

	M5.Display.fillScreen(TFT_BLACK);
	M5.Display.setCursor(20, 60);
	M5.Display.println("Operativo");

	u[0] = 0;
	y[0] = 0;
	y[1] = 0;
}

void setup() {
	auto cfg = M5.config();
	M5.begin(cfg);
	Serial.begin(115200);
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

	delay(500);
	Serial.println("\n--- M5Stack Core Bala2 Avviato ---");

	// Abilita alimentazione verso la base
	M5.Power.setExtOutput(true);

	// Inizializzazione I2C specifica per M5Core (SDA=21, SCL=22)
	Wire.begin(21, 22, 400000UL);
	calibrateZero();
}

void loop() {
	M5.update();

	// Inserisci questo controllo all'inizio di loop()
	if (M5.BtnC.wasPressed()) {
		// 1. Ferma subito i motori
		setMotors(0, 0);
		// 2. Spegne lo schermo e disabilita l'uscita alla base
		M5.Display.sleep();
		M5.Power.setExtOutput(false);
		// 3. Entra in sospensione profonda fino alla successiva pressione del tasto di accensione
		M5.Power.deepSleep();
	}

	if (M5.BtnA.wasPressed()) {
		speedIntegral = 0;
		calibrateZero();
	}
	M5.Imu.getAccel(&accX, &accY, &accZ);
	M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ);
	float trueGyroX = gyroX - gyroXBias;
	float accPitch = -atan2(accY, accZ) * 180.0 / PI;
	filteredPitch = 0.98 * (filteredPitch - trueGyroX * 0.02) + 0.02 * accPitch;

	float virtualTarget = targetAngle + speedIntegral;
	float error = virtualTarget - filteredPitch;

	u[0] = (double)error;
	update_ssm(u, y);

	// Modificato per i 16 bit: moltiplichiamo x8 l'uscita del tuo SSM per scalarla sul nuovo range del Bala2
	int16_t motorSpeedL = constrain((int)(y[0] * pwm_scale), -1023, 1023);
	int16_t motorSpeedR = constrain((int)(y[1] * pwm_scale), -1023, 1023);

	setMotors(motorSpeedL, motorSpeedR);

	speedIntegral += (y[0] * 0.003);
	speedIntegral = constrain(speedIntegral, -3.0, 3.0);

	Serial.printf("Filt: %6.2f | Err: %6.2f | SpdInt: %5.2f | L: %4d\n", filteredPitch, error, speedIntegral, motorSpeedL);

	String logString = String(u[0]) + "," + String(filteredPitch);
	csv = SD.open("/telemetria.csv", FILE_APPEND);
	if (csv) {
		csv.println(logString);
		csv.close();
	}

	delay(20);
}
