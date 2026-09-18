#include "globals.h"

// Variabili del controllo
float Kp = 73.0; 
float Kd = 1.2;
float u = 0; 
float previousError = 0.0; 
float targetAngle = 0.0; 

// Variabili dei Sensori
float accX = 0, accY = 0, accZ = 0; 
float gyroX = 0, gyroY = 0, gyroZ = 0; 

float filteredPitch = 0.0; 

float gyroXBias = 0.0; 

int32_t offsetEnc = 0; 
int32_t lastPosition = 0;

// Variabili Temporizzazione
uint32_t loopTimer = 0;