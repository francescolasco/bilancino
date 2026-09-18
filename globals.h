#pragma once
#include <stdint.h>

// Variabili del controllo
extern float Kp; 
extern float Kd;
extern float u; 
extern float previousError; 
extern float targetAngle; 

// Variabili dei Sensori
extern float accX, accY, accZ; 
extern float gyroX, gyroY, gyroZ; 

extern float filteredPitch; 

extern float gyroXBias; 

extern int32_t offsetEnc; 
extern int32_t lastPosition;

// Variabili Temporizzazione
extern uint32_t loopTimer;