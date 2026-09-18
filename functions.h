#pragma once
#include <stdint.h>

void readEncoders(int32_t *encL, int32_t *encR);

int32_t getAveragePosition();

void setMotors(int16_t speedL, int16_t speedR);

void calibrateZero();