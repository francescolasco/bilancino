
float gyroX; // side to side
float gyroY; // top to bottom (da verificare)
float gyroZ; // points upward (da verificare)

//TODO filtrino
void readGyro() {
  M5.Imu.getGyroData( &gyroX, &gyroY, &gyroZ);
  // M5.Imu.getAccelData(&aX,&aY,&aZ)
}