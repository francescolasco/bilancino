//#include <M5StickC.h>
/*
#include "gyro.h"

void setup() {
  M5.begin();
  M5.IMU.Init(); // per far funzionare gyro
}

void loop() {
  delay(500);
  readGyro();
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("gyroX: %7.2f", gyroX);
} 
*/


// If you use Unit OLED, write this.
// #include <M5UnitOLED.h>

// If you use Unit LCD, write this.
 #include <M5UnitLCD.h>


// Include this to enable the M5 global instance.
#include <M5Unified.h>

// Strength of the calibration operation;
// 0: disables calibration.
// 1 is weakest and 255 is strongest.
static constexpr const uint8_t calib_value = 64;


struct rect_t
{
  int32_t x;
  int32_t y;
  int32_t w;
  int32_t h;
};

static constexpr const uint32_t color_tbl[18] = 
{
0xFF0000u, 0xCCCC00u, 0xCC00FFu,
0xFFCC00u, 0x00FF00u, 0x0088FFu,
0xFF00CCu, 0x00FFCCu, 0x0000FFu,
0xFF0000u, 0xCCCC00u, 0xCC00FFu,
0xFFCC00u, 0x00FF00u, 0x0088FFu,
0xFF00CCu, 0x00FFCCu, 0x0000FFu,
};
static constexpr const float coefficient_tbl[3] = { 0.5f, (1.0f / 256.0f), (1.0f / 1024.0f) };

static auto &dsp = (M5.Display);
static rect_t rect_graph_area;
static rect_t rect_text_area;

static uint8_t calib_countdown = 0;

static int prev_xpos[18];

// aggiunte da fra
float accel_angle = 0;
static float filtered_angle = 0;
uint32_t last_micros = 0;
// fine

void updateCalibration(uint32_t c, bool clear = false)
{
  calib_countdown = c;

  if (c == 0) {
    clear = true;
  }

  if (clear)
  {
    memset(prev_xpos, 0, sizeof(prev_xpos));
    dsp.fillScreen(TFT_BLACK);

    if (c)
    { // Start calibration.
      M5.Imu.setCalibration(calib_value, calib_value, calib_value);
    // ※ The actual calibration operation is performed each time during M5.Imu.update.
    // 
    // There are three arguments, which can be specified in the order of Accelerometer, gyro, and geomagnetic.
    // If you want to calibrate only the Accelerometer, do the following.
    // M5.Imu.setCalibration(100, 0, 0);
    //
    // If you want to calibrate only the gyro, do the following.
    // M5.Imu.setCalibration(0, 100, 0);
    //
    // If you want to calibrate only the geomagnetism, do the following.
    // M5.Imu.setCalibration(0, 0, 100);
    }
    else
    { // Stop calibration. (Continue calibration only for the geomagnetic sensor)
      M5.Imu.setCalibration(0, 0, calib_value);

      // If you want to stop all calibration, write this.
      // M5.Imu.setCalibration(0, 0, 0);

      // save calibration values.
      M5.Imu.saveOffsetToNVS();
    }
  }

  auto backcolor = (c == 0) ? TFT_BLACK : TFT_BLUE;
  dsp.fillRect(rect_text_area.x, rect_text_area.y, rect_text_area.w, rect_text_area.h, backcolor);

  if (c)
  {
    dsp.setCursor(rect_text_area.x + 2, rect_text_area.y + 1);
    dsp.setTextColor(TFT_WHITE, TFT_BLUE);
    dsp.printf("Countdown:%d ", c);
  }
}

void startCalibration(void)
{
  updateCalibration(10, true);
}

void setup(void)
{
  auto cfg = M5.config(); // M5Unified costruisce una struttura dati per configurazione
  M5.begin(cfg);

  int32_t w = dsp.width();
  int32_t h = dsp.height();
  if (w < h)
  {
    dsp.setRotation(dsp.getRotation() ^ 1);
    w = dsp.width();
    h = dsp.height();
  }
  int32_t graph_area_h = ((h - 8) / 18) * 18;
  int32_t text_area_h = h - graph_area_h;
  float fontsize = text_area_h / 8;
  dsp.setTextSize(fontsize);

  rect_graph_area = { 0, 0, w, graph_area_h };
  rect_text_area = {0, graph_area_h, w, text_area_h };


  // Read calibration values from non volatile storage.
  if (!M5.Imu.loadOffsetFromNVS())
  {
    startCalibration();
  }
}

void loop(void)
{
  static uint32_t frame_count = 0;
  static uint32_t prev_sec = 0;

  // To update the IMU value, use M5.Imu.update.
  // If a new value is obtained, the return value is non-zero.
  auto imu_update = M5.Imu.update();
  if (imu_update)
  {
    uint32_t now = micros();
    if (last_micros == 0){
      last_micros = now;
      return;
    }
    float dt = (now - last_micros) / 1000000.0f;
    last_micros = now;

    // Obtain data on the current value of the IMU.
    auto data = M5.Imu.getImuData();
    M5.Display.setCursor(0, 0);
    
    accel_angle = atan2(data.accel.y, data.accel.z) * 180.0 / PI;
    filtered_angle = 0.96 * (filtered_angle + data.gyro.x * dt) + 0.04 * accel_angle;
    M5.Display.printf("%f\n", filtered_angle);
/*
    // The data obtained by getImuData can be used as follows.
    data.accel.x;      // accel x-axis value.
    data.accel.y;      // accel y-axis value.
    data.accel.z;      // accel z-axis value.
    data.accel.value;  // accel 3values array [0]=x / [1]=y / [2]=z.

    data.gyro.x;      // gyro x-axis value.
    data.gyro.y;      // gyro y-axis value.
    data.gyro.z;      // gyro z-axis value.
    data.gyro.value;  // gyro 3values array [0]=x / [1]=y / [2]=z.

    data.mag.x;       // mag x-axis value.
    data.mag.y;       // mag y-axis value.
    data.mag.z;       // mag z-axis value.
    data.mag.value;   // mag 3values array [0]=x / [1]=y / [2]=z.

    data.value;       // all sensor 9values array [0~2]=accel / [3~5]=gyro / [6~8]=mag

    M5_LOGV("ax:%f  ay:%f  az:%f", data.accel.x, data.accel.y, data.accel.z);
    M5_LOGV("gx:%f  gy:%f  gz:%f", data.gyro.x , data.gyro.y , data.gyro.z );
    M5_LOGV("mx:%f  my:%f  mz:%f", data.mag.x  , data.mag.y  , data.mag.z  );
//*/
    ++frame_count;
  }
  else
  {
    M5.update(); // Legge lo stato della board (bottoni premuti ecc..)

    // Calibration is initiated when a button or screen is clicked.
    if (M5.BtnA.wasClicked() || M5.BtnPWR.wasClicked() || M5.Touch.getDetail().wasClicked())
    {
      startCalibration();
    }
  }

  int32_t sec = millis() / 1000; // ottieni il secondo
  if (prev_sec != sec) // ogni secondo effettua logging
  {
    prev_sec = sec;
    M5_LOGI("sec:%d  frame:%d", sec, frame_count);
    frame_count = 0;

    if (calib_countdown)
    {
      updateCalibration(calib_countdown - 1);
    }

    if ((sec & 7) == 0) // ogni 8 secondi
    { // prevent WDT.
      vTaskDelay(1);
    }
  }
}