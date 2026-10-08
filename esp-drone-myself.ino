#include "arduino_secrets.h"

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ============================================================
// WIFI
// ============================================================
const char* WIFI_SSID = "ESP32-DRONE";
const char* WIFI_PASS = "12345678";
WebServer server(80);

// ============================================================
// MPU6050
// ============================================================
#define MPU_ADDR 0x68
#define SDA_PIN 21
#define SCL_PIN 22

int16_t axRaw, ayRaw, azRaw;
int16_t gxRaw, gyRaw, gzRaw;

float ax, ay, az;         // body-frame accel (g)
float gx, gy, gz;         // body-frame gyro (dps)
float gx_f, gy_f, gz_f;   // filtered body-frame gyro

float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

// ============================================================
// MPU REVERSAL â applied in SENSOR frame, before remap
// ============================================================
int revAccX  = -1;
int revAccY  = -1;
int revAccZ  =  1;
int revGyroX = -1;
int revGyroY = -1;
int revGyroZ = -1;

// ============================================================
// BODY-FRAME REVERSAL â applied AFTER the sideways remap
// ============================================================
int revRoll  = 1;
int revPitch = 1;
int revYaw   = 1;

// ============================================================
// SIDEWAYS MOUNT REMAP
// ============================================================
// Sensor is mounted sideways so that the raw IMU axes no longer
// line up with the drone body axes.  Verified by hand:
//   Nose Down  â raw roll  goes NEGATIVE
//   Nose Up    â raw roll  goes POSITIVE
//   Roll Left  â raw pitch goes NEGATIVE
//   Roll Right â raw pitch goes POSITIVE
//
// That is a 90Â° rotation about the drone Z axis.  Mapping used:
//   body_x (right)   = -imu_y
//   body_y (forward) = +imu_x
//   body_z (up)      = +imu_z
// And the gyro picks up the same rotation, with a yaw sign flip:
//   gx_body = -gy_imu
//   gy_body = +gx_imu
//   gz_body = -gz_imu
//
// If any axis is backwards on the bench, flip its sign HERE.
// ============================================================

// ============================================================
// ATTITUDE
// ============================================================
float rollAngle = 0, pitchAngle = 0, yawAngle = 0;
float accRoll = 0, accPitch = 0;

// ============================================================
// FLIGHT MODE
// 0 = RATE (tune this first)
// 1 = ANGLE (stabilized, cascade)
// ============================================================
int flightMode = 0;

// ============================================================
// PID â CASCADE (angle â rate)
// ============================================================
float rollAngleKp  = 6.0;
float pitchAngleKp = 6.0;
const float ANGLE_LIMIT = 30.0;

// Rate PID gains (tuned for 1 kHz loop)
float rollKp = 1.4,  rollKi = 0.08,  rollKd = 0.016;
float pitchKp = 1.4, pitchKi = 0.08, pitchKd = 0.016;
float yawKp  = 3.0,  yawKi  = 0.03,  yawKd  = 0.0;   // No D on yaw

// Yaw feedforward (Crazyflie-style)
float yawFF = 0.5;

// Angle mode feedforward
float angleFF = 1.0;

float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;
float rollPrevRate = 0, pitchPrevRate = 0, yawPrevRate = 0;

const float INTEGRAL_LIMIT = 30.0;
const float PID_LIMIT = 300.0;

// ============================================================
// TRIM
// ============================================================
float rollTrim = 0, pitchTrim = 0, yawTrim = 0;

// ============================================================
// MOTOR PINS â X FRAME
// Front = ESP32 antenna side
// ============================================================
#define M1_PIN 25   // FRONT-LEFT  - CW
#define M2_PIN 16   // FRONT-RIGHT - CCW
#define M3_PIN 33   // BACK-RIGHT  - CW
#define M4_PIN 32   // BACK-LEFT   - CCW

#define M1_CH 0
#define M2_CH 1
#define M3_CH 2
#define M4_CH 3

#define PWM_FREQ 5000
#define PWM_RES  10
#define PWM_MAX  1023
int MAX_MOTOR = 700;

// Motor idle floor â CRITICAL for brushed/gear motors
int MOTOR_IDLE = 90;

// ============================================================
// FLIGHT CONTROL
// ============================================================
bool armed = false;
float throttlePercent = 0;
float stickRoll = 0, stickPitch = 0, stickYaw = 0;

unsigned long lastWebCommand = 0;
const unsigned long FAILSAFE_MS = 1000;

// ============================================================
// SOUND / MOTOR-CHANNEL MUTEX
// ============================================================
// Replaces the old `volatile bool playingSound`.
// Core 1 must acquire this before touching the LEDC channels.
// While a chime is playing, Core 0 holds the mutex so Core 1
// simply skips its motor update â it never races ledcSetup().
SemaphoreHandle_t motorMux = NULL;

// ============================================================
// FILTERS â retuned for 1 kHz loop
// ============================================================
const float GYRO_LPF_ALPHA = 0.85;
const float D_LPF_ALPHA    = 0.70;
float dRollFilt = 0, dPitchFilt = 0, dYawFilt = 0;

// ============================================================
// TIMING â 1 kHz (Flix-style)
// ============================================================
unsigned long lastLoopMicros = 0;
const unsigned long LOOP_PERIOD_US = 1000;
const float DT = 0.001;

volatile bool calibrateRequested = false;

portMUX_TYPE flightMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void motorsOff();
void playArmSound();
void playDisarmSound();
void playCalibDoneSound();

// ============================================================
// MPU INIT
// ============================================================
void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void initMPU() {
  mpuWrite(0x6B, 0x00); delay(100);
  mpuWrite(0x19, 0);
  mpuWrite(0x1A, 3);
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1C, 0x00);
  delay(100);
}

// ============================================================
// READ MPU â sensor frame â drone body frame
// ============================================================
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return;

  int16_t imu_ax = (Wire.read() << 8) | Wire.read();
  int16_t imu_ay = (Wire.read() << 8) | Wire.read();
  int16_t imu_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t imu_gx = (Wire.read() << 8) | Wire.read();
  int16_t imu_gy = (Wire.read() << 8) | Wire.read();
  int16_t imu_gz = (Wire.read() << 8) | Wire.read();

  // Raw sensor values in engineering units (with per-axis reversal)
  float imu_axg = (imu_ax / 16384.0) * revAccX;
  float imu_ayg = (imu_ay / 16384.0) * revAccY;
  float imu_azg = (imu_az / 16384.0) * revAccZ;

  float imu_gxr = ((imu_gx / 131.0) - gyroXOffset) * revGyroX;
  float imu_gyr = ((imu_gy / 131.0) - gyroYOffset) * revGyroY;
  float imu_gzr = ((imu_gz / 131.0) - gyroZOffset) * revGyroZ;

  // ---- remap SIDEWAYS sensor frame â drone body frame ----
  // body_x (right)   = -imu_y
  // body_y (forward) = +imu_x
  // body_z (up)      = +imu_z
  // gyro follows the same rotation, with a yaw sign flip:
  // Then apply body-frame reversal on top.
  ax = -imu_ayg * revRoll;
  ay =  imu_axg * revPitch;
  az =  imu_azg * revYaw;

  gx = -imu_gyr * revRoll;
  gy =  imu_gxr * revPitch;
  gz = -imu_gzr * revYaw;

  // Update raw snapshot for telemetry (body frame)
  axRaw = imu_ax; ayRaw = imu_ay; azRaw = imu_az;
  gxRaw = imu_gx; gyRaw = imu_gy; gzRaw = imu_gz;

  // ---- LPF on body-frame gyro ----
  gx_f = gx_f * (1.0 - GYRO_LPF_ALPHA) + gx * GYRO_LPF_ALPHA;
  gy_f = gy_f * (1.0 - GYRO_LPF_ALPHA) + gy * GYRO_LPF_ALPHA;
  gz_f = gz_f * (1.0 - GYRO_LPF_ALPHA) + gz * GYRO_LPF_ALPHA;
}

// ============================================================
// STARTUP SOUND â DJI-style arpeggio via motor coils
// ============================================================
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_G5  784
#define NOTE_C6  1047

void playTone(int freq, int durationMs) {
  if (freq == 0) {
    motorsOff();
    delay(durationMs);
    return;
  }

  // Reconfigure all 4 channels to audio frequency
  ledcSetup(M1_CH, freq, PWM_RES);
  ledcSetup(M2_CH, freq, PWM_RES);
  ledcSetup(M3_CH, freq, PWM_RES);
  ledcSetup(M4_CH, freq, PWM_RES);

  ledcAttachPin(M1_PIN, M1_CH);
  ledcAttachPin(M2_PIN, M2_CH);
  ledcAttachPin(M3_PIN, M3_CH);
  ledcAttachPin(M4_PIN, M4_CH);

  // 50% duty = max acoustic output
  int halfDuty = (1 << PWM_RES) / 2;
  ledcWrite(M1_CH, halfDuty);
  ledcWrite(M2_CH, halfDuty);
  ledcWrite(M3_CH, halfDuty);
  ledcWrite(M4_CH, halfDuty);

  delay(durationMs);

  ledcWrite(M1_CH, 0);
  ledcWrite(M2_CH, 0);
  ledcWrite(M3_CH, 0);
  ledcWrite(M4_CH, 0);
}

void restoreMotorPWM() {
  ledcSetup(M1_CH, PWM_FREQ, PWM_RES);
  ledcSetup(M2_CH, PWM_FREQ, PWM_RES);
  ledcSetup(M3_CH, PWM_FREQ, PWM_RES);
  ledcSetup(M4_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_PIN, M1_CH);
  ledcAttachPin(M2_PIN, M2_CH);
  ledcAttachPin(M3_PIN, M3_CH);
  ledcAttachPin(M4_PIN, M4_CH);
  motorsOff();
}

// Rising arpeggio â ARM
void playArmSound() {
  playTone(NOTE_C5, 110); delay(30);
  playTone(NOTE_E5, 110); delay(30);
  playTone(NOTE_G5, 110); delay(30);
  playTone(NOTE_C6, 280); delay(60);
  restoreMotorPWM();
}

// Descending arpeggio â DISARM
void playDisarmSound() {
  playTone(NOTE_C6, 110); delay(30);
  playTone(NOTE_G5, 110); delay(30);
  playTone(NOTE_E5, 110); delay(30);
  playTone(NOTE_C5, 280); delay(60);
  restoreMotorPWM();
}

// Calibration complete â two-tone
void playCalibDoneSound() {
  playTone(NOTE_G5, 90); delay(40);
  playTone(NOTE_C6, 160); delay(40);
  restoreMotorPWM();
}

// ============================================================
// SOUND HELPER â owns the motor mutex for the whole chime
// ============================================================
void playSoundBlocking(void (*fn)()) {
  if (motorMux == NULL) { fn(); return; }   // safety
  if (xSemaphoreTake(motorMux, portMAX_DELAY) == pdTRUE) {
    fn();
    xSemaphoreGive(motorMux);
  }
}

// ============================================================
// GYRO CALIBRATION
// ============================================================
void calibrateGyro() {
  Serial.println("GYRO CALIBRATION - KEEP STILL");
  armed = false;
  motorsOff();
  delay(500);

  const int samples = 3000;
  long sx = 0, sy = 0, sz = 0;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
    if (Wire.available() >= 6) {
      sx += (int16_t)((Wire.read() << 8) | Wire.read());
      sy += (int16_t)((Wire.read() << 8) | Wire.read());
      sz += (int16_t)((Wire.read() << 8) | Wire.read());
    }
    delay(2);
  }

  // NOTE: offsets are stored in SENSOR frame, consistent with readMPU()
  gyroXOffset = ((float)sx / samples) / 131.0;
  gyroYOffset = ((float)sy / samples) / 131.0;
  gyroZOffset = ((float)sz / samples) / 131.0;

  Serial.printf("Offsets: GX=%.4f GY=%.4f GZ=%.4f\n",
                gyroXOffset, gyroYOffset, gyroZOffset);

  playSoundBlocking(playCalibDoneSound);
}

// ============================================================
// ATTITUDE â body-frame inputs
// ============================================================
void updateAttitude(float dt) {
  // Body frame: roll from Y/Z, pitch from X/Z, yaw from gyro Z
  accRoll  = atan2(ay, az) * 180.0 / PI;
  accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  rollAngle  += gx_f * dt;
  pitchAngle += gy_f * dt;
  yawAngle   += gz_f * dt;

  const float a = 0.98;
  rollAngle  = a * rollAngle  + (1.0 - a) * accRoll;
  pitchAngle = a * pitchAngle + (1.0 - a) * accPitch;

  if (yawAngle >  180) yawAngle -= 360;
  if (yawAngle < -180) yawAngle += 360;
}

// ============================================================
// MOTOR OUTPUT
// ============================================================
void motorWrite(uint8_t ch, int pwm) {
  ledcWrite(ch, constrain(pwm, 0, PWM_MAX));
}

void motorsOff() {
  ledcWrite(M1_CH, 0);
  ledcWrite(M2_CH, 0);
  ledcWrite(M3_CH, 0);
  ledcWrite(M4_CH, 0);
}

void resetPID() {
  rollIntegral = pitchIntegral = yawIntegral = 0;
  dRollFilt = dPitchFilt = dYawFilt = 0;
  rollPrevRate = pitchPrevRate = yawPrevRate = 0;
}

// ============================================================
// RATE PID
// ============================================================
float ratePID(float error, float rate, float dt,
              float Kp, float Ki, float Kd,
              float &integral, float &dFilt, float &prevRate) {
  (void)prevRate;
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // D on measurement only (avoids derivative kick)
  float dRaw = -rate;
  dFilt = dFilt * D_LPF_ALPHA + dRaw * (1.0 - D_LPF_ALPHA);

  float out = Kp * error + Ki * integral + Kd * dFilt;
  return constrain(out, -PID_LIMIT, PID_LIMIT);
}

// ============================================================
// MOTOR MIXER â X frame
// M1 = Front-Left (CW)   M2 = Front-Right (CCW)
// M4 = Back-Left  (CCW)  M3 = Back-Right  (CW)
// ============================================================
void updateMotors(float rollCorr, float pitchCorr, float yawCorr) {
  if (!armed) { motorsOff(); return; }

  int throttle = map((int)throttlePercent, 0, 100, MOTOR_IDLE, MAX_MOTOR);

  int M1 = throttle + rollCorr + pitchCorr - yawCorr;   // FL CW
  int M2 = throttle - rollCorr + pitchCorr + yawCorr;   // FR CCW
  int M3 = throttle - rollCorr - pitchCorr - yawCorr;   // BR CW
  int M4 = throttle + rollCorr - pitchCorr + yawCorr;   // BL CCW

  motorWrite(M1_CH, constrain(M1, MOTOR_IDLE, MAX_MOTOR));
  motorWrite(M2_CH, constrain(M2, MOTOR_IDLE, MAX_MOTOR));
  motorWrite(M3_CH, constrain(M3, MOTOR_IDLE, MAX_MOTOR));
  motorWrite(M4_CH, constrain(M4, MOTOR_IDLE, MAX_MOTOR));
}

// ============================================================
// CONTROL LOOP â 1 kHz on Core 1
// ============================================================
void controlTask(void *pv) {
  lastLoopMicros = micros();

  for (;;) {
    unsigned long now = micros();
    if (now - lastLoopMicros >= LOOP_PERIOD_US) {
      float dt = (now - lastLoopMicros) * 1e-6;
      if (dt > 0.05) dt = 0.05;
      lastLoopMicros = now;

      readMPU();
      updateAttitude(dt);

      float rollCorr = 0, pitchCorr = 0, yawCorr = 0;

      portENTER_CRITICAL(&flightMux);
      bool a = armed;
      float sR = stickRoll, sP = stickPitch, sY = stickYaw;
      float rT = rollTrim, pT = pitchTrim, yT = yawTrim;
      float rAKp = rollAngleKp, pAKp = pitchAngleKp;
      float rKp = rollKp, rKi = rollKi, rKd = rollKd;
      float pKp = pitchKp, pKi = pitchKi, pKd = pitchKd;
      float yKp = yawKp, yKi = yawKi, yKd = yawKd;
      float yFF = yawFF, aFF = angleFF;
      int mode = flightMode;
      portEXIT_CRITICAL(&flightMux);

      if (a) {
        float rollErr, pitchErr, yawErr;
        float rollRateTarget, pitchRateTarget, yawRateTarget;

        if (mode == 0) {
          // -------- RATE MODE --------
          rollRateTarget  = sR * 300.0;
          pitchRateTarget = sP * 300.0;
          yawRateTarget   = sY * 150.0;

          rollErr  = rollRateTarget  - gx_f;
          pitchErr = pitchRateTarget - gy_f;
          yawErr   = yawRateTarget   - gz_f;

        } else {
          // -------- ANGLE MODE (cascade + feedforward) --------
          float targetRoll  = sR * ANGLE_LIMIT + rT;
          float targetPitch = sP * ANGLE_LIMIT + pT;
          yawRateTarget     = sY * 120.0 + yT;

          float rollAngleErr  = targetRoll  - rollAngle;
          float pitchAngleErr = targetPitch - pitchAngle;

          rollRateTarget  = constrain(rollAngleErr  * rAKp, -300, 300)
                          + rollAngleErr  * aFF;
          pitchRateTarget = constrain(pitchAngleErr * pAKp, -300, 300)
                          + pitchAngleErr * aFF;

          rollErr  = rollRateTarget  - gx_f;
          pitchErr = pitchRateTarget - gy_f;
          yawErr   = yawRateTarget   - gz_f;
        }

        rollCorr  = ratePID(rollErr,  gx_f, dt, rKp, rKi, rKd, rollIntegral,  dRollFilt,  rollPrevRate);
        pitchCorr = ratePID(pitchErr, gy_f, dt, pKp, pKi, pKd, pitchIntegral, dPitchFilt, pitchPrevRate);
        yawCorr   = ratePID(yawErr,   gz_f, dt, yKp, yKi, yKd, yawIntegral,  dYawFilt,   yawPrevRate);

        yawCorr  += sY * 150.0 * yFF;
      }

      // Try to touch motors â skip for this tick if a chime owns the channels
      if (xSemaphoreTake(motorMux, 0) == pdTRUE) {
        updateMotors(rollCorr, pitchCorr, yawCorr);
        xSemaphoreGive(motorMux);
      }
    }
    vTaskDelay(1);
  }
}

// ============================================================
// WEB PAGE
// ============================================================
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP32 Drone</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{background:#0d0d0d;color:#eee;font-family:Arial;margin:0;padding:12px;user-select:none}
.card{max-width:1100px;margin:auto;background:#1a1a1a;border-radius:16px;padding:16px}
h1{font-size:20px;margin:0 0 10px;text-align:center;letter-spacing:2px;color:#8cf}
.status{text-align:center;font-size:14px;margin-bottom:10px}
.badge{padding:4px 12px;border-radius:20px;background:#333;font-weight:bold}
.badge.arm{background:#159447}
.badge.dis{background:#c62828}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}
@media(max-width:700px){.grid{grid-template-columns:1fr}}
.panel{background:#222;border-radius:12px;padding:12px}
.panel h2{font-size:13px;margin:0 0 10px;letter-spacing:1px;color:#8cf;text-transform:uppercase}
button{width:100%;padding:14px;margin:5px 0;border:0;border-radius:10px;font-size:15px;font-weight:bold;cursor:pointer;color:#fff}
.arm{background:#159447}.disarm{background:#c62828}.stop{background:#ef6c00}.cal{background:#0277bd}
.mode{background:#6a1b9a}
.btnrow{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.btnrow button{padding:12px 4px;font-size:13px}
.joy-wrap{display:flex;gap:16px;justify-content:center;align-items:center;flex-wrap:wrap;margin-top:14px}
.joy{position:relative;width:180px;height:180px;background:#111;border-radius:50%;border:2px solid #333;touch-action:none}
.joy .knob{position:absolute;width:60px;height:60px;background:#3498db;border-radius:50%;left:60px;top:60px;box-shadow:0 0 12px #3498db88;pointer-events:none}
.joy .lbl{position:absolute;bottom:-22px;width:100%;text-align:center;font-size:11px;color:#888}
.slider-v{display:flex;flex-direction:column;align-items:center}
.slider-v input{writing-mode:vertical-lr;direction:rtl;width:44px;height:180px}
.val{font-size:18px;font-weight:bold;color:#8cf;margin-top:4px}
.pid-row{display:flex;align-items:center;gap:8px;margin:6px 0;font-size:12px}
.pid-row label{width:34px;color:#aaa}
.pid-row input[type=range]{flex:1}
.pid-row .num{width:56px;text-align:right;color:#8cf;font-weight:bold}
.tele{display:grid;grid-template-columns:1fr 1fr;gap:6px;font-size:13px}
.tele div{background:#111;padding:6px 8px;border-radius:6px}
.tele b{color:#8cf}
.calib-note{font-size:11px;color:#f80;text-align:center;margin-top:6px;height:14px}
</style>
</head>
<body>
<div class="card">
<h1>ESP32 DRONE</h1>
<div class="status">Status: <span id="status" class="badge dis">DISARMED</span> &nbsp; Mode: <span id="modeLbl" class="badge">RATE</span></div>

<div class="grid">
<div class="panel">
<h2>Controls</h2>
<div class="btnrow">
<button class="arm" onclick="cmd('arm')">ARM</button>
<button class="disarm" onclick="cmd('disarm')">DISARM</button>
<button class="stop" onclick="cmd('stop')">STOP</button>
</div>
<button class="cal" onclick="cmd('calibrate')">CALIBRATE MPU</button>
<button class="mode" onclick="toggleMode()">TOGGLE MODE</button>
<div class="calib-note" id="calnote"></div>

<div class="joy-wrap">
<div class="slider-v">
<input type="range" id="thr" min="0" max="100" value="0" step="1">
<div class="val" id="thrV">0%</div>
</div>
<div class="joy" id="joy1">
<div class="knob" id="knob1"></div>
<div class="lbl">Roll / Pitch</div>
</div>
<div class="joy" id="joy2">
<div class="knob" id="knob2"></div>
<div class="lbl">Yaw</div>
</div>
</div>
</div>

<div class="panel">
<h2>Telemetry</h2>
<div class="tele">
<div>Roll: <b id="tRoll">0</b>&deg;</div>
<div>Pitch: <b id="tPitch">0</b>&deg;</div>
<div>Throttle: <b id="tThr">0</b>%</div>
<div>Yaw rate: <b id="tYaw">0</b>&deg;/s</div>
</div>

<h2 style="margin-top:16px">Trim</h2>
<div class="pid-row"><label>Roll</label><input type="range" id="trimR" min="-15" max="15" step="0.1" value="0"><span class="num" id="trimRV">0.0</span></div>
<div class="pid-row"><label>Pitch</label><input type="range" id="trimP" min="-15" max="15" step="0.1" value="0"><span class="num" id="trimPV">0.0</span></div>
<div class="pid-row"><label>Yaw</label><input type="range" id="trimY" min="-30" max="30" step="0.5" value="0"><span class="num" id="trimYV">0.0</span></div>

<h2 style="margin-top:16px">Roll PID</h2>
<div class="pid-row"><label>Kp</label><input type="range" id="rKp" min="0" max="5" step="0.01" value="1.4"><span class="num" id="rKpV">1.40</span></div>
<div class="pid-row"><label>Ki</label><input type="range" id="rKi" min="0" max="0.5" step="0.001" value="0.08"><span class="num" id="rKiV">0.080</span></div>
<div class="pid-row"><label>Kd</label><input type="range" id="rKd" min="0" max="0.1" step="0.0005" value="0.016"><span class="num" id="rKdV">0.0160</span></div>

<h2 style="margin-top:12px">Pitch PID</h2>
<div class="pid-row"><label>Kp</label><input type="range" id="pKp" min="0" max="5" step="0.01" value="1.4"><span class="num" id="pKpV">1.40</span></div>
<div class="pid-row"><label>Ki</label><input type="range" id="pKi" min="0" max="0.5" step="0.001" value="0.08"><span class="num" id="pKiV">0.080</span></div>
<div class="pid-row"><label>Kd</label><input type="range" id="pKd" min="0" max="0.1" step="0.0005" value="0.016"><span class="num" id="pKdV">0.0160</span></div>

<h2 style="margin-top:12px">Yaw PID</h2>
<div class="pid-row"><label>Kp</label><input type="range" id="yKp" min="0" max="5" step="0.01" value="3.0"><span class="num" id="yKpV">3.00</span></div>
<div class="pid-row"><label>Ki</label><input type="range" id="yKi" min="0" max="0.5" step="0.001" value="0.03"><span class="num" id="yKiV">0.030</span></div>
<div class="pid-row"><label>FF</label><input type="range" id="yFF" min="0" max="1.5" step="0.01" value="0.5"><span class="num" id="yFFV">0.50</span></div>

<h2 style="margin-top:12px">Angle P</h2>
<div class="pid-row"><label>Roll</label><input type="range" id="aR" min="1" max="15" step="0.1" value="6"><span class="num" id="aRV">6.0</span></div>
<div class="pid-row"><label>Pitch</label><input type="range" id="aP" min="1" max="15" step="0.1" value="6"><span class="num" id="aPV">6.0</span></div>
<div class="pid-row"><label>FF</label><input type="range" id="aFF" min="0" max="3" step="0.1" value="1"><span class="num" id="aFFV">1.0</span></div>

<h2 style="margin-top:12px">Motor</h2>
<div class="pid-row"><label>Max</label><input type="range" id="mMax" min="300" max="1023" step="5" value="700"><span class="num" id="mMaxV">700</span></div>
<div class="pid-row"><label>Idle</label><input type="range" id="mIdle" min="0" max="200" step="5" value="90"><span class="num" id="mIdleV">90</span></div>
</div>
</div>
</div>

<script>
const $ = id => document.getElementById(id);

function cmd(c){
  fetch("/"+c).then(r=>r.text()).then(t=>{
    if(c==='calibrate'){
      $('calnote').textContent='Calibrating... keep still!';
      setTimeout(()=>{ $('calnote').textContent='Done.'; }, 7000);
    }
  });
}

function toggleMode(){
  fetch('/mode').then(r=>r.text()).then(t=>{
    $('modeLbl').textContent = t;
  });
}

let thrTimer=null;
$('thr').addEventListener('input', e=>{
  const v = +e.target.value;
  $('thrV').textContent = v+'%';
  clearTimeout(thrTimer);
  thrTimer = setTimeout(()=>fetch('/throttle?v='+v), 30);
});

function bindNum(id, numId, dec, url){
  $(id).addEventListener('input', e=>{
    const v = +e.target.value;
    $(numId).textContent = v.toFixed(dec);
    fetch(url+'?v='+v);
  });
}
bindNum('trimR','trimRV',1,'/trim_roll');
bindNum('trimP','trimPV',1,'/trim_pitch');
bindNum('trimY','trimYV',1,'/trim_yaw');
bindNum('rKp','rKpV',2,'/pid?axis=roll&k=p&v=');
bindNum('rKi','rKiV',3,'/pid?axis=roll&k=i&v=');
bindNum('rKd','rKdV',4,'/pid?axis=roll&k=d&v=');
bindNum('pKp','pKpV',2,'/pid?axis=pitch&k=p&v=');
bindNum('pKi','pKiV',3,'/pid?axis=pitch&k=i&v=');
bindNum('pKd','pKdV',4,'/pid?axis=pitch&k=d&v=');
bindNum('yKp','yKpV',2,'/pid?axis=yaw&k=p&v=');
bindNum('yKi','yKiV',3,'/pid?axis=yaw&k=i&v=');
bindNum('yFF','yFFV',2,'/yawff?v=');
bindNum('aR','aRV',1,'/angle?axis=roll&v=');
bindNum('aP','aPV',1,'/angle?axis=pitch&v=');
bindNum('aFF','aFFV',1,'/angleff?v=');
bindNum('mMax','mMaxV',0,'/maxmotor?v=');
bindNum('mIdle','mIdleV',0,'/motoridle?v=');

function makeJoy(containerId, knobId, sendFn){
  const c = $(containerId), k = $(knobId);
  const R = 180/2 - 30;
  let active=false;

  function setKnob(x,y){
    k.style.left = (90 - 30 + x)+'px';
    k.style.top  = (90 - 30 + y)+'px';
    const nx = Math.max(-1,Math.min(1, x/R));
    const ny = Math.max(-1,Math.min(1,-y/R));
    sendFn(nx,ny);
  }
  function pos(e){
    const rect = c.getBoundingClientRect();
    const t = e.touches?e.touches[0]:e;
    let x = t.clientX - rect.left - 90;
    let y = t.clientY - rect.top  - 90;
    const d = Math.hypot(x,y);
    if(d > R){ x = x/d*R; y = y/d*R; }
    setKnob(x,y);
  }
  function reset(){ setKnob(0,0); }

  c.addEventListener('touchstart', e=>{e.preventDefault();active=true;pos(e);}, {passive:false});
  c.addEventListener('touchmove',  e=>{e.preventDefault();if(active)pos(e);}, {passive:false});
  c.addEventListener('touchend',   e=>{e.preventDefault();active=false;reset();}, {passive:false});

  c.addEventListener('mousedown', e=>{active=true;pos(e);});
  window.addEventListener('mousemove', e=>{if(active)pos(e);});
  window.addEventListener('mouseup', ()=>{if(active){active=false;reset();}});
}

makeJoy('joy1','knob1', (x,y)=>{
  fetch('/stick?roll='+x.toFixed(2)+'&pitch='+y.toFixed(2));
});
makeJoy('joy2','knob2', (x,y)=>{
  fetch('/stick?yaw='+x.toFixed(2));
});

setInterval(()=>{
  fetch('/data').then(r=>r.json()).then(d=>{
    $('tRoll').textContent = d.roll.toFixed(1);
    $('tPitch').textContent = d.pitch.toFixed(1);
    $('tThr').textContent = d.throttle.toFixed(0);
    $('tYaw').textContent = d.yawrate.toFixed(1);
    const s = $('status');
    s.textContent = d.armed?'ARMED':'DISARMED';
    s.className = 'badge '+(d.armed?'arm':'dis');
    $('modeLbl').textContent = d.mode===0?'RATE':'ANGLE';
  }).catch(()=>{});
}, 80);
</script>
</body>
</html>
)rawliteral";

// ============================================================
// WEB HANDLERS
// ============================================================
void handleRoot(){ server.send_P(200,"text/html",PAGE); lastWebCommand=millis(); }

void handleArm(){
  if(throttlePercent > 5){ server.send(400,"text/plain","LOW THROTTLE"); return; }
  resetPID();

  // Chime before arming â motorMux prevents Core 1 from touching
  // LEDC while the channels are reconfigured for audio.
  playSoundBlocking(playArmSound);

  armed = true;
  lastWebCommand = millis();
  server.send(200,"text/plain","ARMED");
}

void handleDisarm(){
  armed = false;
  throttlePercent = 0;
  stickRoll = stickPitch = stickYaw = 0;
  resetPID();
  motorsOff();

  playSoundBlocking(playDisarmSound);

  lastWebCommand = millis();
  server.send(200,"text/plain","DISARMED");
}

void handleStop(){
  armed = false;
  throttlePercent = 0;
  stickRoll = stickPitch = stickYaw = 0;
  resetPID();
  motorsOff();
  lastWebCommand = millis();
  server.send(200,"text/plain","STOPPED");
}

void handleThrottle(){
  if(!server.hasArg("v")){ server.send(400,"text/plain","ERR"); return; }
  portENTER_CRITICAL(&flightMux);
  throttlePercent = constrain(server.arg("v").toFloat(),0,100);
  portEXIT_CRITICAL(&flightMux);
  lastWebCommand = millis();
  server.send(200,"text/plain","OK");
}

void handleStick(){
  portENTER_CRITICAL(&flightMux);
  if(server.hasArg("roll"))  stickRoll  = constrain(server.arg("roll").toFloat(),  -1, 1);
  if(server.hasArg("pitch")) stickPitch = constrain(server.arg("pitch").toFloat(), -1, 1);
  if(server.hasArg("yaw"))   stickYaw   = constrain(server.arg("yaw").toFloat(),   -1, 1);
  portEXIT_CRITICAL(&flightMux);
  lastWebCommand = millis();
  server.send(200,"text/plain","OK");
}

void handleTrimRoll(){  portENTER_CRITICAL(&flightMux); rollTrim  = constrain(server.arg("v").toFloat(),-20,20); portEXIT_CRITICAL(&flightMux); lastWebCommand=millis(); server.send(200,"text/plain","OK"); }
void handleTrimPitch(){ portENTER_CRITICAL(&flightMux); pitchTrim = constrain(server.arg("v").toFloat(),-20,20); portEXIT_CRITICAL(&flightMux); lastWebCommand=millis(); server.send(200,"text/plain","OK"); }
void handleTrimYaw(){   portENTER_CRITICAL(&flightMux); yawTrim   = constrain(server.arg("v").toFloat(),-60,60); portEXIT_CRITICAL(&flightMux); lastWebCommand=millis(); server.send(200,"text/plain","OK"); }

void handlePID(){
  String axis = server.arg("axis");
  String k = server.arg("k");
  float v = server.arg("v").toFloat();
  portENTER_CRITICAL(&flightMux);
  if(axis=="roll"){
    if(k=="p") rollKp  = constrain(v,0,10);
    if(k=="i") rollKi  = constrain(v,0,2);
    if(k=="d") rollKd  = constrain(v,0,1);
  } else if(axis=="pitch"){
    if(k=="p") pitchKp = constrain(v,0,10);
    if(k=="i") pitchKi = constrain(v,0,2);
    if(k=="d") pitchKd = constrain(v,0,1);
  } else if(axis=="yaw"){
    if(k=="p") yawKp   = constrain(v,0,10);
    if(k=="i") yawKi   = constrain(v,0,2);
    if(k=="d") yawKd   = constrain(v,0,1);
  }
  portEXIT_CRITICAL(&flightMux);
  lastWebCommand=millis();
  server.send(200,"text/plain","OK");
}

void handleAngle(){
  String axis = server.arg("axis");
  float v = server.arg("v").toFloat();
  portENTER_CRITICAL(&flightMux);
  if(axis=="roll")  rollAngleKp  = constrain(v,0.5,20);
  if(axis=="pitch") pitchAngleKp = constrain(v,0.5,20);
  portEXIT_CRITICAL(&flightMux);
  lastWebCommand=millis();
  server.send(200,"text/plain","OK");
}

void handleYawFF(){
  portENTER_CRITICAL(&flightMux);
  yawFF = constrain(server.arg("v").toFloat(), 0, 1.5);
  portEXIT_CRITICAL(&flightMux);
  lastWebCommand=millis();
  server.send(200,"text/plain","OK");
}

void handleAngleFF(){
  portENTER_CRITICAL(&flightMux);
  angleFF = constrain(server.arg("v").toFloat(), 0, 3.0);
  portEXIT_CRITICAL(&flightMux);
  lastWebCommand=millis();
  server.send(200,"text/plain","OK");
}

void handleMaxMotor(){
  int v = server.arg("v").toInt();
  MAX_MOTOR = constrain(v, 300, 1023);
  lastWebCommand=millis();
  server.send(200,"text/plain","OK");
}

void handleMotorIdle(){
  int v = server.arg("v").toInt();
  MOTOR_IDLE = constrain(v, 0, 300);
  lastWebCommand=millis();
  server.send(200,"text/plain","OK");
}

void handleMode(){
  portENTER_CRITICAL(&flightMux);
  flightMode = (flightMode == 0) ? 1 : 0;
  int m = flightMode;
  portEXIT_CRITICAL(&flightMux);
  resetPID();
  lastWebCommand=millis();
  server.send(200,"text/plain", m==0?"RATE":"ANGLE");
}

void handleCalibrate(){
  calibrateRequested = true;
  lastWebCommand = millis();
  server.send(200,"text/plain","CALIBRATING");
}

void handleData(){
  portENTER_CRITICAL(&flightMux);
  float r=rollAngle, p=pitchAngle, t=throttlePercent, b=armed?1:0;
  float yr=gz_f;
  int m = flightMode;
  portEXIT_CRITICAL(&flightMux);

  String j="{";
  j+="\"roll\":"+String(r,2);
  j+=",\"pitch\":"+String(p,2);
  j+=",\"throttle\":"+String(t,0);
  j+=",\"yawrate\":"+String(yr,1);
  j+=",\"armed\":"+String(b?"true":"false");
  j+=",\"mode\":"+String(m);
  j+="}";
  server.send(200,"application/json",j);
  lastWebCommand=millis();
}

// ============================================================
// FAILSAFE
// ============================================================
void checkFailsafe(){
  if(!armed) return;
  if(millis()-lastWebCommand > FAILSAFE_MS){
    Serial.println("FAILSAFE");
    portENTER_CRITICAL(&flightMux);
    armed=false; throttlePercent=0;
    stickRoll=stickPitch=stickYaw=0;
    portEXIT_CRITICAL(&flightMux);
    resetPID();
    motorsOff();
  }
}

// ============================================================
// SETUP
// ============================================================
void setup(){
  Serial.begin(115200);
  delay(300);

  Serial.println("\n================================");
  Serial.println("ESP32 DRONE v2.2");
  Serial.println("Mutex-guarded sound | Sideways MPU");
  Serial.println("1 kHz loop | Rate + Angle modes");
  Serial.println("Pins: M1=25 M2=16 M3=33 M4=32");
  Serial.println("================================");

  // Create the motor-channel mutex BEFORE anything can touch PWM
  motorMux = xSemaphoreCreateMutex();
  configASSERT(motorMux != NULL);

  ledcSetup(M1_CH,PWM_FREQ,PWM_RES);
  ledcSetup(M2_CH,PWM_FREQ,PWM_RES);
  ledcSetup(M3_CH,PWM_FREQ,PWM_RES);
  ledcSetup(M4_CH,PWM_FREQ,PWM_RES);
  ledcAttachPin(M1_PIN,M1_CH);
  ledcAttachPin(M2_PIN,M2_CH);
  ledcAttachPin(M3_PIN,M3_CH);
  ledcAttachPin(M4_PIN,M4_CH);
  motorsOff();

  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(400000);

  initMPU();
  calibrateGyro();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID,WIFI_PASS);
  WiFi.setSleep(false);

  Serial.printf("AP: %s / %s\n", WIFI_SSID, WIFI_PASS);
  Serial.printf("Open: http://%s\n", WiFi.softAPIP().toString().c_str());

  server.on("/",            HTTP_GET, handleRoot);
  server.on("/arm",         HTTP_GET, handleArm);
  server.on("/disarm",      HTTP_GET, handleDisarm);
  server.on("/stop",        HTTP_GET, handleStop);
  server.on("/calibrate",   HTTP_GET, handleCalibrate);
  server.on("/throttle",    HTTP_GET, handleThrottle);
  server.on("/stick",       HTTP_GET, handleStick);
  server.on("/trim_roll",   HTTP_GET, handleTrimRoll);
  server.on("/trim_pitch",  HTTP_GET, handleTrimPitch);
  server.on("/trim_yaw",    HTTP_GET, handleTrimYaw);
  server.on("/pid",         HTTP_GET, handlePID);
  server.on("/angle",       HTTP_GET, handleAngle);
  server.on("/yawff",       HTTP_GET, handleYawFF);
  server.on("/angleff",     HTTP_GET, handleAngleFF);
  server.on("/maxmotor",    HTTP_GET, handleMaxMotor);
  server.on("/motoridle",   HTTP_GET, handleMotorIdle);
  server.on("/mode",        HTTP_GET, handleMode);
  server.on("/data",        HTTP_GET, handleData);
  server.begin();

  lastWebCommand = millis();

  xTaskCreatePinnedToCore(
    controlTask, "ctrl", 4096, NULL, 10, NULL, 1
  );

  Serial.println("Ready. Control loop @ 1 kHz on Core 1. Default: RATE mode.");
}

// ============================================================
// LOOP (Core 0)
// ============================================================
void loop(){
  server.handleClient();
  checkFailsafe();

  if(calibrateRequested){
    calibrateRequested = false;
    calibrateGyro();
  }
}