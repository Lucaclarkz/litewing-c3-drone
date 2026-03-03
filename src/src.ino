// ESP32-C3 LiteWing (CRTP/UDP) Ready-to-Fly - MPU6050 + Brushed Quad X (Arduino)
// MPU6050: SCL=GPIO6, SDA=GPIO7
// Orientation: Y arrow = FRONT, X arrow = RIGHT
// Motors (X frame):
//  M1 GPIO21 Front Left  (CW)
//  M2 GPIO20 Front Right (CCW)  RIGHT side
//  M3 GPIO10 Back  Right (CW)   RIGHT side
//  M4 GPIO5  Back  Left  (CCW)
// LED: GPIO8 (boot blink, WiFi ready solid)
//
// UDP/CRTP Commander on port 2390
// Packet: [hdr][roll f32][pitch f32][yawrate f32][thrust u16][cksum u8]
// cksum = sum(bytes[0..n-2]) mod256
//
// Safety: failsafe + instant stop on throttle release.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>

// ===================== USER CONFIG =====================
static const char* AP_PASS = "12345678";
static const int   AP_CH   = 6;
static const uint16_t UDP_PORT_RX = 2390;

// LED
static const int PIN_LED = 8;

// I2C
static const int PIN_SCL = 6;
static const int PIN_SDA = 7;
static const uint32_t I2C_HZ = 400000;

// MPU6050
static const uint8_t MPU_ADDR = 0x68;

// Motors
static const int PIN_M1 = 21;
static const int PIN_M2 = 20;
static const int PIN_M3 = 10;
static const int PIN_M4 = 5;

// PWM
static const int PWM_FREQ_HZ = 20000;
static const int PWM_RES_BITS = 8;
static const int PWM_MAX = 255;
static const int MOTOR_MAX_LIMIT = 240;

// Safety
static const uint32_t FAILSAFE_MS = 250;
static const float THR_STOP = 0.03f;
static const float THR_ARM  = 0.08f;

// Loop
static const float LOOP_HZ = 250.0f;
static const float LOOP_DT = 1.0f / LOOP_HZ;

// Filter
static const float CF_ALPHA = 0.98f;

// Limits
static const float MAX_ANGLE_DEG = 30.0f;
static const float MAX_RATE_RP   = 220.0f;
static const float MAX_RATE_Y    = 180.0f;

// Motor tuning
static int motorMin[4]    = {55, 45, 45, 45};          // M1 boosted
static float motorGain[4] = {1.00f, 0.92f, 0.92f, 1.00f}; // right side gain reduce

// PID
static float Kp_angle = 4.5f;

static float Kp_rate_rp = 0.09f;
static float Ki_rate_rp = 0.18f;
static float Kd_rate_rp = 0.0025f;

static float Kp_rate_y  = 0.12f;
static float Ki_rate_y  = 0.10f;
static float Kd_rate_y  = 0.0f;

// flip if yaw direction wrong
static float yawSign = 1.0f;

// ===================== STATE =====================
WiFiUDP Udp;

struct Cmd {
  float roll_deg;
  float pitch_deg;
  float yawrate_dps;
  float thrust;     // 0..1
  uint32_t last_ms;
};

static Cmd g_cmd = {0,0,0,0,0};   // <-- FIX: not volatile
static bool armed = false;

// attitude
static float roll_deg = 0, pitch_deg = 0;
static float rollTrim = 0, pitchTrim = 0;

// gyro bias (FRAME deg/s)
static float gbx=0, gby=0, gbz=0;

// pid integrators
static float i_r=0, i_p=0, i_y=0;
static float prev_er=0, prev_ep=0, prev_ey=0;

// PWM channels
static const int CH_M1 = 0;
static const int CH_M2 = 1;
static const int CH_M3 = 2;
static const int CH_M4 = 3;

// LED
static bool led_blink = true;
static uint32_t led_t = 0;

// ===================== UTILS =====================
static inline float clampf(float x, float a, float b) {
  return (x < a) ? a : (x > b) ? b : x;
}
static inline uint8_t sum_mod256(const uint8_t* d, size_t n) {
  uint32_t s=0; for (size_t i=0;i<n;i++) s += d[i]; return (uint8_t)(s & 0xFF);
}
static inline uint32_t ms() { return (uint32_t)millis(); }

static inline void motors_off() {
  ledcWrite(CH_M1, 0);
  ledcWrite(CH_M2, 0);
  ledcWrite(CH_M3, 0);
  ledcWrite(CH_M4, 0);
}
static inline void motor_write(int ch, int duty) {
  if (duty < 0) duty = 0;
  if (duty > PWM_MAX) duty = PWM_MAX;
  ledcWrite(ch, duty);
}

// ===================== MPU6050 I2C =====================
static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}
static bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t* out, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
  if (got != len) return false;
  for (size_t i=0;i<len;i++) out[i] = (uint8_t)Wire.read();
  return true;
}

static void mpu_init() {
  i2c_write_reg(MPU_ADDR, 0x6B, 0x00);
  delay(20);
  i2c_write_reg(MPU_ADDR, 0x1B, 0x08); // gyro 500dps
  i2c_write_reg(MPU_ADDR, 0x1C, 0x10); // accel 8g
  i2c_write_reg(MPU_ADDR, 0x1A, 0x04); // dlpf ~21Hz
}

static void mpu_read_sensor(float* axg, float* ayg, float* azg, float* gxdps, float* gydps, float* gzdps) {
  uint8_t b[14] = {0};
  if (!i2c_read_regs(MPU_ADDR, 0x3B, b, sizeof(b))) {
    *axg=*ayg=*azg=*gxdps=*gydps=*gzdps=0;
    return;
  }
  int16_t axr = (int16_t)((b[0]<<8) | b[1]);
  int16_t ayr = (int16_t)((b[2]<<8) | b[3]);
  int16_t azr = (int16_t)((b[4]<<8) | b[5]);
  int16_t gxr = (int16_t)((b[8]<<8) | b[9]);
  int16_t gyr = (int16_t)((b[10]<<8)| b[11]);
  int16_t gzr = (int16_t)((b[12]<<8)| b[13]);

  *axg = (float)axr / 4096.0f;
  *ayg = (float)ayr / 4096.0f;
  *azg = (float)azr / 4096.0f;

  *gxdps = (float)gxr / 65.5f;
  *gydps = (float)gyr / 65.5f;
  *gzdps = (float)gzr / 65.5f;
}

// REMAP: Y front, X right => frame: X=Y, Y=-X, Z=Z
static inline void remap_frame(float ax_s, float ay_s, float az_s,
                               float gx_s, float gy_s, float gz_s,
                               float* ax, float* ay, float* az,
                               float* gx, float* gy, float* gz) {
  *ax = ay_s;
  *ay = -ax_s;
  *az = az_s;

  *gx = gy_s;
  *gy = -gx_s;
  *gz = gz_s;
}

static void gyro_calibrate() {
  float sx=0, sy=0, sz=0;
  const int N = 900;
  for (int i=0;i<N;i++) {
    float ax_s,ay_s,az_s,gx_s,gy_s,gz_s;
    mpu_read_sensor(&ax_s,&ay_s,&az_s,&gx_s,&gy_s,&gz_s);
    float ax,ay,az,gx,gy,gz;
    remap_frame(ax_s,ay_s,az_s,gx_s,gy_s,gz_s,&ax,&ay,&az,&gx,&gy,&gz);
    sx += gx; sy += gy; sz += gz;
    delay(2);
  }
  gbx = sx/N; gby = sy/N; gbz = sz/N;
  i_r=i_p=i_y=0; prev_er=prev_ep=prev_ey=0;
}

static void level_trim_calibrate() {
  float r=0, p=0, sr=0, sp=0;
  const int N=180;
  for (int i=0;i<N;i++) {
    float ax_s,ay_s,az_s,gx_s,gy_s,gz_s;
    mpu_read_sensor(&ax_s,&ay_s,&az_s,&gx_s,&gy_s,&gz_s);
    float ax,ay,az,gx,gy,gz;
    remap_frame(ax_s,ay_s,az_s,gx_s,gy_s,gz_s,&ax,&ay,&az,&gx,&gy,&gz);
    gx -= gbx; gy -= gby;

    float accRoll  = atan2f(ay, az) * 180.0f / (float)M_PI;
    float accPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;

    r = CF_ALPHA*(r + gx*LOOP_DT) + (1.0f-CF_ALPHA)*accRoll;
    p = CF_ALPHA*(p + gy*LOOP_DT) + (1.0f-CF_ALPHA)*accPitch;

    sr += r; sp += p;
    delay(5);
  }
  rollTrim  = sr/N;
  pitchTrim = sp/N;
}

// ===================== WiFi AP =====================
static void wifi_init_ap() {
  WiFi.mode(WIFI_AP);
  uint8_t mac[6];
  WiFi.softAPmacAddress(mac);

  char ssid[40];
  snprintf(ssid, sizeof(ssid), "LiteWing_%02X%02X%02X%02X%02X%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  WiFi.softAP(ssid, AP_PASS, AP_CH, 0, 1);
  delay(100);
  Udp.begin(UDP_PORT_RX);
}

// ===================== UDP RX =====================
static bool udp_read_cmd(Cmd* out) {
  int psize = Udp.parsePacket();
  if (psize <= 0) return false;
  if (psize > 64) psize = 64;

  uint8_t buf[64];
  int len = Udp.read(buf, psize);
  if (len < 2) return false;

  if (buf[len-1] != sum_mod256(buf, (size_t)len-1)) return false;

  uint8_t h = buf[0];
  uint8_t port = (h >> 4) & 0x0F;
  uint8_t ch   = h & 0x03;

  if (!(port == 3 && ch == 0)) return false;
  if (len < (1 + 4+4+4 + 2 + 1)) return false;

  float r,p,y;
  uint16_t t16;
  memcpy(&r, &buf[1], 4);
  memcpy(&p, &buf[5], 4);
  memcpy(&y, &buf[9], 4);
  memcpy(&t16,&buf[13],2);

  out->roll_deg    = clampf(r, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
  out->pitch_deg   = clampf(p, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
  out->yawrate_dps = clampf(y, -MAX_RATE_Y, MAX_RATE_Y);
  out->thrust      = clampf((float)t16 / 65535.0f, 0.0f, 1.0f);
  out->last_ms     = ms();
  return true;
}

void setup() {
  // LED boot blink
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  led_blink = true; led_t = ms();

  // Motor low (reduce boot blip)
  pinMode(PIN_M1, OUTPUT); digitalWrite(PIN_M1, LOW);
  pinMode(PIN_M2, OUTPUT); digitalWrite(PIN_M2, LOW);
  pinMode(PIN_M3, OUTPUT); digitalWrite(PIN_M3, LOW);
  pinMode(PIN_M4, OUTPUT); digitalWrite(PIN_M4, LOW);

  // PWM
  ledcSetup(CH_M1, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_M2, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_M3, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_M4, PWM_FREQ_HZ, PWM_RES_BITS);

  ledcAttachPin(PIN_M1, CH_M1);
  ledcAttachPin(PIN_M2, CH_M2);
  ledcAttachPin(PIN_M3, CH_M3);
  ledcAttachPin(PIN_M4, CH_M4);
  motors_off();

  // I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  mpu_init();

  delay(300);
  gyro_calibrate();
  level_trim_calibrate();

  wifi_init_ap();

  // WiFi ready -> solid LED
  led_blink = false;
  digitalWrite(PIN_LED, HIGH);

  g_cmd = {0,0,0,0, ms()};
}

void loop() {
  // LED blink (only while boot)
  if (led_blink && (ms() - led_t) > 250) {
    led_t = ms();
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }

  // RX packets
  Cmd cnew;
  while (udp_read_cmd(&cnew)) g_cmd = cnew;

  const uint32_t now = ms();
  Cmd c = g_cmd; // now works

  // FAILSAFE
  if ((now - c.last_ms) > FAILSAFE_MS) {
    armed = false;
    motors_off();
    i_r=i_p=i_y=0; prev_er=prev_ep=prev_ey=0;
    delay(1);
    return;
  }

  // Instant stop
  if (c.thrust < THR_STOP) {
    armed = false;
    motors_off();
    i_r=i_p=i_y=0; prev_er=prev_ep=prev_ey=0;
    delay(1);
    return;
  }

  // Arm gate
  if (!armed) {
    if (c.thrust > THR_ARM) {
      armed = true;
      i_r=i_p=i_y=0; prev_er=prev_ep=prev_ey=0;
    } else {
      motors_off();
      delay(1);
      return;
    }
  }

  // IMU
  float ax_s,ay_s,az_s,gx_s,gy_s,gz_s;
  mpu_read_sensor(&ax_s,&ay_s,&az_s,&gx_s,&gy_s,&gz_s);

  float ax,ay,az,gx,gy,gz;
  remap_frame(ax_s,ay_s,az_s,gx_s,gy_s,gz_s,&ax,&ay,&az,&gx,&gy,&gz);

  gx -= gbx; gy -= gby; gz -= gbz;

  float accRoll  = atan2f(ay, az) * 180.0f / (float)M_PI;
  float accPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;

  roll_deg  = CF_ALPHA*(roll_deg  + gx*LOOP_DT) + (1.0f-CF_ALPHA)*accRoll;
  pitch_deg = CF_ALPHA*(pitch_deg + gy*LOOP_DT) + (1.0f-CF_ALPHA)*accPitch;

  float rollLevel  = roll_deg  - rollTrim;
  float pitchLevel = pitch_deg - pitchTrim;

  float des_r = clampf((c.roll_deg  - rollLevel)  * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
  float des_p = clampf((c.pitch_deg - pitchLevel) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
  float des_y = clampf(c.yawrate_dps, -MAX_RATE_Y, MAX_RATE_Y);

  // PID roll
  float er = des_r - gx;
  i_r = clampf(i_r + er*LOOP_DT, -200.0f, 200.0f);
  float dr = (er - prev_er)/LOOP_DT; prev_er = er;
  float out_r = Kp_rate_rp*er + Ki_rate_rp*i_r + Kd_rate_rp*dr;

  // PID pitch
  float ep = des_p - gy;
  i_p = clampf(i_p + ep*LOOP_DT, -200.0f, 200.0f);
  float dp = (ep - prev_ep)/LOOP_DT; prev_ep = ep;
  float out_p = Kp_rate_rp*ep + Ki_rate_rp*i_p + Kd_rate_rp*dp;

  // PID yaw
  float ey = des_y - gz;
  i_y = clampf(i_y + ey*LOOP_DT, -200.0f, 200.0f);
  float dy = (ey - prev_ey)/LOOP_DT; prev_ey = ey;
  float out_y = Kp_rate_y*ey + Ki_rate_y*i_y + Kd_rate_y*dy;

  float yawTerm = yawSign * out_y;

  int minAvg = (motorMin[0]+motorMin[1]+motorMin[2]+motorMin[3]) / 4;
  int usable = MOTOR_MAX_LIMIT - minAvg;
  int base = minAvg + (int)(c.thrust * (float)usable);
  base = (base < 0) ? 0 : (base > MOTOR_MAX_LIMIT ? MOTOR_MAX_LIMIT : base);

  float m[4];
  m[0] = base - out_p + out_r - yawTerm;
  m[1] = base - out_p - out_r + yawTerm;
  m[2] = base + out_p - out_r - yawTerm;
  m[3] = base + out_p + out_r + yawTerm;

  for (int i=0;i<4;i++) {
    if (m[i] > 0 && m[i] < motorMin[i]) m[i] = motorMin[i];
    m[i] *= motorGain[i];
  }

  float maxOut = fmaxf(fmaxf(m[0],m[1]), fmaxf(m[2],m[3]));
  if (maxOut > (float)MOTOR_MAX_LIMIT) {
    float s = maxOut - (float)MOTOR_MAX_LIMIT;
    for (int i=0;i<4;i++) m[i] -= s;
  }

  for (int i=0;i<4;i++) m[i] = clampf(m[i], 0.0f, (float)PWM_MAX);

  motor_write(CH_M1, (int)m[0]);
  motor_write(CH_M2, (int)m[1]);
  motor_write(CH_M3, (int)m[2]);
  motor_write(CH_M4, (int)m[3]);

  delayMicroseconds((int)(1000000.0f/LOOP_HZ));
}
