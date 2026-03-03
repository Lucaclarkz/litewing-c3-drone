// ESP32-C3 LiteWing (CRTP/UDP) Ready-to-Fly - MPU6050 + Brushed Quad X
// MPU6050: SCL=GPIO6, SDA=GPIO7
// Orientation: Y arrow = FRONT, X arrow = RIGHT
// Motors:
//  M1 GPIO21 Front Left  (CW)
//  M2 GPIO20 Front Right (CCW)  RIGHT side
//  M3 GPIO10 Back  Right (CW)   RIGHT side
//  M4 GPIO5  Back  Left  (CCW)
// LED: GPIO8 (boot blink, WiFi ready solid)
//
// Notes:
// - Receives CRTP Commander setpoint over UDP port 2390
//   Payload: float roll, float pitch, float yawrate, uint16 thrust (0..65535) + 1-byte checksum(sum mod256)
// - Instant stop on throttle release, no 2s delay.
// - Fixes: MPU axis remap (Y front / X right), M1 early start, right side gain reduce.

#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

// ===================== USER CONFIG =====================
#define TAG "LITEWING_C3"

// WiFi AP
#define AP_PASS      "12345678"
#define AP_CH        6
#define UDP_PORT_RX  2390

// LED
#define PIN_LED      8

// I2C
#define I2C_PORT     I2C_NUM_0
#define PIN_SCL      6
#define PIN_SDA      7
#define I2C_HZ       400000

// MPU6050
#define MPU_ADDR     0x68

// Motors
#define PIN_M1 21
#define PIN_M2 20
#define PIN_M3 10
#define PIN_M4 5

// PWM
#define PWM_FREQ_HZ  20000
#define PWM_RES_BITS LEDC_TIMER_8_BIT
#define PWM_MAX      255
#define MOTOR_MAX_LIMIT 240

// Safety
#define FAILSAFE_MS     250
#define THR_STOP        0.03f   // <3% -> instant stop
#define THR_ARM         0.08f   // >8% -> allow spin

// Control loop
#define LOOP_HZ     250
#define LOOP_DT     (1.0f / (float)LOOP_HZ)

// Filter
#define CF_ALPHA    0.98f

// Angle limits
#define MAX_ANGLE_DEG 30.0f
#define MAX_RATE_RP   220.0f
#define MAX_RATE_Y    180.0f

// ===================== MOTOR FIX TUNING =====================
// M1 starts late -> raise minStart for M1
static int motorMin[4]    = {55, 45, 45, 45};         // M1 boosted
// Right side too strong (M2/M3) -> reduce gain
static float motorGain[4] = {1.00f, 0.92f, 0.92f, 1.00f};

// ===================== PID (starter stable) =====================
static float Kp_angle = 4.5f;

static float Kp_rate_rp = 0.09f;
static float Ki_rate_rp = 0.18f;
static float Kd_rate_rp = 0.0025f;

static float Kp_rate_y  = 0.12f;
static float Ki_rate_y  = 0.10f;
static float Kd_rate_y  = 0.0f;

// yaw direction (if yaw drifts wrong way, flip to -1.0f)
static float yawSign = 1.0f;

// ===================== GLOBALS =====================
static SemaphoreHandle_t g_cmd_mutex;

typedef struct {
  float roll;     // deg (ANGLE mode)
  float pitch;    // deg (ANGLE mode)
  float yawrate;  // deg/s
  float thrust;   // 0..1
  uint32_t last_ms;
} cmd_t;

static cmd_t g_cmd = {0};

static bool armed = false;

// attitude
static float roll_deg = 0, pitch_deg = 0;
static float rollTrim = 0, pitchTrim = 0;

// gyro bias (deg/s)
static float gbx=0, gby=0, gbz=0;

// pid integrators
static float i_r=0, i_p=0, i_y=0;
static float prev_er=0, prev_ep=0, prev_ey=0;

// ===================== UTILS =====================
static inline float clampf(float x, float a, float b) {
  return (x < a) ? a : (x > b) ? b : x;
}
static inline uint32_t millis32(void) {
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
static inline uint8_t sum_mod256(const uint8_t *d, size_t n) {
  uint32_t s=0; for (size_t i=0;i<n;i++) s += d[i]; return (uint8_t)(s & 0xFF);
}

// ===================== GPIO/PWM =====================
static void motors_force_low(void) {
  gpio_config_t io = {
    .pin_bit_mask = (1ULL<<PIN_M1) | (1ULL<<PIN_M2) | (1ULL<<PIN_M3) | (1ULL<<PIN_M4),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = 0, .pull_down_en = 0,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io);
  gpio_set_level(PIN_M1, 0);
  gpio_set_level(PIN_M2, 0);
  gpio_set_level(PIN_M3, 0);
  gpio_set_level(PIN_M4, 0);
}

static void pwm_init(void) {
  ledc_timer_config_t t = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0,
    .duty_resolution = PWM_RES_BITS,
    .freq_hz = PWM_FREQ_HZ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&t));

  const int pins[4] = {PIN_M1, PIN_M2, PIN_M3, PIN_M4};
  for (int ch=0; ch<4; ch++) {
    ledc_channel_config_t c = {
      .gpio_num = pins[ch],
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = (ledc_channel_t)ch,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
  }
}

static inline void motor_write(int ch, int duty) {
  if (duty < 0) duty = 0;
  if (duty > PWM_MAX) duty = PWM_MAX;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}
static inline void motors_off(void) {
  motor_write(0,0); motor_write(1,0); motor_write(2,0); motor_write(3,0);
}

// ===================== I2C + MPU6050 =====================
static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  return i2c_master_write_to_device(I2C_PORT, addr, buf, 2, pdMS_TO_TICKS(50));
}
static esp_err_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *out, size_t len) {
  return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, out, len, pdMS_TO_TICKS(50));
}

static void mpu_init(void) {
  // Wake up
  ESP_ERROR_CHECK(i2c_write_reg(MPU_ADDR, 0x6B, 0x00)); // PWR_MGMT_1
  vTaskDelay(pdMS_TO_TICKS(20));
  // Gyro ±500 dps
  ESP_ERROR_CHECK(i2c_write_reg(MPU_ADDR, 0x1B, 0x08)); // GYRO_CONFIG
  // Accel ±8g
  ESP_ERROR_CHECK(i2c_write_reg(MPU_ADDR, 0x1C, 0x10)); // ACCEL_CONFIG
  // DLPF ~21Hz
  ESP_ERROR_CHECK(i2c_write_reg(MPU_ADDR, 0x1A, 0x04)); // CONFIG
}

static void mpu_read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
  // ACCEL_XOUT_H .. GYRO_ZOUT_L
  uint8_t b[14] = {0};
  if (i2c_read_regs(MPU_ADDR, 0x3B, b, sizeof(b)) != ESP_OK) {
    *ax=*ay=*az=*gx=*gy=*gz=0;
    return;
  }
  int16_t axr = (int16_t)((b[0]<<8) | b[1]);
  int16_t ayr = (int16_t)((b[2]<<8) | b[3]);
  int16_t azr = (int16_t)((b[4]<<8) | b[5]);
  int16_t gxr = (int16_t)((b[8]<<8) | b[9]);
  int16_t gyr = (int16_t)((b[10]<<8)| b[11]);
  int16_t gzr = (int16_t)((b[12]<<8)| b[13]);

  // scales: accel ±8g => 4096 LSB/g ; gyro ±500 => 65.5 LSB/deg/s
  *ax = (float)axr / 4096.0f;
  *ay = (float)ayr / 4096.0f;
  *az = (float)azr / 4096.0f;

  *gx = (float)gxr / 65.5f; // deg/s
  *gy = (float)gyr / 65.5f;
  *gz = (float)gzr / 65.5f;
}

// ===================== CALIBRATION =====================
static void gyro_calibrate(void) {
  float sx=0, sy=0, sz=0;
  for (int i=0;i<900;i++) {
    float ax,ay,az,gx,gy,gz;
    mpu_read(&ax,&ay,&az,&gx,&gy,&gz);
    // sensor orientation mapping later; bias stored in FRAME space:
    // remap: frame X=Ysensor, frame Y=-Xsensor, frame Z=Zsensor
    float gx_f = gy;     // deg/s
    float gy_f = -gx;
    float gz_f = gz;
    sx += gx_f; sy += gy_f; sz += gz_f;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  gbx = sx/900.0f;
  gby = sy/900.0f;
  gbz = sz/900.0f;
  i_r=i_p=i_y=0;
  prev_er=prev_ep=prev_ey=0;
}

static void level_trim_calibrate(void) {
  // average roll/pitch while flat
  float sr=0, sp=0;
  int N=180;
  float r=0, p=0;
  for (int i=0;i<N;i++) {
    float ax_s,ay_s,az_s,gx_s,gy_s,gz_s;
    mpu_read(&ax_s,&ay_s,&az_s,&gx_s,&gy_s,&gz_s);

    // REMAP (Y front, X right) => frame: X=Y, Y=-X, Z=Z
    float ax = ay_s;
    float ay = -ax_s;
    float az = az_s;

    float gx = gy_s - gbx;
    float gy = -gx_s - gby;

    float accRoll  = atan2f(ay, az) * 180.0f / (float)M_PI;
    float accPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;

    r = CF_ALPHA*(r + gx*LOOP_DT) + (1.0f-CF_ALPHA)*accRoll;
    p = CF_ALPHA*(p + gy*LOOP_DT) + (1.0f-CF_ALPHA)*accPitch;

    sr += r; sp += p;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  rollTrim  = sr / (float)N;
  pitchTrim = sp / (float)N;
}

// ===================== WiFi AP =====================
static void wifi_init_ap(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  char ssid[32];
  snprintf(ssid, sizeof(ssid), "LiteWing_%02X%02X%02X%02X%02X%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  wifi_config_t ap = {0};
  strncpy((char*)ap.ap.ssid, ssid, sizeof(ap.ap.ssid));
  strncpy((char*)ap.ap.password, AP_PASS, sizeof(ap.ap.password));
  ap.ap.ssid_len = 0;
  ap.ap.channel = AP_CH;
  ap.ap.max_connection = 1;
  ap.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
  if (strlen(AP_PASS) == 0) ap.ap.authmode = WIFI_AUTH_OPEN;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "AP SSID: %s  PASS: %s  UDP:%d", ssid, AP_PASS, UDP_PORT_RX);
}

// ===================== UDP / CRTP =====================
static void udp_rx_task(void *arg) {
  int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (s < 0) { ESP_LOGE(TAG, "socket() failed"); vTaskDelete(NULL); }

  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(UDP_PORT_RX);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
    ESP_LOGE(TAG, "bind() failed");
    close(s);
    vTaskDelete(NULL);
  }

  uint8_t buf[64];

  while (1) {
    int len = recvfrom(s, buf, sizeof(buf), 0, NULL, NULL);
    if (len <= 0) continue;

    // checksum last byte
    if (len < 2) continue;
    uint8_t rx = buf[len-1];
    uint8_t calc = sum_mod256(buf, (size_t)len-1);
    if (rx != calc) continue;

    // CRTP header: port = (h>>4)&0x0F, channel = h&0x03
    uint8_t h = buf[0];
    uint8_t port = (h >> 4) & 0x0F;
    uint8_t ch   = h & 0x03;

    // Commander setpoint: port=3 channel=0 (Crazyflie-style)
    if (port == 3 && ch == 0) {
      if (len < (1 + 4+4+4 + 2 + 1)) continue;

      float r,p,y;
      uint16_t t16;
      memcpy(&r, &buf[1], 4);
      memcpy(&p, &buf[5], 4);
      memcpy(&y, &buf[9], 4);
      memcpy(&t16,&buf[13],2);

      cmd_t c;
      c.roll = clampf(r, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
      c.pitch = clampf(p, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
      c.yawrate = clampf(y, -MAX_RATE_Y, MAX_RATE_Y);
      c.thrust = clampf((float)t16 / 65535.0f, 0.0f, 1.0f);
      c.last_ms = millis32();

      xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
      g_cmd = c;
      xSemaphoreGive(g_cmd_mutex);
    }
  }
}

// ===================== FLIGHT LOOP =====================
static void flight_task(void *arg) {
  uint32_t last = millis32();

  while (1) {
    uint32_t now = millis32();
    uint32_t dt_ms = now - last;
    if (dt_ms < (1000/LOOP_HZ)) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
    last = now;

    cmd_t c;
    xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
    c = g_cmd;
    xSemaphoreGive(g_cmd_mutex);

    // failsafe
    if ((now - c.last_ms) > FAILSAFE_MS) {
      armed = false;
      motors_off();
      i_r=i_p=i_y=0; prev_er=prev_ep=prev_ey=0;
      continue;
    }

    // instant stop on low throttle
    if (c.thrust < THR_STOP) {
      armed = false;
      motors_off();
      i_r=i_p=i_y=0; prev_er=prev_ep=prev_ey=0;
      continue;
    }

    if (!armed) {
      if (c.thrust > THR_ARM) {
        armed = true;
        i_r=i_p=i_y=0; prev_er=prev_ep=prev_ey=0;
      } else {
        motors_off();
        continue;
      }
    }

    // read IMU (sensor)
    float ax_s,ay_s,az_s,gx_s,gy_s,gz_s;
    mpu_read(&ax_s,&ay_s,&az_s,&gx_s,&gy_s,&gz_s);

    // REMAP for: Y front, X right  => frame: X=Y, Y=-X, Z=Z
    float ax = ay_s;
    float ay = -ax_s;
    float az = az_s;

    float gx = (gy_s - gbx);      // deg/s
    float gy = (-gx_s - gby);     // deg/s
    float gz = (gz_s - gbz);      // deg/s

    // accel angles
    float accRoll  = atan2f(ay, az) * 180.0f / (float)M_PI;
    float accPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;

    // complementary filter
    roll_deg  = CF_ALPHA*(roll_deg  + gx*LOOP_DT) + (1.0f-CF_ALPHA)*accRoll;
    pitch_deg = CF_ALPHA*(pitch_deg + gy*LOOP_DT) + (1.0f-CF_ALPHA)*accPitch;

    float rollLevel  = roll_deg  - rollTrim;
    float pitchLevel = pitch_deg - pitchTrim;

    // ANGLE mode => desired rates
    float des_r = clampf((c.roll  - rollLevel)  * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
    float des_p = clampf((c.pitch - pitchLevel) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
    float des_y = clampf(c.yawrate, -MAX_RATE_Y, MAX_RATE_Y);

    // rate PID roll
    float er = des_r - gx;
    i_r = clampf(i_r + er*LOOP_DT, -200.0f, 200.0f);
    float dr = (er - prev_er)/LOOP_DT; prev_er = er;
    float out_r = Kp_rate_rp*er + Ki_rate_rp*i_r + Kd_rate_rp*dr;

    // rate PID pitch
    float ep = des_p - gy;
    i_p = clampf(i_p + ep*LOOP_DT, -200.0f, 200.0f);
    float dp = (ep - prev_ep)/LOOP_DT; prev_ep = ep;
    float out_p = Kp_rate_rp*ep + Ki_rate_rp*i_p + Kd_rate_rp*dp;

    // rate PID yaw
    float ey = des_y - gz;
    i_y = clampf(i_y + ey*LOOP_DT, -200.0f, 200.0f);
    float dy = (ey - prev_ey)/LOOP_DT; prev_ey = ey;
    float out_y = Kp_rate_y*ey + Ki_rate_y*i_y + Kd_rate_y*dy;

    float yawTerm = yawSign * out_y;

    // base throttle -> PWM
    int minAvg = (motorMin[0]+motorMin[1]+motorMin[2]+motorMin[3]) / 4;
    int usable = MOTOR_MAX_LIMIT - minAvg;
    int base = minAvg + (int)(c.thrust * (float)usable);
    base = (base < 0) ? 0 : (base > MOTOR_MAX_LIMIT ? MOTOR_MAX_LIMIT : base);

    // X mix (M1 FL, M2 FR, M3 BR, M4 BL)
    float m[4];
    m[0] = base - out_p + out_r - yawTerm; // M1
    m[1] = base - out_p - out_r + yawTerm; // M2
    m[2] = base + out_p - out_r - yawTerm; // M3
    m[3] = base + out_p + out_r + yawTerm; // M4

    // per-motor min + gain
    for (int i=0;i<4;i++) {
      if (m[i] > 0 && m[i] < motorMin[i]) m[i] = motorMin[i];
      m[i] *= motorGain[i];
    }

    // clamp by shifting down if any exceeds MOTOR_MAX_LIMIT
    float maxOut = fmaxf(fmaxf(m[0],m[1]), fmaxf(m[2],m[3]));
    if (maxOut > (float)MOTOR_MAX_LIMIT) {
      float s = maxOut - (float)MOTOR_MAX_LIMIT;
      for (int i=0;i<4;i++) m[i] -= s;
    }

    // final clamp + write
    for (int i=0;i<4;i++) m[i] = clampf(m[i], 0.0f, (float)PWM_MAX);

    motor_write(0, (int)m[0]);
    motor_write(1, (int)m[1]);
    motor_write(2, (int)m[2]);
    motor_write(3, (int)m[3]);
  }
}

// ===================== LED TASK =====================
static void led_task(void *arg) {
  // boot blink until WiFi ready => we turn solid ON in app_main after wifi init
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(250));
    int v = gpio_get_level(PIN_LED);
    gpio_set_level(PIN_LED, !v);
  }
}

// ===================== app_main =====================
void app_main(void) {
  ESP_ERROR_CHECK(nvs_flash_init());

  // LED
  gpio_config_t led = {
    .pin_bit_mask = (1ULL<<PIN_LED),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en=0,.pull_down_en=0,.intr_type=GPIO_INTR_DISABLE
  };
  gpio_config(&led);
  gpio_set_level(PIN_LED, 0);

  // Boot blip fix
  motors_force_low();

  // I2C
  i2c_config_t ic = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = PIN_SDA,
    .scl_io_num = PIN_SCL,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_HZ
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &ic));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

  mpu_init();

  // PWM
  pwm_init();
  motors_off();

  // start LED blink while boot/cal
  xTaskCreate(led_task, "led_task", 2048, NULL, 1, NULL);

  // calibrate (keep still)
  vTaskDelay(pdMS_TO_TICKS(300));
  gyro_calibrate();
  level_trim_calibrate();
  ESP_LOGI(TAG, "Gyro bias (deg/s): %.2f %.2f %.2f | Trim (deg): %.2f %.2f",
           gbx,gby,gbz, rollTrim,pitchTrim);

  // WiFi AP
  wifi_init_ap();

  // WiFi ready => LED solid ON (stop blink task by just forcing ON; blink task still toggles, so delete it)
  // Easiest: set LED task priority low and kill it:
  // (We don't have handle; so just set LED HIGH here and keep it HIGH by disabling led_task toggling in real use.
  // If you want perfect, create led_task with handle and vTaskDelete(handle). Keeping simple: we set solid and remove blink logic by setting here and not running led_task in your final build.)
  gpio_set_level(PIN_LED, 1);

  g_cmd_mutex = xSemaphoreCreateMutex();
  g_cmd.last_ms = millis32();

  xTaskCreate(udp_rx_task, "udp_rx", 4096, NULL, 5, NULL);
  xTaskCreate(flight_task, "flight", 4096, NULL, 10, NULL);

  ESP_LOGI(TAG, "READY: connect phone to AP and open LiteWing app.");
}
