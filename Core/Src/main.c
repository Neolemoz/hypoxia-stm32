/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "ppg_hr.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* === BMP280/BME280 === */
#define BMP280_ADDR            0x76
#define BMP280_REG_ID          0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB       0x88

/* === MAX30102 === */
#define MAX30102_ADDR               0x57

#define MAX30102_REG_INTR_STATUS_1  0x00
#define MAX30102_REG_INTR_STATUS_2  0x01
#define MAX30102_REG_INTR_ENABLE_1  0x02
#define MAX30102_REG_INTR_ENABLE_2  0x03

#define MAX30102_REG_FIFO_WR_PTR    0x04
#define MAX30102_REG_FIFO_OVF_CNT   0x05
#define MAX30102_REG_FIFO_RD_PTR    0x06
#define MAX30102_REG_FIFO_DATA      0x07

#define MAX30102_REG_FIFO_CONFIG    0x08
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_SPO2_CONFIG    0x0A
#define MAX30102_REG_LED1_PA        0x0C   // RED
#define MAX30102_REG_LED2_PA        0x0D   // IR

#define MAX30102_MODE_RESET         0x40
#define MAX30102_MODE_SPO2          0x03

/* Tuning */
#define IR_FINGER_THRESHOLD         3000.0f
#define SIGNAL_LOST_DEBOUNCE_MS     7000u

/* I2C fail streak -> force SIGNAL_LOST */
#define I2C_FAIL_STREAK_FORCE_LOST  3u

/* === SpO2 (ratio-of-ratios) tuning === */
#define SPO2_A                      110.0f
#define SPO2_B                      25.0f
#define SPO2_MIN                    50.0f
#define SPO2_MAX                    100.0f
#define SPO2_SPIKE_REJECT_PCT       6.0f     // max change per 1s frame
#define SPO2_MIN_DC                 1000.0f  // กันหารศูนย์/สัญญาณไม่พอ
#define SPO2_MIN_AC                 50.0f    // กัน noise-only (หน่วย ADC)
#define SPO2_FAIL_STREAK_FORCE_LOST 3u       // Mode B: fail ต่อเนื่องกี่เฟรมถึง force SIGNAL_LOST

/* OLED refresh rate (กันกระพริบ) */
#define OLED_UPDATE_MS              250u
/* USER CODE END PD */

void SystemClock_Config(void);

/* USER CODE BEGIN PV */
/* BMP calib */
static uint16_t dig_T1; static int16_t dig_T2, dig_T3;
static uint16_t dig_P1; static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine = 0;

static HR_State g_hr;

static bool bmp_ok = false;
static bool max_ok = false;

/* Debounce / hold last-known-good */
static uint32_t last_good_ms = 0;
static float last_alt = 1.0f;
static float last_spo2 = 98.0f;
static int last_hr = 75, last_resp = 16, last_hrv = 35;

/* I2C error streak (frame-level) */
static uint32_t i2c_fail_streak = 0;

/* SpO2 quality fail streak (Mode B) */
static uint32_t spo2_fail_streak = 0;

/* OLED */
static uint32_t next_oled_ms = 0;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
static uint8_t  i2c_read_u8(uint8_t dev7, uint8_t reg);
static int      i2c_read_buf(uint8_t dev7, uint8_t reg, uint8_t *buf, uint16_t len);
static int      i2c_write_u8(uint8_t dev7, uint8_t reg, uint8_t val);

/* BMP */
static uint16_t u16le(const uint8_t *p);
static int16_t  s16le(const uint8_t *p);
static int      bmp280_read_calib(void);
static int      bmp280_init(void);
static int32_t  bmp280_compensate_T_int32(int32_t adc_T);
static uint32_t bmp280_compensate_P_uint32(int32_t adc_P);
static int      bmp280_read_pressure_pa(float *pressure_pa);
static float    pressure_to_altitude_m(float pressure_pa);

/* MAX */
static int      max30102_init(void);
static int      max30102_read_sample(uint32_t *red, uint32_t *ir);

/* OLED helpers */
static void OLED_ShowBoot(void);
static void OLED_ShowData(uint32_t ts, bool lost);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static uint8_t i2c_read_u8(uint8_t dev7, uint8_t reg)
{
  uint8_t val = 0xFF;
  if (HAL_I2C_Mem_Read(&hi2c1, (dev7 << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) != HAL_OK)
    return 0xFF;
  return val;
}

static int i2c_read_buf(uint8_t dev7, uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(&hi2c1, (dev7 << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 200) == HAL_OK);
}

static int i2c_write_u8(uint8_t dev7, uint8_t reg, uint8_t val)
{
  return (HAL_I2C_Mem_Write(&hi2c1, (dev7 << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK);
}

/* ===== BMP280/BME280 ===== */
static uint16_t u16le(const uint8_t *p){ return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static int16_t  s16le(const uint8_t *p){ return (int16_t)u16le(p); }

static int bmp280_read_calib(void)
{
  uint8_t c[24];
  if (!i2c_read_buf(BMP280_ADDR, BMP280_REG_CALIB, c, 24)) return 0;

  dig_T1 = u16le(&c[0]);  dig_T2 = s16le(&c[2]);  dig_T3 = s16le(&c[4]);
  dig_P1 = u16le(&c[6]);  dig_P2 = s16le(&c[8]);  dig_P3 = s16le(&c[10]);
  dig_P4 = s16le(&c[12]); dig_P5 = s16le(&c[14]); dig_P6 = s16le(&c[16]);
  dig_P7 = s16le(&c[18]); dig_P8 = s16le(&c[20]); dig_P9 = s16le(&c[22]);

  return (dig_P1 != 0);
}

static int bmp280_init(void)
{
  uint8_t id = i2c_read_u8(BMP280_ADDR, BMP280_REG_ID);
  if (id != 0x58 && id != 0x60) return 0;

  (void)i2c_write_u8(BMP280_ADDR, BMP280_REG_RESET, 0xB6);
  HAL_Delay(10);

  if (!bmp280_read_calib()) return 0;

  (void)i2c_write_u8(BMP280_ADDR, BMP280_REG_CONFIG, (5 << 5) | (0 << 2) | 0);
  (void)i2c_write_u8(BMP280_ADDR, BMP280_REG_CTRL_MEAS, (1 << 5) | (1 << 2) | 3);
  HAL_Delay(10);

  return 1;
}

static int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

static uint32_t bmp280_compensate_P_uint32(int32_t adc_P)
{
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1*(int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0) return 0;

  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (uint32_t)p;
}

static int bmp280_read_pressure_pa(float *pressure_pa)
{
  uint8_t d[6];
  if (!i2c_read_buf(BMP280_ADDR, BMP280_REG_PRESS_MSB, d, 6)) return 0;

  int32_t adc_P = (int32_t)((((uint32_t)d[0]) << 12) | (((uint32_t)d[1]) << 4) | (d[2] >> 4));
  int32_t adc_T = (int32_t)((((uint32_t)d[3]) << 12) | (((uint32_t)d[4]) << 4) | (d[5] >> 4));

  (void)bmp280_compensate_T_int32(adc_T);
  uint32_t p_q24_8 = bmp280_compensate_P_uint32(adc_P);
  if (p_q24_8 == 0) return 0;

  *pressure_pa = (float)p_q24_8 / 256.0f;
  return 1;
}

static float pressure_to_altitude_m(float pressure_pa)
{
  const float p0 = 101325.0f;
  float ratio = pressure_pa / p0;
  float alt = 44330.0f * (1.0f - powf(ratio, 0.1903f));
  if (!isfinite(alt) || alt == 0.0f) alt = 1.0f;
  return alt;
}

/* ===== MAX30102 ===== */
static int max30102_init(void)
{
  if (!i2c_write_u8(MAX30102_ADDR, MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET))
    return 0;
  HAL_Delay(30);

  (void)i2c_read_u8(MAX30102_ADDR, MAX30102_REG_INTR_STATUS_1);
  (void)i2c_read_u8(MAX30102_ADDR, MAX30102_REG_INTR_STATUS_2);

  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_INTR_ENABLE_1, 0b11000000);
  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_INTR_ENABLE_2, 0x00);

  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_FIFO_CONFIG, (0b010 << 5) | (0 << 4) | 0x0F);

  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_SPO2_CONFIG,
                     (0b01 << 5) | (0b011 << 2) | 0b11);

  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_LED1_PA, 0x24);
  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_LED2_PA, 0x24);

  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SPO2);

  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_FIFO_WR_PTR, 0x00);
  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_FIFO_OVF_CNT, 0x00);
  (void)i2c_write_u8(MAX30102_ADDR, MAX30102_REG_FIFO_RD_PTR, 0x00);

  return 1;
}

static int max30102_read_sample(uint32_t *red, uint32_t *ir)
{
  uint8_t d[6];
  if (!i2c_read_buf(MAX30102_ADDR, MAX30102_REG_FIFO_DATA, d, 6))
    return 0;

  uint32_t r = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
  uint32_t i = ((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5];
  r &= 0x3FFFF;
  i &= 0x3FFFF;

  *red = r;
  *ir  = i;
  return 1;
}

/* ===== OLED ===== */
static void OLED_ShowBoot(void)
{
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("hypoxia_node_f411", Font_6x8, White);

  ssd1306_SetCursor(0, 12);
  ssd1306_WriteString(bmp_ok ? "BMP: OK" : "BMP: FAIL", Font_7x10, White);

  ssd1306_SetCursor(0, 28);
  ssd1306_WriteString(max_ok ? "MAX: OK" : "MAX: FAIL", Font_7x10, White);

  ssd1306_UpdateScreen();
}

static void OLED_ShowData(uint32_t ts, bool lost)
{
  // update ทุก 250ms กันกระพริบ/หนัก I2C
  uint32_t now = HAL_GetTick();
  if ((int32_t)(now - next_oled_ms) < 0) return;
  next_oled_ms = now + OLED_UPDATE_MS;

  char line[32];

  ssd1306_Fill(Black);

  // Line 1: status + ts
  ssd1306_SetCursor(0, 0);
  if (lost) {
    ssd1306_WriteString("SIGNAL LOST", Font_7x10, White);
  } else {
    snprintf(line, sizeof(line), "TS:%lu", (unsigned long)ts);
    ssd1306_WriteString(line, Font_7x10, White);
  }

  // Line 2: SpO2 + HR
  ssd1306_SetCursor(0, 14);
  snprintf(line, sizeof(line), "SpO2:%4.1f", last_spo2);
  ssd1306_WriteString(line, Font_7x10, White);

  // ✅ HR ใช้ตัวเล็กลงกันตกขอบ (เดิมใช้ Font_7x10)
  ssd1306_SetCursor(80, 16); // +2px ให้บาลานซ์กับ 6x8
  snprintf(line, sizeof(line), "HR:%3d", last_hr);
  ssd1306_WriteString(line, Font_6x8, White);

  // Line 3: Resp + HRV
  ssd1306_SetCursor(0, 30);
  snprintf(line, sizeof(line), "Resp:%2d", last_resp);
  ssd1306_WriteString(line, Font_7x10, White);

  // ✅ HRV ใช้ตัวเล็กลงกันตกขอบ (สำคัญ)
  ssd1306_SetCursor(80, 32); // +2px ให้บาลานซ์กับ 6x8
  snprintf(line, sizeof(line), "HRV:%3d", last_hrv);
  ssd1306_WriteString(line, Font_6x8, White);

  // Line 4: Alt
  ssd1306_SetCursor(0, 46);
  snprintf(line, sizeof(line), "Alt:%6.1f m", last_alt);
  ssd1306_WriteString(line, Font_7x10, White);

  ssd1306_UpdateScreen();
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();

  bmp_ok = (bmp280_init() == 1);
  max_ok = (max30102_init() == 1);

  ssd1306_Init();
  OLED_ShowBoot();
  HAL_Delay(800);

  printf("BMP init: %s (ID=0x%02X)\r\n", bmp_ok ? "OK" : "FAIL",
         i2c_read_u8(BMP280_ADDR, BMP280_REG_ID));

  printf("MAX init: %s (PART=0x%02X REV=0x%02X)\r\n", max_ok ? "OK" : "FAIL",
         i2c_read_u8(MAX30102_ADDR, 0xFF),
         i2c_read_u8(MAX30102_ADDR, 0xFE));

  printf("FW=1.0,FS=1HZ,FORMAT=ts,spo2,hr,resp,hrv,alt\r\n");

  HR_Init(&g_hr, 100);

  last_good_ms = 0;

  static uint32_t next_frame_ms = 0;
  next_frame_ms = HAL_GetTick();

  next_oled_ms = HAL_GetTick() + 50;

  while (1)
  {
    while ((int32_t)(HAL_GetTick() - next_frame_ms) < 0) { }
    uint32_t loop_start = next_frame_ms;
    next_frame_ms += 1000;

    uint32_t now = loop_start;

    static uint32_t ts = 0;
    ts++;

    bool frame_i2c_ok = true;

    /* ---- BMP pressure -> altitude ---- */
    float pressure_pa = 0.0f;
    float altitude = 1.0f;

    bool bmp_read_ok = false;
    if (bmp_ok) {
      bmp_read_ok = bmp280_read_pressure_pa(&pressure_pa);
      if (!bmp_read_ok) frame_i2c_ok = false;
    } else {
      frame_i2c_ok = false;
    }

    bool bmp_valid = bmp_ok && bmp_read_ok &&
                     (pressure_pa >= 30000.0f && pressure_pa <= 110000.0f);

    if (bmp_valid)
      altitude = pressure_to_altitude_m(pressure_pa);

    /* ---- MAX finger detect + SpO2 window stats ---- */
    bool max_valid = false;
    bool spo2_frame_ok = false;

    if (max_ok)
    {
      uint32_t red_sum = 0, ir_sum = 0;
      uint64_t red_sumsq = 0, ir_sumsq = 0;
      uint32_t red = 0, ir = 0;
      int n = 0;
      int read_fail = 0;

      /* ===== NEW: buffer IR samples + timestamps (กัน signal_ok กระตุกแล้ว reset HRV/Resp) ===== */
      uint32_t ir_buf[90];
      uint32_t t_buf[90];
      uint8_t  buf_n = 0;

      for (int k = 0; k < 90; k++)
      {
        if (max30102_read_sample(&red, &ir))
        {
          red_sum += red;
          ir_sum  += ir;
          red_sumsq += (uint64_t)red * (uint64_t)red;
          ir_sumsq  += (uint64_t)ir  * (uint64_t)ir;
          n++;

          if (buf_n < 90) {
            ir_buf[buf_n] = ir;
            t_buf[buf_n]  = HAL_GetTick();
            buf_n++;
          }
        }
        else
        {
          read_fail++;
        }

        HAL_Delay(10);
      }

      if (read_fail > 20) {
        frame_i2c_ok = false;
      }

      bool finger_ok = false;
      float red_dc = 0.0f, ir_dc = 0.0f;
      float red_ac = 0.0f, ir_ac = 0.0f;

      if (n >= 10)
      {
        red_dc = (float)red_sum / (float)n;
        ir_dc  = (float)ir_sum  / (float)n;

        if (isfinite(ir_dc) && ir_dc >= IR_FINGER_THRESHOLD) {
          finger_ok = true;
          max_valid = true;
        }

        float red_ex2 = (float)red_sumsq / (float)n;
        float ir_ex2  = (float)ir_sumsq  / (float)n;

        float red_var = red_ex2 - (red_dc * red_dc);
        float ir_var  = ir_ex2  - (ir_dc  * ir_dc);

        if (red_var < 0.0f) red_var = 0.0f;
        if (ir_var  < 0.0f) ir_var  = 0.0f;

        red_ac = sqrtf(red_var);
        ir_ac  = sqrtf(ir_var);

        bool spo2_new_valid = true;
        if (!isfinite(red_dc) || !isfinite(ir_dc) || !isfinite(red_ac) || !isfinite(ir_ac)) spo2_new_valid = false;
        if (red_dc < SPO2_MIN_DC || ir_dc < SPO2_MIN_DC) spo2_new_valid = false;
        if (red_ac < SPO2_MIN_AC || ir_ac < SPO2_MIN_AC) spo2_new_valid = false;
        if (!max_valid) spo2_new_valid = false;

        if (spo2_new_valid)
        {
          float r_red = red_ac / red_dc;
          float r_ir  = ir_ac  / ir_dc;

          if (!isfinite(r_red) || !isfinite(r_ir) || r_ir <= 0.0f) {
            spo2_new_valid = false;
          } else {
            float R = r_red / r_ir;
            if (!isfinite(R) || R <= 0.0f) {
              spo2_new_valid = false;
            } else {
              float spo2 = SPO2_A - (SPO2_B * R);

              if (spo2 < SPO2_MIN) spo2 = SPO2_MIN;
              if (spo2 > SPO2_MAX) spo2 = SPO2_MAX;

              float diff = spo2 - last_spo2;
              if (diff < 0) diff = -diff;

              if (diff <= SPO2_SPIKE_REJECT_PCT) {
                last_spo2 = spo2;
              }
              spo2_frame_ok = true;
            }
          }
        }
      }

      /* ===== NEW: set signal_ok ONCE per frame ===== */
      bool stream_ok = (finger_ok && frame_i2c_ok && (read_fail <= 20));
      HR_SetSignalOK(&g_hr, stream_ok);

      /* ===== NEW: feed HR/HRV/Resp ONLY when stream_ok ===== */
      if (stream_ok)
      {
        for (uint8_t k = 0; k < buf_n; k++) {
          HR_ProcessSample(&g_hr, (uint32_t)ir_buf[k], (uint32_t)t_buf[k]);
        }
      }

      if (max_valid && spo2_frame_ok) {
        spo2_fail_streak = 0;
      } else if (max_valid) {
        spo2_fail_streak++;
      } else {
        spo2_fail_streak = 0;
      }
    }
    else
    {
      frame_i2c_ok = false;
      spo2_fail_streak = 0;
      HR_SetSignalOK(&g_hr, false);
    }

    if (frame_i2c_ok) {
      i2c_fail_streak = 0;
    } else {
      i2c_fail_streak++;
    }

    now = HAL_GetTick();

    bool valid_frame = (bmp_valid && max_valid && (i2c_fail_streak == 0) &&
                        (spo2_fail_streak < SPO2_FAIL_STREAK_FORCE_LOST));

    if (valid_frame)
    {
      last_good_ms = now;
      last_alt = (isfinite(altitude) && altitude != 0.0f) ? altitude : 1.0f;

      if (HR_HasValid(&g_hr)) {
        float bpm = HR_GetBPM(&g_hr);
        if (bpm < 30.0f) bpm = 30.0f;
        if (bpm > 220.0f) bpm = 220.0f;

        int new_hr = (int)(bpm + 0.5f);
        if (new_hr < 30) new_hr = 30;
        if (new_hr > 220) new_hr = 220;

        int diff = new_hr - last_hr;
        if (diff < 0) diff = -diff;

        if (diff <= 15) last_hr = new_hr;
      }

      if (RESP_HasValid(&g_hr)) {
        float rbpm = RESP_GetBPM(&g_hr);

        if (!isfinite(rbpm) || rbpm < RESP_MIN_BPM) rbpm = RESP_MIN_BPM;
        if (rbpm > RESP_MAX_BPM) rbpm = RESP_MAX_BPM;

        int new_resp = (int)(rbpm + 0.5f);
        if (new_resp < (int)RESP_MIN_BPM) new_resp = (int)RESP_MIN_BPM;
        if (new_resp > (int)RESP_MAX_BPM) new_resp = (int)RESP_MAX_BPM;

        int rdiff = new_resp - last_resp;
        if (rdiff < 0) rdiff = -rdiff;

        if (rdiff <= RESP_SPIKE_REJECT_BPM) last_resp = new_resp;
      }

      if (HRV_HasValid(&g_hr)) {
        float rmssd = HRV_GetRMSSD(&g_hr);

        if (!isfinite(rmssd) || rmssd < 1.0f) rmssd = 1.0f;
        if (rmssd > 300.0f) rmssd = 300.0f;

        last_hrv = (int)(rmssd + 0.5f);
        if (last_hrv < 1) last_hrv = 1;
      }

      printf("%lu,%.1f,%d,%d,%d,%.1f\r\n",
             (unsigned long)ts, last_spo2, last_hr, last_resp, last_hrv, last_alt);

      OLED_ShowData(ts, false);
    }
    else
    {
      bool force_lost = (i2c_fail_streak >= I2C_FAIL_STREAK_FORCE_LOST) ||
                        (spo2_fail_streak >= SPO2_FAIL_STREAK_FORCE_LOST);

      bool timeout_lost = (last_good_ms == 0) ||
                          ((now - last_good_ms) >= SIGNAL_LOST_DEBOUNCE_MS);

      if (force_lost || timeout_lost)
      {
        printf("SIGNAL_LOST\r\n");
        OLED_ShowData(ts, true);
      }
      else
      {
        printf("%lu,%.1f,%d,%d,%d,%.1f\r\n",
               (unsigned long)ts, last_spo2, last_hr, last_resp, last_hrv, last_alt);

        OLED_ShowData(ts, false);
      }
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
