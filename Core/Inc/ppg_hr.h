// ppg_hr.h
#ifndef PPG_HR_H
#define PPG_HR_H

#include <stdint.h>
#include <stdbool.h>

/* ===== Config ===== */
#define HRV_IBI_BUF_N  20   // 20 beats window

/* Respiration limits (model-safe) */
#define RESP_MIN_BPM           6.0f
#define RESP_MAX_BPM           30.0f
#define RESP_REFRACTORY_MS     3000u
#define RESP_WARMUP_PEAKS      2u
#define RESP_SPIKE_REJECT_BPM  6

/* amplitude gate */
#define RESP_AMP_FRACTION_GATE 0.20f
#define RESP_AMP_FLOOR_GATE    100.0f

typedef struct {
    uint16_t fs_hz;

    /* ===== HR (peak detect) ===== */
    int32_t dc;
    int32_t ac_prev2;
    int32_t ac_prev1;

    int32_t env;
    int32_t thresh;

    uint32_t last_peak_ms;
    uint32_t last_beat_ms;

    float bpm;
    uint8_t bpm_valid;

    /* ===== HRV (RMSSD) ===== */
    uint16_t ibi_ms[HRV_IBI_BUF_N];
    uint8_t  ibi_n;
    uint8_t  ibi_w;
    float    rmssd_ms;
    uint8_t  rmssd_valid;

    /* ===== Signal quality gate ===== */
    uint8_t  signal_ok;

    /* ===== Respiration (from PPG envelope) ===== */
    float    resp_lp;
    float    resp_prev2;
    float    resp_prev1;
    uint32_t resp_last_peak_ms;
    float    resp_bpm;
    uint8_t  resp_valid;

    uint8_t  resp_peaks_good;

    float    resp_amp_floor;
    float    resp_amp_frac;

} HR_State;

/* ===== APIs ===== */
void HR_Init(HR_State *st, uint16_t fs_hz);
void HR_SetSignalOK(HR_State *st, bool ok);
bool HR_ProcessSample(HR_State *st, uint32_t ir_raw, uint32_t t_ms);

/* HR */
bool  HR_HasValid(const HR_State *st);
float HR_GetBPM(const HR_State *st);

/* HRV */
bool  HRV_HasValid(const HR_State *st);
float HRV_GetRMSSD(const HR_State *st);

/* Respiration */
bool  RESP_HasValid(const HR_State *st);
float RESP_GetBPM(const HR_State *st);

#endif
