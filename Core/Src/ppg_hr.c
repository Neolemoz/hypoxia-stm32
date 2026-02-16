// ppg_hr.c
#include "ppg_hr.h"
#include <math.h>

static inline int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

static uint16_t ibi_get_at(const HR_State *st, uint8_t idx)
{
    if (!st) return 0;
    return st->ibi_ms[idx % HRV_IBI_BUF_N];
}

// median ของ IBI ล่าสุด (เอา 5 ค่า ถ้ามีน้อยกว่าใช้เท่าที่มี)
static uint16_t ibi_median_last5(const HR_State *st)
{
    if (!st || st->ibi_n == 0) return 0;

    uint8_t n = st->ibi_n;
    if (n > 5) n = 5;

    uint16_t v[5] = {0,0,0,0,0};

    // start = index ของตัวเก่าสุดในชุดล่าสุด n ค่า
    uint8_t start = (uint8_t)((st->ibi_w + HRV_IBI_BUF_N - n) % HRV_IBI_BUF_N);
    for (uint8_t k = 0; k < n; k++) {
        v[k] = ibi_get_at(st, (uint8_t)(start + k));
    }

    // sort เล็กๆ (insertion sort)
    for (uint8_t i = 1; i < n; i++) {
        uint16_t key = v[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && v[j] > key) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = key;
    }

    return v[n / 2];
}

void HR_Init(HR_State *st, uint16_t fs_hz)
{
    st->fs_hz = fs_hz;

    st->dc = 0;
    st->ac_prev2 = 0;
    st->ac_prev1 = 0;

    st->env = 0;
    st->thresh = 0;

    st->last_peak_ms = 0;
    st->last_beat_ms = 0;

    st->bpm = 75.0f;   // default non-zero
    st->bpm_valid = 0;

    // ---- HRV init ----
    st->ibi_n = 0;
    st->ibi_w = 0;
    for (uint8_t i = 0; i < HRV_IBI_BUF_N; i++) st->ibi_ms[i] = 0;
    st->rmssd_ms = 35.0f;   // non-zero default
    st->rmssd_valid = 0;

    // ---- signal gate (main.c sets this) ----
    st->signal_ok = 0;

    // ---- Resp init ----
    st->resp_lp = 0.0f;
    st->resp_prev2 = 0.0f;
    st->resp_prev1 = 0.0f;
    st->resp_last_peak_ms = 0;
    st->resp_bpm = 16.0f;   // default non-zero
    st->resp_valid = 0;

    st->resp_peaks_good = 0;

    // runtime gates
    st->resp_amp_floor = RESP_AMP_FLOOR_GATE;
    st->resp_amp_frac  = RESP_AMP_FRACTION_GATE;
}

void HR_SetSignalOK(HR_State *st, bool ok)
{
    if (!st) return;

    uint8_t new_ok = ok ? 1u : 0u;
    if (st->signal_ok == new_ok) {
        st->signal_ok = new_ok;
        return;
    }

    st->signal_ok = new_ok;

    if (!new_ok) {
        // ---- Resp ----
        st->resp_peaks_good = 0;
        st->resp_valid = 0;
        st->resp_last_peak_ms = 0;

        // ---- HRV ----
        st->ibi_n = 0;
        st->ibi_w = 0;
        for (uint8_t i = 0; i < HRV_IBI_BUF_N; i++) st->ibi_ms[i] = 0;
        st->rmssd_valid = 0;

        // ---- HR timing ----
        st->last_peak_ms = 0;
        st->last_beat_ms = 0;
        st->bpm_valid = 0;  // main.c จะคง last_hr เอง
    }
}

bool HR_HasValid(const HR_State *st)
{
    return st && (st->bpm_valid != 0);
}

float HR_GetBPM(const HR_State *st)
{
    return st ? st->bpm : 0.0f;
}

// ---- HRV APIs ----
bool HRV_HasValid(const HR_State *st)
{
    return st && (st->rmssd_valid != 0);
}

float HRV_GetRMSSD(const HR_State *st)
{
    return st ? st->rmssd_ms : 0.0f;
}

// ---- Resp APIs ----
bool RESP_HasValid(const HR_State *st)
{
    return st && (st->resp_valid != 0);
}

float RESP_GetBPM(const HR_State *st)
{
    return st ? st->resp_bpm : 0.0f;
}

bool HR_ProcessSample(HR_State *st, uint32_t ir_raw, uint32_t t_ms)
{
    int32_t x = (int32_t)ir_raw;

    // ---- DC removal (IIR) ----
    if (st->dc == 0) {
        st->dc = x;
    } else {
        st->dc += (x - st->dc) >> 5; // /32
    }

    int32_t ac = x - st->dc;

    // ---- Envelope (for HR + Resp gates) ----
    int32_t a = iabs32(ac);
    st->env += (a - st->env) >> 4; // /16

    int32_t thr = (st->env * 3) / 5;
    if (thr < 150) thr = 150;
    st->thresh = thr;

    // ==============================
    // Respiration from envelope (model-safe)
    // ==============================
    if (st->signal_ok)
    {
        float abs_ac = (float)iabs32(ac);

        // heavy low-pass
        st->resp_lp += (abs_ac - st->resp_lp) * 0.015f;

        float gate_floor = st->resp_amp_floor;
        float gate_frac  = st->resp_amp_frac;
        float gate_env   = (float)st->env * gate_frac;
        float gate_thr   = (gate_env > gate_floor) ? gate_env : gate_floor;

        if ((st->resp_prev2 < st->resp_prev1) &&
            (st->resp_prev1 > st->resp_lp) &&
            (st->resp_prev1 > gate_thr))
        {
            uint32_t now_ms = t_ms;

            if (st->resp_last_peak_ms == 0 ||
                (now_ms - st->resp_last_peak_ms) > RESP_REFRACTORY_MS)
            {
                if (st->resp_last_peak_ms != 0)
                {
                    uint32_t interval = now_ms - st->resp_last_peak_ms;

                    if (interval >= 1500U && interval <= 10000U)
                    {
                        float rbpm = 60000.0f / (float)interval;

                        if (rbpm < RESP_MIN_BPM) rbpm = RESP_MIN_BPM;
                        if (rbpm > RESP_MAX_BPM) rbpm = RESP_MAX_BPM;

                        st->resp_bpm = (0.8f * st->resp_bpm) + (0.2f * rbpm);

                        if (st->resp_peaks_good < 255u) st->resp_peaks_good++;
                        if (st->resp_peaks_good >= RESP_WARMUP_PEAKS) {
                            st->resp_valid = 1;
                        }
                    }
                }

                st->resp_last_peak_ms = now_ms;
            }
        }

        st->resp_prev2 = st->resp_prev1;
        st->resp_prev1 = st->resp_lp;
    }

    bool beat = false;

    // ==============================
    // HR / HRV (update เฉพาะเมื่อ signal_ok)
    // ==============================
    if (st->signal_ok)
    {
        if ((st->ac_prev2 < st->ac_prev1) &&
            (st->ac_prev1 > ac) &&
            (st->ac_prev1 > st->thresh))
        {
            if (st->last_peak_ms == 0 || (t_ms - st->last_peak_ms) > 330U)
            {
                uint32_t peak_ms = t_ms;

                if (st->last_peak_ms != 0)
                {
                    uint32_t ibi = peak_ms - st->last_peak_ms;

                    if (ibi >= 350U && ibi <= 2000U)
                    {
                        // ---- IBI median gate (กัน false/skip beat) ----
                        bool accept_ibi = true;
                        uint16_t med = ibi_median_last5(st);

                        if (med != 0) {
                            // ยอมให้แกว่ง ±20%
                            uint32_t lo = (uint32_t)med * 80u / 100u;
                            uint32_t hi = (uint32_t)med * 120u / 100u;
                            if (ibi < lo || ibi > hi) accept_ibi = false;
                        }

                        float bpm_inst = 60000.0f / (float)ibi;

                        // ---- BPM jump gate (อีกชั้น) ----
                        float diffb = bpm_inst - st->bpm;
                        if (diffb < 0) diffb = -diffb;
                        bool accept_bpm = (diffb <= 25.0f);

                        bool accept = accept_ibi && accept_bpm;

                        // อัปเดต last_peak_ms เสมอ (กัน double detect)
                        st->last_peak_ms = peak_ms;
                        // harmonic reject: ถ้า bpm_inst ประมาณ 2x ของ bpm ปัจจุบัน ให้ reject
                        if (st->bpm_valid) {
                            float ratio = bpm_inst / st->bpm;
                            if (ratio > 1.8f && ratio < 2.2f) {
                                // likely double-frequency false peak
                                st->last_peak_ms = peak_ms;
                                return false;
                            }
                        }

                        if (accept)
                        {
                            // EMA
                            st->bpm = (0.8f * st->bpm) + (0.2f * bpm_inst);

                            if (st->bpm < 30.0f) st->bpm = 30.0f;
                            if (st->bpm > 220.0f) st->bpm = 220.0f;

                            st->bpm_valid = 1;
                            st->last_beat_ms = peak_ms;

                            // ---- HRV: store IBI ----
                            st->ibi_ms[st->ibi_w] = (uint16_t)ibi;
                            st->ibi_w = (uint8_t)((st->ibi_w + 1) % HRV_IBI_BUF_N);
                            if (st->ibi_n < HRV_IBI_BUF_N) st->ibi_n++;

                            // ---- RMSSD when >= 6 IBIs ----
                            if (st->ibi_n >= 6)
                            {
                                uint8_t N = HRV_IBI_BUF_N;
                                uint8_t use_n = st->ibi_n;
                                if (use_n > 10) use_n = 10;

                                uint32_t sum_sq = 0;
                                uint8_t count = 0;

                                uint8_t start = (uint8_t)((st->ibi_w + N - use_n) % N);

                                uint16_t prev = st->ibi_ms[start];
                                for (uint8_t k = 1; k < use_n; k++)
                                {
                                    uint8_t idx = (uint8_t)((start + k) % N);
                                    uint16_t cur = st->ibi_ms[idx];

                                    int32_t d = (int32_t)cur - (int32_t)prev;

                                    // outlier reject: ±33%
                                    int32_t prev_i = (int32_t)prev;
                                    if (prev_i < 1) prev_i = 1;
                                    int32_t ad = (d < 0) ? -d : d;

                                    if (ad > (prev_i / 3)) {
                                        prev = cur;
                                        continue;
                                    }

                                    if (d > 200) d = 200;
                                    if (d < -200) d = -200;

                                    sum_sq += (uint32_t)(d * d);
                                    count++;
                                    prev = cur;
                                }

                                if (count > 0)
                                {
                                    float mean_sq = (float)sum_sq / (float)count;
                                    float rmssd = sqrtf(mean_sq);

                                    if (!isfinite(rmssd) || rmssd < 1.0f) rmssd = 1.0f;
                                    if (rmssd > 300.0f) rmssd = 300.0f;

                                    st->rmssd_ms = (0.9f * st->rmssd_ms) + (0.1f * rmssd);
                                    st->rmssd_valid = 1;
                                }
                            }

                            beat = true;
                        }
                        else
                        {
                            // reject: ไม่อัปเดต bpm_valid/last_beat/HRV
                            // แต่ last_peak_ms ถูกอัปเดตแล้วด้านบน
                        }
                    }
                    else
                    {
                        st->last_peak_ms = peak_ms;
                    }
                }
                else
                {
                    st->last_peak_ms = peak_ms;
                }
            }
        }
    }

    st->ac_prev2 = st->ac_prev1;
    st->ac_prev1 = ac;

    return beat;
}
