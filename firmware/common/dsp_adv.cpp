/*
# DSP Enhancements for PortaPack Mayhem Firmware

- IQ-корекція
- Згладжування: Gaussian, Median, Moving Average, Savitzky-Golay, Adaptive
- Масштабування: Linear, Log, Adaptive
- Кольорові схеми: Jet, Hot, Gray, Cool, Magma
- Порогове та адаптивне шумоподавлення
- Приклад інтеграції в `spectrum.cpp`

##Інтеграція

1. Копіюй `dsp/` в `firmware/dsp/`
2. Замінити або оновити `apps/spectrum.cpp` за прикладом `spectrum_patch.cpp`
3. Додай `.c` файли до `Makefile`

*/
#include "dsp_adv.h"

#include <math.h>
#define MAX_KERNEL 25
static float kernel[MAX_KERNEL];

void update_heatmap(float* spectrum, size_t len) {
    for (size_t i = 0; i < len; i++) {
        heatmap[i] = 0.95f * heatmap[i] + 0.05f * spectrum[i]; // EMA
    }
}
void erode_waterfall(uint8_t* row, size_t len) {
    for (size_t i = 1; i < len - 1; i++) {
        uint8_t min_val = row[i];
        if (row[i - 1] < min_val) min_val = row[i - 1];
        if (row[i + 1] < min_val) min_val = row[i + 1];
        row[i] = min_val;
    }
}
void iq_correct(float* I, float* Q, size_t len) {
    float sumI = 0.0f, sumQ = 0.0f;
    for (size_t i = 0; i < len; i++) {
        sumI += I[i];
        sumQ += Q[i];
    }
    float meanI = sumI / len;
    float meanQ = sumQ / len;

    for (size_t i = 0; i < len; i++) {
        I[i] -= meanI;
        Q[i] -= meanQ;
    }

    float rmsI = 0.0f, rmsQ = 0.0f;
    for (size_t i = 0; i < len; i++) {
        rmsI += I[i]*I[i];
        rmsQ += Q[i]*Q[i];
    }
    rmsI = sqrtf(rmsI / len);
    rmsQ = sqrtf(rmsQ / len);
    float gain = rmsI / (rmsQ + 1e-6f);
    for (size_t i = 0; i < len; i++)
        Q[i] *= gain;
}

void gaussian_smooth(float* data, size_t len, float sigma) {
    int radius = (int)(3.0f * sigma);
    int ksize = 2 * radius + 1;
    if (ksize > MAX_KERNEL) ksize = MAX_KERNEL;

    float sum = 0.0f;
    for (int i = -radius; i <= radius; i++) {
        int idx = i + radius;
        kernel[idx] = expf(-0.5f * (i*i) / (sigma*sigma));
        sum += kernel[idx];
    }
    for (int i = 0; i < ksize; i++) kernel[i] /= sum;

    float temp[len];
    for (int i = 0; i < len; i++) temp[i] = data[i];

    for (int i = 0; i < len; i++) {
        float acc = 0.0f;
        for (int j = -radius; j <= radius; j++) {
            int idx = i + j;
            if (idx < 0) idx = 0;
            if (idx >= len) idx = len - 1;
            acc += temp[idx] * kernel[j + radius];
        }
        data[i] = acc;
    }
}
float estimate_noise_threshold(float* spectrum, size_t len) {
    float sum = 0.0f, var = 0.0f;
    for (size_t i = 0; i < len; i++)
        sum += spectrum[i];
    float mean = sum / len;
    for (size_t i = 0; i < len; i++)
        var += (spectrum[i] - mean) * (spectrum[i] - mean);
    var /= len;
    float Q = var / (mean * mean + 1e-6f);
    return (Q < 2.0f) ? mean + 3.0f : mean + 6.0f;
}
void suppress_noise(float* spectrum, size_t len, float threshold_db) {
    for (size_t i = 0; i < len; i++)
        if (spectrum[i] < threshold_db)
            spectrum[i] = 0.0f;
}
void savitzky_golay(float* data, size_t len) {
    const float coeffs[5] = { -3.0f/35, 12.0f/35, 17.0f/35, 12.0f/35, -3.0f/35 };
    float temp[len];
    for (int i = 2; i < len - 2; i++) {
        float acc = 0.0f;
        for (int j = -2; j <= 2; j++)
            acc += data[i + j] * coeffs[j + 2];
        temp[i] = acc;
    }
    for (size_t i = 2; i < len - 2; i++)
        data[i] = temp[i];
}
void median_filter(uint8_t* data, size_t len, size_t window) {
    uint8_t temp[len];
    for (int i = 0; i < len; i++) {
        int half = window / 2;
        uint8_t buf[15];
        int count = 0;
        for (int j = -half; j <= half; j++) {
            int idx = i + j;
            if (idx < 0) idx = 0;
            if (idx >= len) idx = len - 1;
            buf[count++] = data[idx];
        }
        for (int m = 1; m < count; m++) {
            uint8_t key = buf[m];
            int n = m - 1;
            while (n >= 0 && buf[n] > key) {
                buf[n + 1] = buf[n];
                n--;
            }
            buf[n + 1] = key;
        }
        temp[i] = buf[count / 2];
    }
    for (size_t i = 0; i < len; i++) data[i] = temp[i];
}

ColorRGB LUT[256];
static ColorScheme active_scheme = LUT_JET;

float scale_power(float raw, ScaleMode mode, float gain) {
    static float avg = 0.0f;
    switch (mode) {
        case SCALE_LINEAR: return raw * gain;
        case SCALE_LOG: return 10.0f * log10f(raw + 1e-6f) * gain;
        case SCALE_ADAPTIVE:
            avg = 0.95f * avg + 0.05f * raw;
            return (raw - avg) * gain;
        default: return raw;
    }
}
void generate_lut(ColorScheme scheme) {
    active_scheme = scheme;
    for (int i = 0; i < 256; i++) {
        float v = i / 255.0f;
        switch (scheme) {
            case LUT_GRAY: LUT[i] = (ColorRGB){i,i,i}; break;
            case LUT_COOL: LUT[i] = (ColorRGB){255*(1-v),255*v,255}; break;
            case LUT_HOT: LUT[i] = (ColorRGB){255*v,128*v,64*v}; break;
            case LUT_JET:
                LUT[i] = (ColorRGB){
                    (unsigned char)(255 * fmax(0,fmin(1,4*(v - 0.75f)))),
                    (unsigned char)(255 * fmax(0,fmin(1,4*fabs(v - 0.5f)))),
                    (unsigned char)(255 * fmax(0,fmin(1,4*(0.25f - v))))
                }; break;
            case LUT_MAGMA:
                LUT[i] = (ColorRGB){
                    (unsigned char)(255 * powf(v, 1.5f)),
                    (unsigned char)(255 * powf(v, 0.8f)),
                    (unsigned char)(255 * powf(v, 0.3f))
                }; break;
        }
    }
}

ColorRGB get_color(float db) {
    int i = (int)(db * 2.5f + 100);
    if (i < 0) i = 0;
    if (i > 255) i = 255;
    return LUT[i];
}