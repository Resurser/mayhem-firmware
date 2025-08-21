/*
# DSP Enhancements for PortaPack Mayhem Firmware

- IQ-корекція
- Згладжування: Gaussian, Median, Moving Average, Savitzky-Golay, Adaptive
- Масштабування: Linear, Log, Adaptive
- Кольорові схеми: Jet, Hot, Gray, Cool, Magma
- Порогове та адаптивне шумоподавлення
- Приклад інтеграції в `spectrum.cpp`
*/
#include "dsp_adv.hpp"
#include <math.h>
#include <algorithm>
#include "dsp_types.hpp"
#include "utility.hpp"

namespace dsp_utils {

#define MAX_KERNEL 25
constexpr int32_t SCALE = 32768;  // Q15-фіксована точка
static float kernel[MAX_KERNEL];
static float heatmap[256] = {0};  // Ініціалізація теплової карти

void update_heatmap(uint8_t* spectrum, size_t len) {
    for (size_t i = 0; i < len; i++) {
        heatmap[i] = 0.95f * heatmap[i] + 0.05f * spectrum[i];  // EMA
    }
    
}

void erode_waterfall(uint8_t* row, size_t len) {
    uint8_t tmp[len];
    tmp[0] = row[0];
    tmp[len - 1] = row[len - 1];
    for (size_t i = 1; i < len - 1; i++) {
        uint8_t min_val = row[i];
        if (row[i - 1] < min_val) min_val = row[i - 1];
        if (row[i + 1] < min_val) min_val = row[i + 1];
        tmp[i] = min_val;
    }
    for (size_t i = 1; i < len-1; i++) {
        row[i] = tmp[i];
    }
}

void dival_waterfall(uint8_t* row, size_t len) {
    uint8_t tmp[len];
    tmp[0] = row[0];
    tmp[len - 1] = row[len - 1];
    for (size_t i = 1; i < len - 1; i++) {
        uint8_t max_val = row[i];
        if (row[i - 1] > max_val) max_val = row[i - 1];
        if (row[i + 1] > max_val) max_val = row[i + 1];
        tmp[i] = max_val;
    }
    for (size_t i = 0; i < len; i++) {
        row[i] = tmp[i];
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
        rmsI += I[i] * I[i];
        rmsQ += Q[i] * Q[i];
    }
    rmsI = sqrtf(rmsI / len);
    rmsQ = sqrtf(rmsQ / len);
    float gain = rmsI / (rmsQ + 1e-6f);
    for (size_t i = 0; i < len; i++)
        Q[i] *= gain;
}

void gaussian_smooth(float* data, int len, float sigma) {
    int radius = (int)(3.0f * sigma);
    int ksize = 2 * radius + 1;
    if (ksize > MAX_KERNEL) ksize = MAX_KERNEL;

    float sum = 0.0f;
    for (int i = -radius; i <= radius; i++) {
        int idx = i + radius;
        kernel[idx] = expf(-0.5f * (i * i) / (sigma * sigma));
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

/**
 * @brief Estimates a noise threshold from a given spectrum.
 *
 * This function calculates the mean and variance of the input spectrum,
 * then computes a normalized variance (Q). Based on Q, it returns a threshold
 * value for noise estimation:
 *   - If Q < 2.0, returns mean + 3.0
 *   - Otherwise, returns mean + 6.0
 *
 * @param spectrum Pointer to the array containing spectrum data (uint8_t).
 * @param len Number of elements in the spectrum array.
 * @return Estimated noise threshold as a float.
 */
float estimate_noise_threshold(uint8_t* spectrum, size_t len) {
    uint32_t sum = 0;
    float var = 0.0f;

    for (size_t i = 0; i < len; i++)
        sum += spectrum[i];
    float mean = sum / len;
    for (size_t i = 0; i < len; i++)
        var += (spectrum[i] - mean) * (spectrum[i] - mean);
    var /= len;
    float Q = var / (mean * mean + 1e-6f);
    return (Q < 2.0f) ? mean + 3.0f : mean + 6.0f;
}

void suppress_noise(uint8_t* spectrum, size_t len, uint8_t threshold_db) {
    for (size_t i = 0; i < len; i++)
        if (spectrum[i] < threshold_db)
            spectrum[i] = 0;
}

void savitzky_golay(uint8_t* data, size_t len) {
    const int8_t coeffs[5] = {-3, 12, 17, 12, -3};
    uint8_t temp[len];
    for (size_t i = 2; i < len - 2; i++) {
        int32_t acc = 0;
        for (int j = -2; j <= 2; j++)
            acc += data[i + j] * coeffs[j + 2];
        temp[i] = acc / 35;
    }
    for (size_t i = 2; i < len - 2; i++)
        data[i] = temp[i];
}

void median_filter(uint8_t* data, int len, size_t window) {
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
    for (int i = 0; i < len; i++) data[i] = temp[i];
}

// Основна корекція дзеркала
void mirror_signals_clear(int16_t* i_data, int16_t* q_data, size_t length, int32_t gain_fixed, int32_t phase_fixed) {
    for (size_t i = 0; i < length; ++i) {
        int32_t ci = (i_data[i] * gain_fixed) >> 15;  // Q15 множення
        int32_t cq = q_data[i] - ((ci * phase_fixed) >> 15);

        i_data[i] = ci < -32768 ? -32768 : (ci > 32767 ? 32767 : ci);
        q_data[i] = cq < -32768 ? -32768 : (cq > 32767 ? 32767 : cq);  // Обмеження значень
    }
}

// Вимір потужності дзеркального сигналу
static int32_t measureImagePower(const int16_t* i_data, const int16_t* q_data, size_t len) {
    int64_t power = 0;
    for (size_t i = 0; i < len; ++i)
        power += std::abs(static_cast<int32_t>(i_data[i]) * q_data[i]);
    return static_cast<int32_t>(power / len);
}

// Автоматичне пригнічення дзеркальних сигналів
void mirror_signals_cancellation(int16_t* i_data, int16_t* q_data, size_t length, size_t max_iterations) {
    int32_t gain_fixed = SCALE;
    int32_t phase_fixed = 0;

    const int32_t gain_step = SCALE / 256;
    const int32_t phase_step = SCALE / 512;

    for (size_t it = 0; it < max_iterations; ++it) {
        int32_t img_power = measureImagePower(i_data, q_data, length);

        // Модифікація коефіцієнтів на основі зворотного зв’язку
        gain_fixed -= (gain_step * img_power) >> 15;
        phase_fixed -= (phase_step * img_power) >> 15;

        mirror_signals_clear(i_data, q_data, length, gain_fixed, phase_fixed);
    }
}

}  // namespace dsp_utils