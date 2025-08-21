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

class WaveletDenoise {
public:
    // Конструктор: levels_ - кількість рівнів декомпозиції
    explicit WaveletDenoise(unsigned int levels = 3) : decomposition_levels(levels) {
        if (decomposition_levels == 0) decomposition_levels = 1; // Мінімум 1 рівень
    }

    // Основна функція шумоподавлення
    // input_signal: вказівник на вхідні дані (0..255)
    // length: довжина сигналу (має бути ступенем двійки для простоти Хаара)
    // output_signal: вказівник на буфер для виведення очищених даних (0..255)
    bool denoise(const uint8_t* input_signal, size_t length, uint8_t* output_signal) {
        if (input_signal == nullptr || output_signal == nullptr || length == 0) {
            return false; // Недійсні вхідні дані
        }

        // Перевірка, чи довжина сигналу є ступенем двійки
        if ((length & (length - 1)) != 0) {
            // Для реальних SDR сигналів, де довжина може бути довільною,
            // потрібно реалізувати доповнення нулями (padding) або відображенням.
            // Для простоти цього прикладу, вимагаємо ступінь двійки.
            return false; // Довжина не є ступенем двійки, не підтримується цим прикладом
        }

        // 1. Конвертація uint8_t в double для внутрішньої обробки
        // Ми працюємо в діапазоні float/double [0.0, 255.0]
        std::vector<double> data(length);
        for (size_t i = 0; i < length; ++i) {
            data[i] = static_cast<double>(input_signal[i]);
        }

        // 2. Виконання Дискретного Вейвлет-Перетворення (DWT) - Хаар
        size_t current_len = length;
        std::vector<double> detail_coeffs_finest; // Для оцінки шуму на найвищому рівні
        std::vector<size_t> level_lengths(decomposition_levels); // Довжини на кожному рівні

        for (unsigned int level = 0; level < decomposition_levels; ++level) {
            if (current_len < 2) break; // Недостатньо даних для подальшої декомпозиції

            level_lengths[level] = current_len;
            apply_haar_dwt_level(data.data(), current_len);
            current_len /= 2;

            // Зберегти детальні коефіцієнти найвищого рівня для оцінки шуму
            if (level == 0) { // Перший рівень декомпозиції
                detail_coeffs_finest.assign(data.begin() + current_len, data.begin() + length);
            }
        }

        // 3. Оцінка шуму та обчислення порогу
        // Поріг Донахо: T = sigma * sqrt(2 * log(N))
        // Оцінка sigma: sigma = MAD / 0.6745
        double noise_std = 0.0;
        if (!detail_coeffs_finest.empty()) {
            noise_std = estimate_noise_std(detail_coeffs_finest.data(), detail_coeffs_finest.size());
        }

        double threshold = noise_std * std::sqrt(2.0 * std::log(static_cast<double>(length)));


        // 4. Застосування порогової обробки до детальних коефіцієнтів
        current_len = length; // Починаємо з повного буфера
        size_t approx_len = length / std::pow(2, decomposition_levels); // Довжина апроксимації на останньому рівні

        for (unsigned int level = 0; level < decomposition_levels; ++level) {
            size_t start_idx = (level == 0) ? length / 2 : length / (std::pow(2, level + 1));
            size_t detail_len = start_idx; // Довжина детальних коефіцієнтів на цьому рівні

            // Для вкладеного DWT/IDWT, детальні коефіцієнти зберігаються після апроксимаційних.
            // Перша половина буфера - апроксимація, друга - деталі.
            // На наступному рівні, перша половина знову розбивається.
            // Отже, детальні коефіцієнти поточного рівня знаходяться в діапазоні [current_approx_len, current_len]
            // де current_approx_len - це довжина апроксимаційних коефіцієнтів для *цього* рівня.
            
            // Якщо ми зберігаємо всі коефіцієнти в одному векторі, то після першої декомпозиції:
            // data[0...length/2-1] = A1 (approx. level 1)
            // data[length/2...length-1] = D1 (detail level 1)
            // Після другої декомпозиції A1:
            // data[0...length/4-1] = A2
            // data[length/4...length/2-1] = D2
            // data[length/2...length-1] = D1
            // і так далі.
            // Отже, для порогу обробки ми працюємо з деталями D1, D2, D3...
            
            // Застосовуємо поріг до детальних коефіцієнтів (починаючи з найгрубіших/низьких рівнів в буфері,
            // які відповідають D3, потім D2, D1).
            // Довжина поточного блоку деталей
            size_t block_start_idx = current_len / 2;
            size_t block_len = current_len / 2;

            // Soft thresholding (м'яка порогова обробка)
            apply_soft_threshold(data.data() + block_start_idx, block_len, threshold);
            
            current_len = block_start_idx; // Переходимо до наступного (меншого) блоку для обробки
        }


        // 5. Виконання Оберненого Вейвлет-Перетворення (IDWT) - Хаар
        current_len = static_cast<size_t>(length / std::pow(2, decomposition_levels -1)); // Довжина першого рівня реконструкції

        for (unsigned int level = 0; level < decomposition_levels; ++level) {
            // Починаємо з найменших апроксимаційних коефіцієнтів, які є після порогової обробки.
            // current_len збільшується на кожному кроці IDWT.
            apply_haar_idwt_level(data.data(), current_len);
            current_len *= 2;
        }

        // 6. Конвертація double назад у uint8_t з кліпуванням
        for (size_t i = 0; i < length; ++i) {
            double val = data[i];
            // Кліпування значень, щоб вони були в діапазоні [0, 255]
            if (val < 0.0) val = 0.0;
            if (val > 255.0) val = 255.0;
            output_signal[i] = static_cast<uint8_t>(val + 0.5); // Додаємо 0.5 для округлення
        }

        return true;
    }

private:
    unsigned int decomposition_levels;

    // Застосовує один рівень DWT Хаара
    // data: вказівник на масив даних (модифікується на місці)
    // len: поточна довжина сегмента даних (має бути парною)
    void apply_haar_dwt_level(double* data, size_t len) {
        if (len < 2 || len % 2 != 0) return;

        std::vector<double> temp(len); // Тимчасовий буфер

        for (size_t i = 0; i < len / 2; ++i) {
            temp[i] = (data[2 * i] + data[2 * i + 1]) / 2.0;         // Апроксимаційні (Avg)
            temp[i + len / 2] = (data[2 * i] - data[2 * i + 1]) / 2.0; // Детальні (Diff)
        }

        // Копіюємо результати назад у вихідний масив
        for (size_t i = 0; i < len; ++i) {
            data[i] = temp[i];
        }
    }

    // Застосовує один рівень оберненого DWT Хаара
    // data: вказівник на масив даних (модифікується на місці)
    // len: поточна довжина сегмента даних (має бути парною)
    void apply_haar_idwt_level(double* data, size_t len) {
        if (len < 2 || len % 2 != 0) return;

        std::vector<double> temp(len); // Тимчасовий буфер

        for (size_t i = 0; i < len / 2; ++i) {
            double approx = data[i];           // Апроксимаційні
            double detail = data[i + len / 2]; // Детальні

            temp[2 * i] = approx + detail; // Відновлений S_i
            temp[2 * i + 1] = approx - detail; // Відновлений S_{i+1}
        }

        // Копіюємо результати назад у вихідний масив
        for (size_t i = 0; i < len; ++i) {
            data[i] = temp[i];
        }
    }

    // Оцінка стандартного відхилення шуму за допомогою MAD (Median Absolute Deviation)
    // detail_coeffs: вказівник на детальні коефіцієнти (з найвищого рівня DWT)
    // len: довжина масиву детальних коефіцієнтів
    double estimate_noise_std(const double* detail_coeffs, size_t len) {
        if (len == 0) return 0.0;

        std::vector<double> abs_coeffs(len);
        for (size_t i = 0; i < len; ++i) {
            abs_coeffs[i] = std::fabs(detail_coeffs[i]);
        }

        std::sort(abs_coeffs.begin(), abs_coeffs.end());

        double median_abs_dev;
        if (len % 2 == 1) {
            median_abs_dev = abs_coeffs[len / 2];
        } else {
            median_abs_dev = (abs_coeffs[len / 2 - 1] + abs_coeffs[len / 2]) / 2.0;
        }

        // Коефіцієнт для переведення MAD в оцінку стандартного відхилення для нормального розподілу
        const double MAD_TO_STD_FACTOR = 1.4826; // 1 / Q_inverse(0.75) приблизно
        return median_abs_dev * MAD_TO_STD_FACTOR;
    }

    // Застосовує м'яку порогову обробку
    // data: вказівник на детальні коефіцієнти
    // len: довжина сегмента
    // threshold: значення порогу
    void apply_soft_threshold(double* data, size_t len, double threshold) {
        for (size_t i = 0; i < len; ++i) {
            if (std::fabs(data[i]) < threshold) {
                data[i] = 0.0;
            } else {
                data[i] = std::copysign(std::fabs(data[i]) - threshold, data[i]);
            }
        }
    }
};

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