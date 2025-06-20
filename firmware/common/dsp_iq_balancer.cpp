#include "dsp_iq_balancer.hpp"

IQBalancerCMSIS::IQBalancerCMSIS() {
    params_ = {0x7FFFFFFF, 0, 0, 0};  // unity gain
}

void IQBalancerCMSIS::estimate(const q31_t* i_buf, const q31_t* q_buf, uint32_t length) {
    q31_t mean_i, mean_q;
    arm_mean_q31(i_buf, length, &mean_i);
    arm_mean_q31(q_buf, length, &mean_q);
    params_.dc_i = mean_i;
    params_.dc_q = mean_q;

    std::vector<q31_t> i_corr(length), q_corr(length);
    for (uint32_t n = 0; n < length; ++n) {
        i_corr[n] = i_buf[n] - mean_i;
        q_corr[n] = q_buf[n] - mean_q;
    }

    q63_t energy_i, energy_q, cross_iq;
    arm_dot_prod_q31(i_corr.data(), i_corr.data(), length, &energy_i);
    arm_dot_prod_q31(q_corr.data(), q_corr.data(), length, &energy_q);
    arm_dot_prod_q31(i_corr.data(), q_corr.data(), length, &cross_iq);

    // Gain compensation (scale Q to match I energy)
    if (energy_q > 0 && energy_i > 0) {
        float32_t gain_q_f = sqrtf((float32_t)energy_i / (float32_t)energy_q);
        params_.gain_q = (q31_t)(gain_q_f * 2147483647.0f);  // Q31
    } else {
        params_.gain_q = 0x7FFFFFFF;
    }

    // Phase error estimation (small-angle approximation)
    float32_t phase_rad = atanf((2.0f * cross_iq) / (energy_i - energy_q + 1e-5f));
    params_.phase_error = (q15_t)(phase_rad * 32768.0f / 3.1415926f);  // Q15
}

void IQBalancerCMSIS::apply(q31_t* i_buf, q31_t* q_buf, uint32_t length) const {
    for (uint32_t n = 0; n < length; ++n) {
        q31_t i = i_buf[n] - params_.dc_i;
        q31_t q = q_buf[n] - params_.dc_q;

        // Масштабуємо Q
        q31_t q_scaled;
        arm_mult_q31(&q, &params_.gain_q, &q_scaled, 1);

        // Застосовуємо фазовий зсув: Q = Q - phase * I
        q31_t phase_term = (q31_t)(((int64_t)i * params_.phase_error) >> 15);
        q31_t q_corr = q_scaled - phase_term;

        i_buf[n] = i;  // I без зміни масштабу
        q_buf[n] = q_corr;
    }
}

const IQBalancerCMSIS::Params& IQBalancerCMSIS::getParams() const {
    return params_;
}
#include <iostream>
#include <vector>
#include <cmath>
#include <complex>

// Структура для параметрів корекції I/Q
struct IQCorrectionParams {
    float gain_i = 1.0f;        // Посилення для I
    float gain_q = 1.0f;        // Посилення для Q
    float phase_offset = 0.0f;  // Фазовий зсув
    float dc_offset_i = 0.0f;   // DC-зсув для I
    float dc_offset_q = 0.0f;   // DC-зсув для Q
};

// Функція для аналізу спектру (перевірка дзеркальних компонент)
bool analyzeSpectrum(const std::vector<std::complex<float>>& iq_samples) {
    float mirror_power = 0.0f;
    float signal_power = 0.0f;

    for (const auto& sample : iq_samples) {
        float power = std::norm(sample);
        signal_power += power;
        if (sample.imag() < 0) {  // Дзеркальні компоненти
            mirror_power += power;
        }
    }

    float ratio = mirror_power / signal_power;
    return ratio > 0.05f;  // Якщо дзеркальні компоненти >5% від основного сигналу, потрібна корекція
}

// Функція для перевірки гістограми (форма має бути коловою)
bool analyzeHistogram(const std::vector<std::complex<float>>& iq_samples) {
    float sum_i = 0.0f, sum_q = 0.0f;
    float sum_i2 = 0.0f, sum_q2 = 0.0f;

    for (const auto& sample : iq_samples) {
        sum_i += sample.real();
        sum_q += sample.imag();
        sum_i2 += sample.real() * sample.real();
        sum_q2 += sample.imag() * sample.imag();
    }

    float variance_i = sum_i2 / iq_samples.size() - (sum_i / iq_samples.size()) * (sum_i / iq_samples.size());
    float variance_q = sum_q2 / iq_samples.size() - (sum_q / iq_samples.size()) * (sum_q / iq_samples.size());

    return std::abs(variance_i - variance_q) > 0.1f;  // Якщо дисперсія сильно відрізняється, потрібна корекція
}

// Функція для вимірювання DC-зсуву
IQCorrectionParams measureDCOffset(const std::vector<std::complex<float>>& iq_samples) {
    float sum_i = 0.0f, sum_q = 0.0f;

    for (const auto& sample : iq_samples) {
        sum_i += sample.real();
        sum_q += sample.imag();
    }

    IQCorrectionParams params;
    params.dc_offset_i = sum_i / iq_samples.size();
    params.dc_offset_q = sum_q / iq_samples.size();
    return params;
}

// Функція для вимірювання фазового зсуву
float measurePhaseOffset(const std::vector<std::complex<float>>& iq_samples) {
    float sum_iq = 0.0f, sum_i2 = 0.0f, sum_q2 = 0.0f;

    for (const auto& sample : iq_samples) {
        sum_iq += sample.real() * sample.imag();
        sum_i2 += sample.real() * sample.real();
        sum_q2 += sample.imag() * sample.imag();
    }

    return std::atan2(2 * sum_iq, sum_i2 - sum_q2);
}

// Функція для корекції I/Q
std::complex<float> correctIQ(std::complex<float> iq_sample, const IQCorrectionParams& params) {
    // Компенсація DC-зсуву
    iq_sample.real(iq_sample.real() - params.dc_offset_i);
    iq_sample.imag(iq_sample.imag() - params.dc_offset_q);

    // Компенсація амплітудного дисбалансу
    iq_sample.real(iq_sample.real() * params.gain_i);
    iq_sample.imag(iq_sample.imag() * params.gain_q);

    // Компенсація фазового зсуву
    return iq_sample * std::polar(1.0f, -params.phase_offset);
}

// Основна функція для автоматичної компенсації I/Q
void autoIQCompensation(std::vector<std::complex<float>>& iq_samples) {
    // Аналіз спектру
    bool spectrum_issue = analyzeSpectrum(iq_samples);
    bool histogram_issue = analyzeHistogram(iq_samples);

    // Вимірювання DC-зсуву та фазового зсуву
    IQCorrectionParams params = measureDCOffset(iq_samples);
    params.phase_offset = measurePhaseOffset(iq_samples);

    // Якщо є проблеми, застосовуємо корекцію
    if (spectrum_issue || histogram_issue) {
        for (auto& sample : iq_samples) {
            sample = correctIQ(sample, params);
        }
    }
}

// Приклад використання
int main() {
    // Генерація тестових I/Q даних (імітація сигналу)
    std::vector<std::complex<float>> iq_samples = {
        {0.1f, 0.5f}, {0.2f, 0.6f}, {0.3f, 0.7f}, {0.4f, 0.8f}};

    // Автоматична компенсація I/Q
    autoIQCompensation(iq_samples);

    // Виведення скоригованих даних
    for (const auto& sample : iq_samples) {
        printf("Corrected I: %.3f, Corrected Q: %.3f\n", sample.real(), sample.imag());
    }

    return 0;
}