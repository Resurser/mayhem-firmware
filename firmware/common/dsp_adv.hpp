#ifndef __DSP_ADV_H__
#define __DSP_ADV_H__

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

/**
 * @namespace dsp_utils
 * @brief Utility functions and types for advanced digital signal processing.
 *
 * This namespace provides a collection of DSP algorithms and helpers for signal correction,
 * smoothing, filtering, scaling, color mapping, and noise suppression, commonly used in
 * spectrum analysis and visualization applications.
 *
 * Enumerations:
 * - ScaleMode: Modes for scaling power values (linear, logarithmic, adaptive).
 * - ColorScheme: Predefined color lookup tables for visualization.
 *
 * Structures:
 * - ColorRGB: Represents an RGB color.
 *
 * Global Variables:
 * - LUT: Color lookup table with 256 entries.
 *
 * Functions:
 * - iq_correct: Corrects I/Q signal imbalance.
 * - gaussian_smooth: Applies Gaussian smoothing to data.
 * - savitzky_golay: Applies Savitzky-Golay smoothing filter.
 * - median_filter: Applies median filtering with a specified window size.
 * - scale_power: Scales raw power values according to the selected mode and gain.
 * - generate_lut: Generates a color lookup table for the specified scheme.
 * - get_color: Maps a dB value to an RGB color using the current LUT.
 * - suppress_noise: Suppresses noise in a spectrum based on a threshold in dB.
 * - estimate_noise_threshold: Estimates the noise threshold from a spectrum.
 * - update_heatmap: Updates a heatmap visualization from spectrum data.
 * - erode_waterfall: Applies erosion to a waterfall row for visualization.
 * - mirror_signals_clear: Clears mirror signals using fixed gain and phase correction.
 * - mirror_signals_cancellation: Iteratively cancels mirror signals in I/Q data.
 */
namespace dsp_utils {

enum ScaleMode {
    SCALE_LINEAR = 0,
    SCALE_LOG,
    SCALE_ADAPTIVE
};

enum ColorScheme {
    LUT_JET = 0,
    LUT_HOT,
    LUT_GRAY,
    LUT_COOL,
    LUT_MAGMA
};

struct ColorRGB {
    uint8_t r, g, b;
};
extern ColorRGB LUT[256];

void iq_correct(int16_t* I, int16_t* Q, size_t len);

void gaussian_smooth(uint8_t* data, size_t len, float sigma);
void savitzky_golay(uint8_t* data, size_t len);
void median_filter(uint8_t* data, size_t len, size_t window);
float scale_power(uint8_t raw, ScaleMode mode, float gain);

void generate_lut(ColorScheme scheme);
ColorRGB get_color(float db);

void suppress_noise(uint8_t* spectrum, size_t len, float threshold_db);
float estimate_noise_threshold(uint8_t* spectrum, int size_t);
void update_heatmap(uint8_t* spectrum, size_t len);
void erode_waterfall(uint8_t* row, size_t len);
void dival_waterfall(uint8_t* row, size_t len);

void mirror_signals_clear(int16_t* i_data, int16_t* q_data, size_t length, int32_t gain_fixed, int32_t phase_fixed);
void mirror_signals_cancellation(int16_t* i_data, int16_t* q_data, size_t length, size_t max_iterations = 8);

}  // namespace dsp_utils
#endif  // __DSP_ADV_H__
