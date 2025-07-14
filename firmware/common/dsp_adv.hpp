#ifndef __DSP_ADV_H__
#define __DSP_ADV_H__

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

/**
 * @namespace dsp_adv
 * @brief Advanced digital signal processing utilities.
 *
 * This namespace provides a collection of functions and types for advanced DSP tasks,
 * including IQ correction, smoothing, filtering, scaling, color mapping, noise suppression,
 * and signal mirroring. It is designed for use in firmware applications requiring real-time
 * signal analysis and visualization.
 *
 * @enum ScaleMode
 *   - SCALE_LINEAR: Linear scaling mode.
 *   - SCALE_LOG: Logarithmic scaling mode.
 *   - SCALE_ADAPTIVE: Adaptive scaling mode.
 *
 * @enum ColorScheme
 *   - LUT_JET: Jet color map.
 *   - LUT_HOT: Hot color map.
 *   - LUT_GRAY: Grayscale color map.
 *   - LUT_COOL: Cool color map.
 *   - LUT_MAGMA: Magma color map.
 *
 * @struct ColorRGB
 *   - r: Red channel (0-255).
 *   - g: Green channel (0-255).
 *   - b: Blue channel (0-255).
 *
 * @var LUT
 *   - Lookup table of 256 RGB colors for visualization.
 *
 * @fn void iq_correct(float* I, float* Q, size_t len)
 *   - Corrects IQ imbalance in the provided I/Q data arrays.
 *
 * @fn void gaussian_smooth(float* data, size_t len, float sigma)
 *   - Applies Gaussian smoothing to the data array.
 *
 * @fn void savitzky_golay(float* data, size_t len)
 *   - Applies Savitzky-Golay smoothing filter to the data array.
 *
 * @fn void median_filter(uint8_t* data, size_t len, size_t window)
 *   - Applies a median filter with the specified window size to the data array.
 *
 * @fn float scale_power(float raw, ScaleMode mode, float gain)
 *   - Scales the raw power value according to the selected mode and gain.
 *
 * @fn void generate_lut(ColorScheme scheme)
 *   - Generates a color lookup table based on the selected color scheme.
 *
 * @fn ColorRGB get_color(float db)
 *   - Maps a dB value to an RGB color using the current LUT.
 *
 * @fn void suppress_noise(float* spectrum, size_t len, float threshold_db)
 *   - Suppresses noise in the spectrum below the specified dB threshold.
 *
 * @fn float estimate_noise_threshold(float* spectrum, int size_t)
 *   - Estimates the noise threshold from the spectrum data.
 *
 * @fn void update_heatmap(float* spectrum, size_t len)
 *   - Updates the heatmap visualization with the current spectrum data.
 *
 * @fn void erode_waterfall(uint8_t* row, size_t len)
 *   - Applies erosion to a waterfall display row for visualization effects.
 *
 * @fn void mirror_signals_clear(int16_t* i_data, int16_t* q_data, size_t length, int32_t gain_fixed, int32_t phase_fixed)
 *   - Clears mirror signals using fixed gain and phase correction.
 *
 * @fn void mirror_signals_cancellation(int16_t* i_data, int16_t* q_data, size_t length, size_t max_iterations = 8)
 *   - Cancels mirror signals using iterative optimization.
 */
namespace dsp_adv {

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
abs extern ColorRGB LUT[256];

void iq_correct(float* I, float* Q, size_t len);

void gaussian_smooth(float* data, size_t len, float sigma);
void savitzky_golay(float* data, size_t len);
void median_filter(uint8_t* data, size_t len, size_t window);

float scale_power(float raw, ScaleMode mode, float gain);

void generate_lut(ColorScheme scheme);
ColorRGB get_color(float db);

void suppress_noise(float* spectrum, size_t len, float threshold_db);
float estimate_noise_threshold(float* spectrum, int size_t);
void update_heatmap(float* spectrum, size_t len);
void erode_waterfall(uint8_t* row, size_t len);

void mirror_signals_clear(int16_t* i_data, int16_t* q_data, size_t length, int32_t gain_fixed, int32_t phase_fixed);
void mirror_signals_cancellation(int16_t* i_data, int16_t* q_data, size_t length, size_t max_iterations = 8);

}  // namespace dsp_adv
#endif  // __DSP_ADV_H__
