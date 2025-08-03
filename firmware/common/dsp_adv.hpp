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
const uint8_t LutLogIdx[256] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    16,
    29,
    40,
    48,
    56,
    63,
    69,
    74,
    79,
    84,
    88,
    92,
    96,
    99,
    103,
    106,
    109,
    112,
    114,
    117,
    119,
    122,
    124,
    126,
    128,
    130,
    132,
    134,
    136,
    138,
    139,
    141,
    143,
    144,
    146,
    147,
    149,
    150,
    151,
    153,
    154,
    155,
    157,
    158,
    159,
    160,
    161,
    163,
    164,
    165,
    166,
    167,
    168,
    169,
    170,
    171,
    172,
    173,
    174,
    175,
    176,
    176,
    177,
    178,
    179,
    180,
    181,
    182,
    182,
    183,
    184,
    185,
    185,
    186,
    187,
    188,
    188,
    189,
    190,
    191,
    191,
    192,
    193,
    193,
    194,
    195,
    195,
    196,
    196,
    197,
    198,
    198,
    199,
    199,
    200,
    201,
    201,
    202,
    202,
    203,
    203,
    204,
    205,
    205,
    206,
    206,
    207,
    207,
    208,
    208,
    209,
    209,
    210,
    210,
    211,
    211,
    212,
    212,
    213,
    213,
    214,
    214,
    215,
    215,
    215,
    216,
    216,
    217,
    217,
    218,
    218,
    218,
    219,
    219,
    220,
    220,
    221,
    221,
    221,
    222,
    222,
    223,
    223,
    223,
    224,
    224,
    225,
    225,
    225,
    226,
    226,
    226,
    227,
    227,
    228,
    228,
    228,
    229,
    229,
    229,
    230,
    230,
    230,
    231,
    231,
    231,
    232,
    232,
    232,
    233,
    233,
    233,
    234,
    234,
    234,
    235,
    235,
    235,
    236,
    236,
    236,
    237,
    237,
    237,
    237,
    238,
    238,
    238,
    239,
    239,
    239,
    240,
    240,
    240,
    240,
    241,
    241,
    241,
    242,
    242,
    242,
    242,
    243,
    243,
    243,
    244,
    244,
    244,
    244,
    245,
    245,
    245,
    245,
    246,
    246,
    246,
    247,
    247,
    247,
    247,
    248,
    248,
    248,
    248,
    249,
    249,
    249,
    249,
    250,
    250,
    250,
    250,
    251,
    251,
    251,
    251,
    252,
    252,
    252,
    252,
    253,
    253,
    253,
    254,
    254,
    255,
    255,
};

void iq_correct(int16_t* I, int16_t* Q, size_t len);

void gaussian_smooth(uint8_t* data, size_t len, float sigma);
void savitzky_golay(uint8_t* data, size_t len);
void median_filter(uint8_t* data, size_t len, size_t window);
float scale_power(uint8_t raw, ScaleMode mode, float gain);

void generate_lut(ColorScheme scheme);
ColorRGB get_color(float db);

void suppress_noise(uint8_t* spectrum, size_t len, uint8_t threshold_db);
float estimate_noise_threshold(uint8_t* spectrum, int size_t);
void update_heatmap(uint8_t* spectrum, size_t len);
void erode_waterfall(uint8_t* row, size_t len);
void dival_waterfall(uint8_t* row, size_t len);

void mirror_signals_clear(int16_t* i_data, int16_t* q_data, size_t length, int32_t gain_fixed, int32_t phase_fixed);
void mirror_signals_cancellation(int16_t* i_data, int16_t* q_data, size_t length, size_t max_iterations = 8);

}  // namespace dsp_utils
#endif  // __DSP_ADV_H__
