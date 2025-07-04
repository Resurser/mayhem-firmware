#ifndef INCLUDE_DSP_ADV_H
#define INCLUDE_DSP_ADV_H

typedef enum { SCALE_LINEAR = 0, SCALE_LOG, SCALE_ADAPTIVE } ScaleMode;
typedef enum { LUT_JET = 0, LUT_HOT, LUT_GRAY, LUT_COOL, LUT_MAGMA } ColorScheme;
typedef struct { uint8_t r, g, b; } ColorRGB;

extern ColorRGB LUT[256];

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
#endif // INCLUDE_DSP_ADV_H

