/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
 * Copyleft Mr. Robot 2025
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ui_spectrum.hpp"

#include "portapack.hpp"
using namespace portapack;

#include "baseband_api.hpp"
#include "string_format.hpp"
#include "utility.hpp"

#include <cmath>
#include <array>

namespace pmem = portapack::persistent_memory;

namespace ui {
namespace spectrum {

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

/* AudioSpectrumView ******************************************************/

AudioSpectrumView::AudioSpectrumView(
    const Rect parent_rect)
    : View{parent_rect} {
    set_focusable(true);

    add_children({&labels,
                  &field_frequency,
                  &waveform});

    field_frequency.on_change = [this](int32_t) {
        set_dirty();
    };
    field_frequency.set_value(0);
}

void AudioSpectrumView::paint(Painter& painter) {
    const auto r = screen_rect();

    painter.fill_rectangle(r, Theme::getInstance()->bg_darkest->background);

    // if( !spectrum_sampling_rate ) return;

    // Cursor
    const Rect r_cursor{
        field_frequency.value() / (48000 / 240), r.bottom() - 32 - cursor_band_height,
        1, cursor_band_height};
    painter.fill_rectangle(
        r_cursor,
        Color::red());
}

void AudioSpectrumView::on_audio_spectrum(const AudioSpectrum* spectrum) {
    for (size_t i = 0; i < spectrum->db.size(); i++)
        audio_spectrum[i] = ((int16_t)spectrum->db[i] - 127) * 256;
    waveform.set_dirty();
}

/* FrequencyScale ********************************************************/

void FrequencyScale::on_show() {
    clear();
}

void FrequencyScale::set_spectrum_sampling_rate(const int new_sampling_rate) {
    if ((spectrum_sampling_rate != new_sampling_rate)) {
        spectrum_sampling_rate = new_sampling_rate;
        set_dirty();
    }
}

void FrequencyScale::set_channel_filter(
    const int low_frequency,
    const int high_frequency,
    const int transition) {
    if ((channel_filter_low_frequency != low_frequency) ||
        (channel_filter_high_frequency != high_frequency) ||
        (channel_filter_transition != transition)) {
        channel_filter_low_frequency = low_frequency;
        channel_filter_high_frequency = high_frequency;
        channel_filter_transition = transition;
        set_dirty();
    }
}

void FrequencyScale::set_ddc_freq(const int freq) {
    if (ddc_freq != freq) {
        ddc_freq = freq;
        set_dirty();
    }
}

void FrequencyScale::set_cursor_position(const int32_t position) {
    cursor_position = position;

    cursor_position = std::min<int32_t>(cursor_position, 119);
    cursor_position = std::max<int32_t>(cursor_position, -120);

    set_dirty();
}

void FrequencyScale::paint(Painter& painter) {
    const auto r = screen_rect();

    clear_background(painter, r);

    if (!spectrum_sampling_rate) {
        // Can't draw without non-zero scale.
        return;
    }

    draw_filter_ranges(painter, r);
    draw_frequency_ticks(painter, r);
    if (abs(cursor_position) > 2) {
        const Rect r_cursor{
            118 + cursor_position, r.bottom() - filter_band_height,
            5, filter_band_height};
        painter.fill_rectangle(
            r_cursor,
            Color::red());
    }
}

void FrequencyScale::clear() {
    spectrum_sampling_rate = 0;
    set_dirty();
}

void FrequencyScale::clear_background(Painter& painter, const Rect r) {
    painter.fill_rectangle(r, Theme::getInstance()->bg_darkest->background);
}

void FrequencyScale::draw_frequency_ticks(Painter& painter, const Rect r) {
    const auto x_center = r.width() / 2;

    // const Rect tick{r.left() + x_center, r.top(), 1, r.height()};
    const Rect tick{
        r.left() + x_center + ddc_freq * spectrum_bins / spectrum_sampling_rate,
        r.bottom() - filter_band_height * 2,
        1,
        filter_band_height * 2};
    painter.fill_rectangle(tick, Theme::getInstance()->bg_darkest->foreground);

    constexpr int tick_count_max = 4;
    float rough_tick_interval = float(spectrum_sampling_rate) / tick_count_max;
    int magnitude = 1;
    int magnitude_n = 0;
    while (rough_tick_interval >= 10.0f) {
        rough_tick_interval /= 10;
        magnitude *= 10;
        magnitude_n += 1;
    }
    const int tick_interval = std::ceil(rough_tick_interval);

    auto tick_offset = tick_interval;
    while ((tick_offset * magnitude) < spectrum_sampling_rate / 2) {
        const Dim pixel_offset = tick_offset * magnitude * spectrum_bins / spectrum_sampling_rate;

        const std::string zero_pad =
            ((magnitude_n % 3) == 0) ? "" : ((magnitude_n % 3) == 1) ? "0"
                                                                     : "00";
        const std::string unit =
            (magnitude_n >= 6) ? "M" : (magnitude_n >= 3) ? "k"
                                                          : "";

        const std::string label = to_string_dec_uint(tick_offset) + zero_pad + unit;
        const auto label_width = style().font.size_of(label).width();

        const Coord offset_low = r.left() + x_center - pixel_offset;
        const Rect tick_low{offset_low, r.top(), 1, r.height()};
        painter.fill_rectangle(tick_low, Theme::getInstance()->bg_darkest->foreground);
        painter.draw_string({offset_low + 2, r.top()}, style(), label);

        const Coord offset_high = r.left() + x_center + pixel_offset;
        const Rect tick_high{offset_high, r.top(), 1, r.height()};
        painter.fill_rectangle(tick_high, Theme::getInstance()->bg_darkest->foreground);
        painter.draw_string({offset_high - 2 - label_width, r.top()}, style(), label);

        tick_offset += tick_interval;
    }
}

void FrequencyScale::draw_filter_ranges(Painter& painter, const Rect r) {
    if (channel_filter_low_frequency != channel_filter_high_frequency) {
        // const auto x_center = r.width() / 2;
        const auto x_center = r.width() / 2 + ddc_freq * spectrum_bins / spectrum_sampling_rate;

        const auto x_low = x_center + channel_filter_low_frequency * spectrum_bins / spectrum_sampling_rate;
        const auto x_high = x_center + channel_filter_high_frequency * spectrum_bins / spectrum_sampling_rate;

        if (channel_filter_transition) {
            const auto trans = channel_filter_transition * spectrum_bins / spectrum_sampling_rate;

            const Rect r_all{
                r.left() + x_low - trans, r.bottom() - filter_band_height,
                x_high - x_low + trans * 2, filter_band_height};
            painter.fill_rectangle(
                r_all,
                Color::yellow());
        }

        const Rect r_pass{
            r.left() + x_low, r.bottom() - filter_band_height,
            x_high - x_low, filter_band_height};
        painter.fill_rectangle(
            r_pass,
            Color::green());
    }
}

void FrequencyScale::on_focus() {
    set_dirty();
}

void FrequencyScale::on_blur() {
    set_dirty();
}

bool FrequencyScale::on_encoder(const EncoderEvent delta) {
    cursor_position += delta;

    cursor_position = std::min<int32_t>(cursor_position, 119);
    cursor_position = std::max<int32_t>(cursor_position, -120);

    set_dirty();

    return true;
}

bool FrequencyScale::on_key(const KeyEvent key) {
    if (key == KeyEvent::Select) {
        if (on_select) {
            on_select((cursor_position * spectrum_sampling_rate) / 240);
            cursor_position = 0;
            set_dirty();
            return true;
        }
    }

    return false;
}

bool FrequencyScale::on_touch(const TouchEvent touch) {
    if (touch.type == TouchEvent::Type::Start) {
        if (on_select) {
            on_select((touch.point.x() * spectrum_sampling_rate) / 240);
        }
    }
    return true;
}

/* WaterfallWidget *********************************************************/
// TODO: buffer and use "paint" instead of immediate drawing would help with
// preventing flicker from drawing. Would use more RAM however.

void WaterfallWidget::on_show() {
    clear();
    logLUT[0]= 0;
    for (int i = 1; i < 256; i++) {
        logLUT[i] = static_cast<uint8_t>(20 * fast_log10(i / 255.0f) * 5.05 + 255);
    }

    const auto screen_r = screen_rect();
    display.scroll_set_area(screen_r.top(), screen_r.bottom());
}

void WaterfallWidget::on_hide() {
    /* TODO: Clear region to eliminate brief flash of content at un-shifted
     * position?
     */
    display.scroll_disable();
}

// Apply Savitzky-Golay filter using fixed-point arithmetic
void WaterfallWidget::applySavitzkyGolay(const std::array<uint8_t, 240> spectrum_db_in,
                                         std::array<uint8_t, 240>& spectrum_db_out) {
    static const int SG_N = 5;  // Number of coefficients
    static const int coeffs[SG_N] = {-3, 12, 17, 12, -3};  // Coefficients for Savitzky-Golay filter
    size_t size = spectrum_db_in.size();
    spectrum_db_out[0] = spectrum_db_in[0];
    spectrum_db_out[1] = spectrum_db_in[1];
    spectrum_db_out[size - 2] = spectrum_db_in[size - 2];
    spectrum_db_out[size - 1] = spectrum_db_in[size - 1];

    for (size_t i = 2; i < size - 2; i++) {
        int32_t value = (spectrum_db_in[i - 2] * coeffs[0] +
                         spectrum_db_in[i - 1] * coeffs[1] +
                         spectrum_db_in[i] * coeffs[2] +
                         spectrum_db_in[i + 1] * coeffs[3] +
                         spectrum_db_in[i + 2] * coeffs[4]) >> 5;  // Normalize sum

        // Ensure value is within the range of 0-255
        spectrum_db_out[i] = (uint8_t)max(0, min(255, value));
    }
}

inline void spectrum_db_filter_median(const std::array<uint8_t, 240>& signal, std::array<uint8_t, 240>& filtered, 
    const size_t window_size = 5) {
    size_t size = window_size;
    if (window_size > 127) size = 127;
    if (window_size % 2 == 0) size++; // Забезпечити непарний розмір вікна

    int half_window = size / 2;
    for (size_t i = 0; i < signal.size(); ++i) {
        std::vector<uint8_t> window;
        for (int j = -half_window; j <= half_window; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < static_cast<int>(signal.size())) {
                window.push_back(signal[idx]);
            }
        }
        std::nth_element(window.begin(), window.begin() + window.size()/2, window.end());
        filtered[i] = window[window.size() / 2];
    }
}


void WaterfallWidget::on_channel_spectrum(const ChannelSpectrum& spectrum) {
    std::array<Color, 240> pixel_row;
    std::array<uint8_t, 240> spectrum_db;

    for (size_t i = 0; i < 120; i++) {
        spectrum_db[i] = spectrum.db[256 - 120 + i];
        spectrum_db[i + 120] = spectrum.db[i];
    }

    if (pmem::spectrum_view_type()) {
        std::array<uint8_t, 240> spectrum_db_upd = spectrum_db;
        uint8_t min = spectrum.min_db;
        uint8_t max = spectrum.max_db;
        // uint8_t noise_floor = estimateNoiseFloor(spectrum_db, min, max);

        switch (pmem::spectrum_view_type()) {
            case 1:
                applySavitzkyGolay(spectrum_db_upd, spectrum_db);  // Initialize with the first value
                break;
            case 2:
                spectrum_db_filter_median(spectrum_db_upd, spectrum_db, 5);
                // clearNoise(spectrum_db_upd, noise_floor, 15);
                // spectrum_db = spectrum_db_upd;
                break;
            case 3:
                for (size_t i = 0; i < 240; i++) {
                    spectrum_db[i] = logLUT[spectrum_db_upd[i]];
                }
                // clearNoise(spectrum_db, noise_floor, 15);
                // applySavitzkyGolay(spectrum_db, spectrum_db_upd);  // Initialize with the first value
                break;
        }
    }

    for (size_t i = 0; i < 240; i++) {
        pixel_row[i] = gradient.lut[spectrum_db[i]];
        // gradient.lut[spectrum_db[240 - 120 + i]];
        // pixel_row[i + 120] = gradient.lut[spectrum_db[i]];
    }

    const auto draw_y = display.scroll(1);

    display.draw_pixels(
        {{0, draw_y}, {pixel_row.size(), 1}},
        pixel_row);
}

bool WaterfallWidget::on_touch(const TouchEvent event) {
    if (event.type == TouchEvent::Type::Start) {
        if (on_touch_select) {
            on_touch_select(event.point.x(), event.point.y());
        }
    }
    uint8_t c = pmem::spectrum_view_type();
    c += 1;

    pmem::set_spectrum_view_type(c > 3 ? 0 : c);
    return true;
}

void WaterfallWidget::clear() {
    display.fill_rectangle(
        screen_rect(),
        Color::black());
}

/* WaterfallView *******************************************************/

WaterfallView::WaterfallView(const bool cursor) {
    add_children({&waterfall_widget,
                  &frequency_scale});

    frequency_scale.set_focusable(cursor);
    // Making the event climb up all the way up to here kinda sucks
    frequency_scale.on_select = [this](int32_t offset) {
        if (on_select) on_select(offset);
    };

    waterfall_widget.on_touch_select = [this](int32_t x, int32_t y) {
        if (y > screen_height - screen_height * 0.1) return;  // prevent ghost touch

        frequency_scale.focus();  // focus on frequency scale to show cursor

        if (sampling_rate) {
            // screen x to frequency scale x, NB we need two widgets align
            int32_t cursor_position = x - (screen_width / 2);
            frequency_scale.set_cursor_position(cursor_position);
        }
    };

    load_gradient();
}

void WaterfallView::load_gradient() {
    if (!waterfall_widget.gradient.load_file(default_gradient_file)) {
        waterfall_widget.gradient.set_default();
    }
}

void WaterfallView::on_show() {
    start();
}

void WaterfallView::on_hide() {
    stop();
}

void WaterfallView::start() {
    if (!running_) {
        baseband::spectrum_streaming_start();
        running_ = true;
    }
}

void WaterfallView::stop() {
    if (running_) {
        baseband::spectrum_streaming_stop();
        running_ = false;
    }
}

void WaterfallView::show_audio_spectrum_view(const bool show) {
    if ((audio_spectrum_view && show) || (!audio_spectrum_view && !show)) return;

    if (show) {
        audio_spectrum_view = std::make_unique<AudioSpectrumView>(audio_spectrum_view_rect);
        add_child(audio_spectrum_view.get());
        update_widgets_rect();
    } else {
        audio_spectrum_update = false;
        remove_child(audio_spectrum_view.get());
        audio_spectrum_view.reset();
        update_widgets_rect();
    }
}

void WaterfallView::update_widgets_rect() {
    if (audio_spectrum_view) {
        frequency_scale.set_parent_rect({0, audio_spectrum_height, screen_rect().width(), scale_height});
        waterfall_widget.set_parent_rect(waterfall_reduced_rect);
    } else {
        frequency_scale.set_parent_rect({0, 0, screen_rect().width(), scale_height});
        waterfall_widget.set_parent_rect(waterfall_normal_rect);
    }
    waterfall_widget.on_show();
}

void WaterfallView::set_parent_rect(const Rect new_parent_rect) {
    View::set_parent_rect(new_parent_rect);

    waterfall_normal_rect = {0, scale_height, new_parent_rect.width(), new_parent_rect.height() - scale_height};
    waterfall_reduced_rect = {0, audio_spectrum_height + scale_height, new_parent_rect.width(), new_parent_rect.height() - scale_height - audio_spectrum_height};

    update_widgets_rect();
}

void WaterfallView::on_channel_spectrum(const ChannelSpectrum& spectrum) {
    waterfall_widget.on_channel_spectrum(spectrum);
    sampling_rate = spectrum.sampling_rate;
    frequency_scale.set_spectrum_sampling_rate(sampling_rate);
    frequency_scale.set_channel_filter(
        spectrum.channel_filter_low_frequency,
        spectrum.channel_filter_high_frequency,
        spectrum.channel_filter_transition);
}

void WaterfallView::on_audio_spectrum() {
    audio_spectrum_view->on_audio_spectrum(audio_spectrum_data);
}

} /* namespace spectrum */

uint32_t filter_bandwidth_for_sampling_rate(int32_t sampling_rate) {
    switch (sampling_rate) {   // Use the var fs (sampling_rate) to set up BPF aprox < fs_max / 2 by Nyquist theorem.
        case 0 ... 3'500'000:  // BW Captured range BW (<=250K) : fs = 8x250k = 2000k, 16x150k = 2400k, 16x100k=1600k,
                               // 32x75k = 2400k, 32x50k=1600, 32x32k=1024, 64x25k = 1600k, 64x16k = 1024k, 64x12k5 = 800k.
            return 1'750'000;  // Minimum BPF MAX2837 for all those lower BW options.

        case 4'000'000 ... 7'000'000:  // OVS x8, BW capture range (500k...750kHz max) fs_max = 8 x 750k = 6Mhz
                                       // BW 500k...750kHz, ex. 500kHz (fs = 8 x BW = 4Mhz), BW 600kHz (fs = 4,8Mhz), BW 750 kHz (fs = 6Mhz).
            return 2'500'000;          // In some IC, MAX2837 appears as 2250000, but both work similarly.

        case 7'000'001 ... 10'000'000:  // OVS x8 and x4, BW capture 1Mhz fs = 8 x 1Mhz = 8Mhz. (1Mhz showed slightly higher noise background).
            return 3'500'000;           // some low SD cards, if not showing avg. writing speed >4MB/sec, they will produce sammples drop at REC with 1MB and C16 format.

        case 12'000'000 ... 14'000'000:  // OVS x4, BW capture 3Mhz, fs = 4 x 3Mhz = 12Mhz
                                         // Good BPF, good matching, we have some periodical M4 % samples drop.
            return 5'000'000;

        case 16'000'000:  // OVS x4, BW capture 4Mhz, fs = 4 x 4Mhz = 16Mhz
                          // Good BPF, good matching, we have some periodical M4 % samples drop.
            return 5'500'000;

        case 18'000'000:  // OVS x4, BW capture 4,5Mhz, fs = 4 x 4,5Mhz = 18Mhz
                          // Good BPF, good matching, we have some periodical M4 % samples drop.
            return 6'000'000;

        case 20'000'000:  // OVS x4, BW capture 5Mhz, fs = 4 x 5Mhz = 20Mhz
                          // Good BPF, good matching, we have some periodical M4 % samples drop.
            return 7'000'000;

        default:  // BW capture 5,5Mhz, fs = 4 x 5,5Mhz = 22Mhz max ADC sampling and others.
                  // We tested also 9Mhz FPB slightly too much noise floor, better at 8Mhz.
            return 8'000'000;
    }
}

} /* namespace ui */
