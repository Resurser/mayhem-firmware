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

#ifndef __UI_SPECTRUM_H__
#define __UI_SPECTRUM_H__

#include "gradient.hpp"
#include "ui.hpp"
#include "ui_widget.hpp"

#include "event_m0.hpp"

#include "message.hpp"

#include <cstddef>
#include <cstdint>

namespace ui {
namespace spectrum {

class AudioSpectrumView : public View {
   public:
    AudioSpectrumView(const Rect parent_rect);

    void paint(Painter& painter) override;

    void on_audio_spectrum(const AudioSpectrum* spectrum);

   private:
    static constexpr int cursor_band_height = 4;

    int16_t audio_spectrum[128]{0};

    Labels labels{
        {{6 * 8, 0 * 16}, "Hz", Theme::getInstance()->fg_light->foreground}};

    NumberField field_frequency{
        {0 * 8, 0 * 16},
        5,
        {0, 48000},
        48000 / 240,
        ' '};

    Waveform waveform{
        {0, 1 * 16 + cursor_band_height, screen_width, 2 * 16},
        audio_spectrum,
        128,
        0,
        false,
        Theme::getInstance()->bg_darkest->foreground,
        true};
};

class FrequencyScale : public Widget {
   public:
    std::function<void(int32_t offset)> on_select{};

    void on_show() override;
    void on_focus() override;
    void on_blur() override;

    bool on_encoder(const EncoderEvent delta) override;
    bool on_key(const KeyEvent key) override;
    bool on_touch(const TouchEvent touch) override;

    void set_spectrum_sampling_rate(const int new_sampling_rate);
    void set_channel_filter(const int low_frequency, const int high_frequency, const int transition);
    void set_ddc_freq(const int freq);
    void set_cursor_position(const int32_t position);

    void paint(Painter& painter) override;

   private:
    static constexpr int filter_band_height = 4;

    int32_t cursor_position{0};
    int spectrum_sampling_rate{0};
    const int spectrum_bins = std::tuple_size<decltype(ChannelSpectrum::db)>::value;
    int channel_filter_low_frequency{0};
    int channel_filter_high_frequency{0};
    int channel_filter_transition{0};
    int ddc_freq{0};

    void clear();
    void clear_background(Painter& painter, const Rect r);

    void draw_frequency_ticks(Painter& painter, const Rect r);
    void draw_filter_ranges(Painter& painter, const Rect r);
};

/* NB: These visualizations rely on having a baseband image running.
 * If the baseband is shutdown or otherwise not running when interacting
 * with these, they will almost certainly hang the device. */

class WaterfallWidget : public Widget {
   public:
    std::function<void(int32_t offset, int32_t y)> on_touch_select{};

    Gradient gradient{};

    void on_show() override;
    void on_hide() override;
    void paint(Painter&) override {}
    bool on_touch(const TouchEvent event) override;

    void on_channel_spectrum(const ChannelSpectrum& spectrum);

   private:
    const uint8_t logLUT[256] = {
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
        252,
        253,
        253,
        253,
        253,
        254,
        254,
        254,
        254,
        255,
        255};

    // Function to apply linear normalization with noise floor compensation
    void applySavitzkyGolay(std::array<unsigned char, 240> spectrum_db_in, std::array<unsigned char, 240>& spectrum_db_out);

    void clear();
};

class WaterfallView : public View {
   public:
    std::function<void(int32_t offset)> on_select{};

    WaterfallView(const bool cursor = false);

    WaterfallView(const WaterfallView&) = delete;
    WaterfallView(WaterfallView&&) = delete;
    WaterfallView& operator=(const WaterfallView&) = delete;
    WaterfallView& operator=(WaterfallView&&) = delete;

    // TODO: remove these, use start/stop directly instead.
    void on_show() override;
    void on_hide() override;

    void start();
    void stop();

    void set_parent_rect(const Rect new_parent_rect) override;
    void show_audio_spectrum_view(const bool show);
    void load_gradient();

   private:
    void update_widgets_rect();

    const Rect audio_spectrum_view_rect{0 * 8, 0 * 16, screen_width, 2 * 16 + 20};
    static constexpr Dim audio_spectrum_height = 16 * 2 + 20;
    static constexpr Dim scale_height = 20;

    WaterfallWidget waterfall_widget{};
    FrequencyScale frequency_scale{};
    bool running_{false};

    ChannelSpectrumFIFO* channel_fifo{nullptr};
    AudioSpectrum* audio_spectrum_data{nullptr};
    bool audio_spectrum_update{false};

    std::unique_ptr<AudioSpectrumView> audio_spectrum_view{};

    int sampling_rate{0};
    int32_t cursor_position{0};
    ui::Rect waterfall_normal_rect{};
    ui::Rect waterfall_reduced_rect{};

    MessageHandlerRegistration message_handler_channel_spectrum_config{
        Message::ID::ChannelSpectrumConfig,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const ChannelSpectrumConfigMessage*>(p);
            this->channel_fifo = message.fifo;
        }};

    MessageHandlerRegistration message_handler_audio_spectrum{
        Message::ID::AudioSpectrum,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const AudioSpectrumMessage*>(p);
            this->audio_spectrum_data = message.data;
            this->audio_spectrum_update = true;
        }};

    MessageHandlerRegistration message_handler_ddc_config{
        Message::ID::DDCConfig,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const DDCConfigMessage*>(p);
            this->frequency_scale.set_ddc_freq(message.freq);
        }};

    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            if (this->channel_fifo) {
                ChannelSpectrum channel_spectrum;
                while (channel_fifo->out(channel_spectrum)) {
                    this->on_channel_spectrum(channel_spectrum);
                }
            }
            if (this->audio_spectrum_update) {
                this->audio_spectrum_update = false;
                this->on_audio_spectrum();
            }
        }};

    void on_channel_spectrum(const ChannelSpectrum& spectrum);
    void on_audio_spectrum();
};

} /* namespace spectrum */

/* Calculates the best anti_alias_baseband_bandwidth_filter for the given sampling rate. */
uint32_t filter_bandwidth_for_sampling_rate(int32_t sampling_rate);

} /* namespace ui */

#endif /*__UI_SPECTRUM_H__*/
