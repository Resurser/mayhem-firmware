/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
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

#include "spectrum_collector.hpp"

#include "dsp_fft.hpp"

#include "utility.hpp"
#include "event_m4.hpp"
#include "portapack_shared_memory.hpp"

#include <algorithm>

void SpectrumCollector::on_message(const Message* const message) {
    switch (message->id) {
        case Message::ID::UpdateSpectrum:
            update();
            break;

        case Message::ID::SpectrumStreamingConfig:
            set_state(*reinterpret_cast<const SpectrumStreamingConfigMessage*>(message));
            break;

        default:
            break;
    }
}

void SpectrumCollector::set_state(const SpectrumStreamingConfigMessage& message) {
    if (message.mode == SpectrumStreamingConfigMessage::Mode::Running) {
        start();
    } else {
        stop();
    }
}

void SpectrumCollector::start() {
    streaming = true;
    ChannelSpectrumConfigMessage message{&fifo};
    shared_memory.application_queue.push(message);
}

void SpectrumCollector::stop() {
    streaming = false;
    fifo.reset_in();
}

void SpectrumCollector::set_decimation_factor(
    const size_t decimation_factor) {
    channel_spectrum_decimator.set_factor(decimation_factor);
}

/* TODO: Refactor to register task with idle thread?
 * It's sad that the idle thread has to call all the way back here just to
 * perform the deferred task on the buffer of data we prepared.
 */

void SpectrumCollector::feed(
    const buffer_c16_t& channel,
    const int32_t filter_low_frequency,
    const int32_t filter_high_frequency,
    const int32_t filter_transition) {
    // Called from baseband processing thread.
    channel_filter_low_frequency = filter_low_frequency;
    channel_filter_high_frequency = filter_high_frequency;
    channel_filter_transition = filter_transition;

    channel_spectrum_decimator.feed(
        channel,
        [this](const buffer_c16_t& data) {
            this->post_message(data);
        });
}

void SpectrumCollector::post_message(const buffer_c16_t& data) {
    // Called from baseband processing thread.
    if (streaming && !channel_spectrum_request_update) {
        fft_swap(data, channel_spectrum);
        channel_spectrum_sampling_rate = data.sampling_rate;
        channel_spectrum_request_update = true;
        EventDispatcher::events_flag(EVT_MASK_SPECTRUM);
    }
}

template <typename T>
static typename T::value_type spectrum_window_none(const T& s, const size_t i) {
    constexpr size_t length = sizeof(s) / sizeof(s[0]);
    static_assert(power_of_two(length), "Array length must be power of 2");
    return s[i];
};

template <typename T>
static typename T::value_type spectrum_window_hamming_3(const T& s, const size_t i) {
    constexpr size_t length = sizeof(s) / sizeof(s[0]);
    static_assert((length), "Array length must be power of 2");
    constexpr size_t mask = length - 1;
    // Three point Hamming window.
    const auto prev = s[(i - 1) & mask];
    const auto next = s[(i + 1) & mask];
    return s[i] * 0.54f + (prev + next) * -0.23f;
};

template <typename T>
static typename T::value_type spectrum_window_blackman_3(const T& s, const size_t i) {
    constexpr size_t length = sizeof(s) / sizeof(s[0]);
    static_assert(power_of_two(length), "Array length must be power of 2");
    constexpr size_t mask = length - 1;
    // Three term Blackman window.
    constexpr float alpha = 0.42f;
    constexpr float beta = 0.5f * 0.5f;
    constexpr float gamma = 0.08f * 0.08f;
    return s[i] * alpha - (s[(i - 1) & mask] + s[(i + 1) & mask]) * beta + (s[(i - 2) & mask] + s[(i + 2) & mask]) * gamma;
};

static void spectrum_db_filter_gausian(const std::array<uint8_t, 256>& signal, std::array<uint8_t, 256>& filtered, uint8_t kernel = 4){
    int half_kernel = kernel / 2;
    for (size_t i = 0; i < signal.size(); ++i) {
        int sum = 0, count = 0;
        for (int j = -half_kernel; j <= half_kernel; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < static_cast<int>(signal.size())) {
                sum += signal[idx];
                count++;
            }
        }
        filtered[i] = sum / count;
    }
}
static void spectrum_db_filter_wiener(const std::array<uint8_t, 256>& signal, std::array<uint8_t, 256>& filtered, const size_t window_size = 5) {
    int half_window = window_size / 2;
    for (size_t i = 0; i < 256; ++i) {
        float localMean = 0, localVariance = 0;
        int count = 0;
        for (int j = -half_window; j <= half_window; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < 256) {
                localMean += signal[idx];
                count++;
            }
        }
        localMean /= count;
        for (int j = -half_window; j <= half_window; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < 256) {
                localVariance += std::pow(signal[idx] - localMean, 2);
            }
        }
        localVariance /= count;
        float noiseVariance = 1;  // Assumed or precomputed
        float gain = (localVariance - noiseVariance) / localVariance;
        gain = std::max(0.0f, gain);  // Ensure non-negative gain
        filtered[i] = static_cast<uint8_t>(localMean + gain * (signal[i] - localMean));
    }
}

static void spectrum_db_filter_median(const std::array<uint8_t, 256>& signal, std::array<uint8_t, 256>& filtered, const size_t window_size = 5) {
    size_t size = window_size;
    if (window_size > 127) size = 127;
    if (window_size % 2 == 0) size++; // Забезпечити непарний розмір вікна

    int halfWindow = size / 2;
    for (size_t i = 0; i < signal.size(); ++i) {
        std::vector<uint8_t> window;
        for (int j = -halfWindow; j <= halfWindow; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < static_cast<int>(signal.size())) {
                window.push_back(signal[idx]);
            }
        }
        std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
        filtered[i] = window[window.size() / 2];
    }
}


void SpectrumCollector::update() {
    // Called from idle thread (after EVT_MASK_SPECTRUM is flagged)
    if (streaming && channel_spectrum_request_update) {
        /* Decimated buffer is full. Compute spectrum. */
        fft_c_preswapped(channel_spectrum, 0, 8);
        
        ChannelSpectrum spectrum;
        spectrum.sampling_rate = channel_spectrum_sampling_rate;
        spectrum.channel_filter_low_frequency = channel_filter_low_frequency;
        spectrum.channel_filter_high_frequency = channel_filter_high_frequency;
        spectrum.channel_filter_transition = channel_filter_transition;
        
        for (size_t i = 0; i < spectrum.db.size(); i++) {
            // const auto corrected_sample = spectrum_window_hamming_3(channel_spectrum, i);
            const auto corrected_sample = spectrum_window_blackman_3(channel_spectrum, i);
            const auto mag2 = magnitude_squared(corrected_sample * (1.0f / 32768.0f));
            const float db = mag2_to_dbv_norm(mag2);
            constexpr float mag_scale = 5.0f;// 5.0f;
            const unsigned int v = (db * mag_scale) + 255.0f;
        
            spectrum.db[i] = std::max(0U, std::min(255U, v));
        }
        
        std::array<uint8_t, 256> db_filtered;
        // spectrum_db_filter_median(spectrum.db, db_filtered, 3);
        // spectrum_db_filter_gausian(spectrum.db, db_filtered, 2);
        spectrum_db_filter_gausian(spectrum.db, db_filtered, 5);
        spectrum.db = db_filtered;
        
        fifo.in(spectrum);
    }

    channel_spectrum_request_update = false;
}
