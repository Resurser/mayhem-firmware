/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2016 Furrtek
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

#include "proc_rtty_rx.hpp"

#include "portapack_persistent_memory.hpp"

#include "audio_dma.hpp"
#include "event_m4.hpp"
#include "math.h"
#include <complex>

#include <complex.h>
#include <vector>
#include <cmath>

#include "utility.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif

using namespace portapack;
using namespace modems;

RTTYRxProcessor::RTTYRxProcessor() {
    // decim_0.configure(taps_6k0_decim_0.taps);
    // decim_1.configure(taps_6k0_decim_1.taps);
    // channel_filter.configure(taps_2k8_lsb_channel.taps, 2);
    
    // // demod.configure(audio_fs, 5000);

    // audio_output.configure(audio_12k_hpf_300hz_config, audio_12k_deemph_300_6_config, 0);

    // samples_per_bit = audio_fs / message.baudrate;

    // phase_inc = (0x10000 * message.baudrate) / audio_fs;
    // phase = 0;

    // trigger_word = message.trigger_word;
    // word_length = message.word_length;
    // trigger_value = message.trigger_value;
    // word_mask = (1 << word_length) - 1;

    // // Delay line
    // delay_line_index = 0;

    // triggered = false;
    // state = WAIT_START;

    // configured = true;
}

void RTTYRxProcessor::execute(const buffer_c8_t& buffer) {
    // This is called at 3072000 / 2048 = 1500Hz

    if (!configured) return;

    // FM demodulation
    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);              // 2048 / 8 = 256 (512 I/Q samples)
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);         // 256 / 8 = 32 (64 I/Q samples)
    const auto channel_out = channel_filter.execute(decim_1_out, dst_buffer);  // 32 / 2 = 16 (32 I/Q samples)

    feed_channel_stats(channel_out);

    auto audio = demod.execute(channel_out, audio_buffer);

    audio_output.write(audio);
    
    std::vector<int> demodulated;
    int baud_samples = samples_per_bit; // 

    // Audio signal processing
    for (size_t c = 0; c < audio.count; c += baud_samples) {
        float mark_energy = 0;
        float space_energy = 0;

        for (int j = 0; j < baud_samples; ++j) {
            if (c + j < audio.count) {
                mark_energy += audio.p[c + j] * cos(2 * M_PI * freq_mark * j / audio_fs); // Mark frequency 2125 Hz
                space_energy += audio.p[c + j] * cos(2 * M_PI * freq_space * j / audio_fs); // Space frequency 2295 Hz
            }
        }

        if (mark_energy > space_energy) {
            demodulated.push_back(1);
        } else {
            demodulated.push_back(0);
        }
    }
    for (size_t i = 0; i < demodulated.size(); i += 5) {
        uint32_t code = 0;
        for (int j = 0; j < 5; ++j) {
            if (i + j < demodulated.size()) {
                code = (code << 1) | demodulated[i + j];
            }
        }

        data_message.is_data = true;
        data_message.value = code;
        shared_memory.application_queue.push(data_message);
    }
}

void RTTYRxProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::RTTYRxConfigure)
        configure(*reinterpret_cast<const RTTYRxConfigureMessage*>(message));
}

void RTTYRxProcessor::configure(const RTTYRxConfigureMessage& message) {
    configured = false;
    decim_0.configure(taps_11k0_decim_0.taps);
    decim_1.configure(taps_11k0_decim_1.taps);
    channel_filter.configure(taps_11k0_channel.taps, 2);
    // demod.configure(audio_fs, 5000);

    audio_output.configure(audio_24k_hpf_300hz_config, audio_24k_deemph_300_6_config, 0);

    samples_per_bit = audio_fs / message.baudrate;

    phase_inc = (0x10000 * message.baudrate) / audio_fs;
    phase = 0;

    trigger_word = 0;
    word_length = message.word_length;
    freq_mark = message.mark_freq;
    freq_space = message.space_freq;
    trigger_value = 0;
    word_mask = (1 << word_length) - 1;

    // Delay line
    delay_line_index = 0;

    triggered = false;
    state = WAIT_START;
    configured = true;
    
}

int main() {
    audio::dma::init_audio_out();

    EventDispatcher event_dispatcher{std::make_unique<RTTYRxProcessor>()};
    event_dispatcher.run();
    return 0;
}