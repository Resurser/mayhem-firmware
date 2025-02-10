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
#include "portapack_shared_memory.hpp"
#include "audio_dma.hpp"
#include "event_m4.hpp"
#include "math.h"
#include <complex>

#include <complex.h>
#include <vector>
#include <cmath>

#include "dsp_goertzel.hpp"
#include "utility.hpp"

#define SAMPLE_RATE 24000 // 24 кГц
#define BAUD_RATE 45.45
#define MARK_FREQ 2125
#define SPACE_FREQ 2295
#define SAMPLES_PER_BIT (SAMPLE_RATE / BAUD_RATE)
#define BUFFER_SIZE 8192


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
    
    // SSB demodulation
    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);              // 2048 / 8 = 256 (512 I/Q samples)
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);         // 256 / 8 = 32 (64 I/Q samples)
    const auto channel_out = channel_filter.execute(decim_1_out, dst_buffer);  // 32 / 2 = 16 (32 I/Q samples)

    feed_channel_stats(channel_out);
    auto audio = demod.execute(channel_out, audio_buffer);

    audio_output.write(audio);
    // try decoder ...
    float mark_coeff, space_coeff;
    float mark_q0, mark_q1, mark_q2;
    float space_q0, space_q1, space_q2;

    init_goertzel(SAMPLE_RATE, MARK_FREQ, &mark_coeff, &mark_q0, &mark_q1, &mark_q2);
    init_goertzel(SAMPLE_RATE, SPACE_FREQ, &space_coeff, &space_q0, &space_q1, &space_q2);

    int bit_counter = 0;
    uint8_t byte = 0;
    int bit_index = 0;
    int mark_count = 0;
    int space_count = 0;

    std::string decoded_text = "";

    for (size_t c = 0; c < audio.count; c++) {
            // const int32_t sample_int = audio.p[c] * 32768.0f;
            // int32_t current_sample = __SSAT(sample_int, 16);
            //  /= 128;
        float sample = (float)((audio.p[c] * 32768.0f) / 127.0);

        float mark_power = run_goertzel(mark_coeff, &mark_q0, &mark_q1, &mark_q2, sample);
        float space_power = run_goertzel(space_coeff, &space_q0, &space_q1, &space_q2, sample);

        if (mark_power > space_power) {
            mark_count++;
        } else {
            space_count++;
        }

        if (++bit_counter >= SAMPLES_PER_BIT) {
            if (mark_count > space_count) {
                byte |= (1 << bit_index);
            }

            bit_counter = 0;
            mark_count = 0;
            space_count = 0;
            bit_index++;

            if (bit_index >= 5) { // 5-бітові слова
                data_message.is_data = true;
                data_message.value = byte;
                shared_memory.application_queue.push(data_message);
                byte = 0;
                bit_index = 0;
            }
        }
    }
    // Audio signal processing
}

void RTTYRxProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::RTTYRxConfigure)
        configure(*reinterpret_cast<const RTTYRxConfigureMessage*>(message));
}

void RTTYRxProcessor::configure(const RTTYRxConfigureMessage& message) {
    decim_0.configure(taps_6k0_decim_0.taps);
    decim_1.configure(taps_6k0_decim_1.taps);
    channel_filter.configure(taps_2k8_lsb_channel.taps, 4);
    
    audio_output.configure(audio_12k_hpf_300hz_config, audio_12k_deemph_300_6_config, 0);
    // audio_output.configure(audio_12k_hpf_300hz_config);
    
    samples_per_bit = baseband_fs / message.baudrate;
    word_length     = message.word_length;
    baud_rate       = message.baudrate;

    configured = true;    
}

int main() {
    audio::dma::init_audio_out();

    EventDispatcher event_dispatcher{std::make_unique<RTTYRxProcessor>()};
    event_dispatcher.run();
    return 0;
}