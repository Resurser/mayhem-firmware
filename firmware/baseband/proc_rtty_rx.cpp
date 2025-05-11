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

#include "utility.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif

RTTYRxProcessor::RTTYRxProcessor() {
    // decim_0.configure(taps_200k_decim_0.taps);
    // decim_1.configure(taps_16k0_decim_1.taps);
    // channel_filter.configure(taps_11k0_channel.taps, 2);
    // audio_output.configure(audio_24k_hpf_300hz_config);

    // samples_per_bit = audio_fs / BAUD_RATE;

    // phase_inc  = (0x10000 * BAUD_RATE) / audio_fs;
    // phase      = 0;
    // freq_mark  = MARK_FREQ;
    // freq_space = SPACE_FREQ;

    // trigger_word  = 0;
    // word_length   = 5;
    // trigger_value = 0;
    // word_mask     = (1 << word_length) - 1;

    // // Delay line
    // delay_line_index = 0;
    // triggered        = false;
    // state            = WAIT_START;

    // configured = true;
}
// Обробка одного аудіосигналу
void RTTYRxProcessor::processRTTYBit(int16_t sample) {
    bool currentSign = (sample >= 0);

    if (currentSign != lastSign) {
        zeroCrossings++;
        lastSign = currentSign;
    }

    measureSignalAmplitude(sample); // Фільтрація шуму

    if (++sampleCount >= SAMPLES_PER_BIT) {
        bool bit = (zeroCrossings > adaptiveThreshold / (SAMPLE_RATE / SAMPLES_PER_BIT));

        // Оновлення порогового значення
        for (int i = ADAPTIVE_WINDOW_SIZE - 1; i > 0; --i) {
            zeroCrossHistory[i] = zeroCrossHistory[i - 1];
        }
        zeroCrossHistory[0] = zeroCrossings;
        updateAdaptiveThreshold();

        // Коригування частоти (AFC)
        adaptiveFrequencyCorrection();

        if (!isStartBit) {
            if (!bit) {
                isStartBit = true;
                zeroCrossings = 0;
                sampleCount = 0;
            }
            return;
        }

        currentChar >>= 1;
        if (bit) currentChar |= 0x10;

        if (++bitCount == 5) {
            stopBitCount = 0;
            sampleCount = 0;
            return;
        }

        if (bitCount == 5 && ++stopBitCount >= SAMPLES_STOP_BITS) {
            char decodedChar = BAUDOT_LETTERS[currentChar & 0x1F];
            if (decodedChar != '\0') decodedMessage += decodedChar;

            gui_clear();
            gui_draw_text(10, 10, decodedMessage.c_str(), GUI_COLOR_WHITE, GUI_COLOR_BLACK);

            currentChar = 0;
            bitCount = 0;
            isStartBit = false;
        }

        zeroCrossings = 0;
        sampleCount = 0;
    }
}

void RTTYRxProcessor::rtty_process_bit_decision(bool is_mark) {
    char decoded_char = 0;

    switch (current_rtty_state) {
        case RTTY_STATE_IDLE:
            if (!is_mark) {
                current_rtty_state = RTTY_STATE_START_BIT;
                rtty_sample_counter = 0;
            }
            break;

        case RTTY_STATE_START_BIT:
            rtty_sample_counter++;
            if (rtty_sample_counter >= samples_per_half_bit) {
                if (!is_mark) {
                    current_rtty_state = RTTY_STATE_DATA_BITS;
                    bit_counter = 0;
                    current_rtty_byte = 0;
                    rtty_sample_counter = samples_per_half_bit;
                } else {
                    current_rtty_state = RTTY_STATE_IDLE;
                }
            }
            break;

        case RTTY_STATE_DATA_BITS:
            rtty_sample_counter++;
            if (rtty_sample_counter >= samples_per_bit) {
                rtty_sample_counter = 0;
                if (is_mark) {
                    current_rtty_byte |= (1 << bit_counter);
                }
                bit_counter++;
                if (bit_counter >= 5) {
                    current_rtty_state = RTTY_STATE_STOP_BITS;
                    rtty_sample_counter = 0;
                }
            }
            break;

        case RTTY_STATE_STOP_BITS:
            rtty_sample_counter++;
             if (!is_mark && rtty_sample_counter < samples_per_stop_bit) {
                 current_rtty_state = RTTY_STATE_IDLE;
            }
            else if (rtty_sample_counter >= samples_per_stop_bit) {
                if (is_mark) {                    
                    // send to UI 
                    data_message.is_data = true;
                    data_message.value = current_rtty_byte;
                    shared_memory.application_queue.push(data_message);
                } else {
                   // Помилка стоп-біта
                }
                current_rtty_state = RTTY_STATE_IDLE;
            }
            break;
    }
    // return decoded_char;
}

void RTTYRxProcessor::execute(const buffer_c8_t& buffer) {
    // This is called at 3072000 / 2048 = 1500Hz
    if (!configured) return;

    // SSB demodulation
    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);              // 2048 / 8 = 256 (512 I/Q samples)
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);         // 256 / 8 = 32 (64 I/Q samples)
    const auto decim_2_out = decim_2.execute(decim_1_out, dst_buffer);         // 256 / 8 = 32 (64 I/Q samples)
    const auto channel_out = channel_filter.execute(decim_2_out, dst_buffer);  // 32 / 2 = 16 (32 I/Q samples)

    feed_channel_stats(channel_out);

    auto audio = demod.execute(channel_out, audio_buffer);
    audio_output.write(audio);

    std::vector<int> demodulated;
    int baud_samples = samples_per_bit;  //

    // Audio signal processing
    for (size_t c = 0; c < audio.count; c += baud_samples) {
        float mark_energy = 0;
        float space_energy = 0;

        for (int j = 0; j < baud_samples; ++j) {
            if (c + j < audio.count) {
                mark_energy += audio.p[c + j] * cos(2 * M_PI * freq_mark * j / audio_fs);    // Mark frequency 2125 Hz
                space_energy += audio.p[c + j] * cos(2 * M_PI * freq_space * j / audio_fs);  // Space frequency 2295 Hz
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

        if (code > 0) {
            // data_message.is_data = true;
            // data_message.value = code;
            // shared_memory.application_queue.push(data_message);
        } 
    }
}

void RTTYRxProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::RTTYRxConfigure)
        configure(*reinterpret_cast<const RTTYRxConfigureMessage*>(message));
}

void RTTYRxProcessor::configure(const RTTYRxConfigureMessage& message) {
    configured = false;
    decim_0.configure(taps_6k0_decim_0.taps);
    decim_1.configure(taps_6k0_decim_1.taps);
    decim_2.configure(taps_6k0_decim_2.taps,4);
    channel_filter.configure(taps_2k8_lsb_channel.taps, 1);
    audio_output.configure(audio_12k_hpf_300hz_config);
    samples_per_bit = audio_fs / message.baudrate;

    phase_inc = (0x10000 * message.baudrate) / audio_fs;
    phase = 0;

    trigger_word = 0;
    word_length  = message.word_length;
    // freq_mark = message.mark_freq;
    // freq_space = message.space_freq;
    trigger_value = 0;
    word_mask     = (1 << word_length) - 1;

    // Delay line
    delay_line_index = 0;

    triggered  = false;
    state      = WAIT_START;
    configured = true;
}

int main() {
    audio::dma::init_audio_out();

    EventDispatcher event_dispatcher{std::make_unique<RTTYRxProcessor>()};
    event_dispatcher.run();
    return 0;
}