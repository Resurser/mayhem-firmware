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
#include <cstring>

#include "utility.hpp"

// --- Константи ITA-2 ---
// constexpr uint8_t ITA2_LTRS_SHIFT_CODE = 0x1F;  // 11111
// constexpr uint8_t ITA2_FIGS_SHIFT_CODE = 0x1B;  // 11011
// constexpr uint8_t ITA2_CODE_MASK = 0x1F;        // Маска для 5 біт
// Для константи M_PI
#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// Таблиця кодів Baudot для букв (Letters shift)
static const char baudot_ltrs[] = { 0, 'E', '\n', 'A', ' ', 'S', 'I', 'U', '\r', 'D', 'R', 'J', 'N', 'F', 'C', 'K', 'T', 'Z', 'L', 'W', 'H', 'Y', 'P', 'Q', 'O', 'B', 'G', 0, 'M', 'X', 'V', 0 };
// Таблиця кодів Baudot для цифр та знаків (Figures shift)
static const char baudot_figs[] = { 0, '3', '\n', '-', ' ', 0x7, '8', '7', '\r', '$', '4', 0x27, ',', '!', ':', '(', '5', '+', ')', '2', '#', '6', '0', '1', '9', '?', '&', 0, '.', '/', '=', 0 };

RTTYRxProcessor::RTTYRxProcessor() {
    decim_0.configure(taps_6k0_decim_0.taps);
    decim_1.configure(taps_6k0_decim_1.taps);
    decim_2.configure(taps_6k0_decim_2.taps, 4);
    channel_filter.configure(taps_2k8_usb_channel.taps, 1);
    audio_output.configure(audio_12k_hpf_300hz_config);  //, audio_12k_deemph_300_6_config);
}

void RTTYRxProcessor::execute(const buffer_c8_t& buffer) {
    
    // SSB demodulation
    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);              // 2048 / 8 = 256 (512 I/Q samples)
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);         // 256 / 8 = 32 (64 I/Q samples)
    const auto decim_2_out = decim_2.execute(decim_1_out, dst_buffer);         // 256 / 8 = 32 (64 I/Q samples)
    const auto channel_out = channel_filter.execute(decim_2_out, dst_buffer);  // 32 / 2 = 16 (32 I/Q samples)

    feed_channel_stats(channel_out);

    auto audio = demod.execute(channel_out, audio_buffer);
    audio_output.write(audio);
    // for (size_t i = 0; i < 128; i++) {
    //     re = buffer.p[i].real();
    //     // im = buffer.p[i].imag();
    //     // mag = __builtin_sqrtf((re * re) + (im * im)) ;
    //     const unsigned int v = re + 127.0f;  // timescope
    //     audio_spectrum.db[i] = std::max(0U, std::min(255U, v));
    // }
    // AudioSpectrumMessage message{&audio_spectrum};
    // shared_memory.application_queue.push(message);
    for (size_t c = 0; c < audio.count; c++) {
        // Scale and saturate the sample
        const int32_t sample_int = audio.p[c] * 32768.0f;

        int32_t current_sample = __SSAT(sample_int, 16);  // Scale to Q15 format
        process_sample_pair(current_sample, channel_out.p[c]);
    }
}
// --- Обробка одного семпла (12кГц) ---
void RTTYRxProcessor::process_sample_pair(int16_t audio_sample, complex16_t iq_sample) {
    // 1. Нормалізація амплітуди
    float s = (float)audio_sample / 32768.0f;

    // 2. Goertzel (Амплітудний детектор)
    float s0_m = s + coeff_mark * s1_mark - s2_mark;
    s2_mark = s1_mark; s1_mark = s0_m;

    float s0_s = s + coeff_space * s1_space - s2_space;
    s2_space = s1_space; s1_space = s0_s;

    float mag_sq_mark = s1_mark * s1_mark + s2_mark * s2_mark - coeff_mark * s1_mark * s2_mark;
    float mag_sq_space = s1_space * s1_space + s2_space * s2_space - coeff_space * s1_space * s2_space;

    // Статистика для UI
    mark_energy_accumulator += std::min(mag_sq_mark, 100.0f);
    space_energy_accumulator += std::min(mag_sq_space, 100.0f);
    
    // 3. AFC: Розрахунок миттєвої частоти через Delta Phase
    if (afc_enabled) {
        float re = (float)iq_sample.real() * last_iq_sample.real() + (float)iq_sample.imag() * last_iq_sample.imag();
        float im = (float)iq_sample.imag() * last_iq_sample.real() - (float)iq_sample.real() * last_iq_sample.imag();
        
        float phase_delta = std::atan2(im, re);
        float inst_freq = (phase_delta * (float)sample_rate) / (2.0f * (float)M_PI);

        measured_freq_accumulator += inst_freq;
        measured_samples_count++;

        last_iq_sample = iq_sample;
    }

    sample_count++;
    stats_send_counter++;
    afc_update_counter++;

    // Періодична відправка статистики енергії
    if (stats_send_counter >= STATS_SEND_PERIOD) {
        send_stats();
        stats_send_counter = 0;
        mark_energy_accumulator = space_energy_accumulator = 0.0f;
    }

    // Періодичне застосування AFC
    if (afc_update_counter >= AFC_UPDATE_PERIOD) {
        apply_afc();
        afc_update_counter = 0;
    }
    static constexpr float SQUELCH_THRESHOLD = 0.005f; 


    // --- RTTY State Machine ---
    if (state == IDLE) {
        if (sample_count >= (samples_per_bit / 4)) {
            if ((mag_sq_space > SQUELCH_THRESHOLD) && (mag_sq_space > mag_sq_mark)) { // Start bit (Space) detected
                state = DATA;
                bit_buffer = bits_received = sample_count = 0;
                s1_mark = s2_mark = s1_space = s2_space = 0;
                measured_freq_accumulator = 0;
                measured_samples_count = 0;
            } else {
                sample_count = 0;
                s1_mark = s2_mark = s1_space = s2_space = 0;
            }
        }
    } 
    else {
        if (sample_count >= samples_per_bit) {
            bool bit = (mag_sq_mark > mag_sq_space);
            
            // --- Збір даних для AFC ---
            if (afc_enabled && measured_samples_count > 0) {
                float avg_freq = measured_freq_accumulator / measured_samples_count;
                
                if (bit && (mag_sq_mark > 0.001f)) { 
                    afc_mark_sum += avg_freq;
                    afc_mark_count++;
                } 
                else if (!bit && (mag_sq_space > 0.001f)) {
                    afc_space_sum += avg_freq;
                    afc_space_count++;
                }
            }
            
            measured_freq_accumulator = 0;
            measured_samples_count = 0;

            handle_bit(bit);
            
            sample_count = 0;
            s1_mark = s2_mark = 0;
            s1_space = s2_space = 0;
        }
    }
}

void RTTYRxProcessor::apply_afc() {
    if (!afc_enabled) return;
    const float alpha = 0.1f; 
    bool updated = false;

    if (afc_mark_count > 5) { 
        float measured_mark = afc_mark_sum / afc_mark_count;
        if (std::abs(measured_mark - (float)mark_freq) < 200.0f) {
            mark_freq = (uint32_t)((float)mark_freq * (1.0f - alpha) + measured_mark * alpha);
            updated = true;
        }
    }

    if (afc_space_count > 5) {
        float measured_space = afc_space_sum / afc_space_count;
        if (std::abs(measured_space - (float)space_freq) < 200.0f) {
            space_freq = (uint32_t)((float)space_freq * (1.0f - alpha) + measured_space * alpha);
            updated = true;
        }
    }

    afc_mark_sum = 0; afc_mark_count = 0;
    afc_space_sum = 0; afc_space_count = 0;

    if (updated) {
        update_coeffs();
    }
}

void RTTYRxProcessor::send_stats() {
    uint32_t scaled_mark = (uint32_t)((mark_energy_accumulator / (float)STATS_SEND_PERIOD) * 1000.0f);
    uint32_t scaled_space = (uint32_t)((space_energy_accumulator / (float)STATS_SEND_PERIOD) * 1000.0f);
    RTTYStatsMessage msg(scaled_mark, scaled_space);
    shared_memory.application_queue.push(msg);
}

void RTTYRxProcessor::handle_bit(bool bit) {
     if (state == DATA) {
        if (bit) bit_buffer |= (1 << bits_received);
        bits_received++;
        if (bits_received >= 5) state = STOP;
    } else if (state == STOP) {
        if (bit) decode_baudot(bit_buffer);
        state = IDLE;
    }
}

void RTTYRxProcessor::decode_baudot(uint8_t bits) {
    if (bits == 0x1B) { shift_figs = true; return; }
    if (bits == 0x1F) { shift_figs = false; return; }
    char c = shift_figs ? baudot_figs[bits] : baudot_ltrs[bits];
    if (c != 0) {
        RTTYCharMessage msg(c);
        shared_memory.application_queue.push(msg);
    }
}

void RTTYRxProcessor::update_coeffs() {
    const float k = 2.0f * 3.1415926535f / (float)sample_rate;
    coeff_mark = 2.0f * cosf(k * (float)mark_freq);
    coeff_space = 2.0f * cosf(k * (float)space_freq);
}

void RTTYRxProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::RTTYConfig) {
        const auto* config = reinterpret_cast<const RTTYConfigMessage*>(message);
        mark_freq = config->mark_freq;
        space_freq = config->space_freq;
        baud_rate = (float)config->baud_rate;
        if (baud_rate > 0) samples_per_bit = sample_rate / baud_rate;
        update_coeffs();
    }
}

void RTTYRxProcessor::configure(const RTTYConfigMessage& message) {
    mark_freq = message.mark_freq;
    space_freq = message.space_freq;
    baud_rate = (float)message.baud_rate;
    if (baud_rate > 0) {
        samples_per_bit = sample_rate / baud_rate;
    }
    
    update_coeffs();
}

int main() {
    audio::dma::init_audio_out();

    EventDispatcher event_dispatcher{std::make_unique<RTTYRxProcessor>()};
    event_dispatcher.run();
    return 0;
}