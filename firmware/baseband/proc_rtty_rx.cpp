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
#include "utility.hpp"
const double MARK_FREQ = 2125.0; 
const double SPACE_FREQ = 2295.0; 
const double SAMPLE_RATE = 48000.0; 
const double BAUD_RATE = 45.45; 
const double PI = 3.14159265358979323846;

std::unordered_map<int, char> baudotToAsciiMap = { {0b00000, ' '}, {0b00001, 'E'}, {0b00010, '\n'}, {0b00011, 'A'}, {0b00100, 'S'}, {0b00101, 'I'}, {0b00110, 'U'}, {0b00111, 'D'}, {0b01000, 'R'}, {0b01001, 'J'}, {0b01010, 'N'}, {0b01011, 'F'}, {0b01100, 'C'}, {0b01101, 'K'}, {0b01110, 'T'}, {0b01111, 'Z'}, {0b10000, 'L'}, {0b10001, 'W'}, {0b10010, 'H'}, {0b10011, 'Y'}, {0b10100, 'P'}, {0b10101, 'Q'}, {0b10110, 'O'}, {0b10111, 'B'}, {0b11000, 'G'}, {0b11001, 'M'}, {0b11010, 'X'}, {0b11011, 'V'}, {0b11100, 'K'}, {0b11101, ' '}, {0b11110, '\r'}, {0b11111, 'Q'} };
std::string decodeRTTY(const std::vector<int>& fskTones) { 
    std::string decodedText; 
    size_t bitsPerChar = 5; 
    size_t toneIndex = 0; 
    while (toneIndex + bitsPerChar <= fskTones.size()) { 
        int character = 0; 
        for (size_t bit = 0; bit < bitsPerChar; ++bit) { 
            character |= (fskTones[toneIndex + bit] << (bitsPerChar - 1 - bit)); 
        } 
        if (baudotToAsciiMap.find(character) != baudotToAsciiMap.end()) { 
            decodedText += baudotToAsciiMap[character]; 
        } else { 
            decodedText += '?'; // Unknown character 
        } 
        toneIndex += bitsPerChar; 
    } 
    return decodedText; 
}
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

    // Audio signal processing
    for (size_t c = 0; c < audio.count; c++) {
        const int32_t sample_int = audio.p[c] * 32768.0f;
        int32_t current_sample = __SSAT(sample_int, 16);

        current_sample /= 128;

        // Delay line put
        delay_line[delay_line_index & 0x3F] = current_sample;

        // Delay line get, and LPF
        sample_mixed = (delay_line[(delay_line_index - (samples_per_bit / 2)) & 0x3F] * current_sample) / 4;
        sample_filtered = prev_mixed + sample_mixed + (prev_filtered / 2);

        delay_line_index++;

        prev_filtered = sample_filtered;
        prev_mixed = sample_mixed;

        // Slice
        sample_bits <<= 1;
        sample_bits |= (sample_filtered < -20) ? 1 : 0;

        // Check for "clean" transition: either 0011 or 1100
        if ((((sample_bits >> 2) ^ sample_bits) & 3) == 3) {
            // Adjust phase
            if (phase < 0x8000)
                phase += 0x800;  // Is this a proper value ?
            else
                phase -= 0x800;
        }

        phase += phase_inc;

        if (phase >= 0x10000) {
            phase &= 0xFFFF;

            if (trigger_word) {
                // Continuous-stream value-triggered mode (AX.25) - UNTESTED
                word_bits <<= 1;
                word_bits |= (sample_bits & 1);

                bit_counter++;

                if (triggered) {
                    if (bit_counter == word_length) {
                        bit_counter = 0;

                        data_message.is_data = true;
                        // data_message.value = word_bits & word_mask;
                        data_message.value = word_bits;
                        shared_memory.application_queue.push(data_message);
                    }
                } else {
                    // if ((word_bits & word_mask) == trigger_value) {
                    if (word_bits == trigger_value) {
                        triggered = !triggered;
                        bit_counter = 0;

                        data_message.is_data = true;
                        data_message.value = trigger_value;
                        shared_memory.application_queue.push(data_message);
                    }
                }

            } else {
                // RS232-like modem mode
                if (state == WAIT_START) {
                    if (!(sample_bits & 1)) {
                        // Got start bit
                        state = RECEIVE;
                        bit_counter = 0;
                    }
                } else if (state == WAIT_STOP) {
                    if (sample_bits & 1) {
                        // Got stop bit
                        state = WAIT_START;
                    }
                } else {
                    word_bits <<= 1;
                    word_bits |= (sample_bits & 1);

                    bit_counter++;
                }

                if (bit_counter == word_length) {
                    bit_counter = 0;
                    state = WAIT_STOP;

                    data_message.is_data = true;
                    data_message.value = word_bits;
                    shared_memory.application_queue.push(data_message);
                }
            }
        }
    }
}

std::vector<int> demodulateFSK(const std::vector<uint8_t>& buffer) {
    std::vector<int> fskTones;
    size_t samplesPerBit = static_cast<size_t>(SAMPLE_RATE / BAUD_RATE);
    
    for (size_t i = 0; i + samplesPerBit <= buffer.size(); i += samplesPerBit) {
        double markPower = 0.0;
        double spacePower = 0.0;
        
        for (size_t j = 0; j < samplesPerBit; ++j) {
            double sample = static_cast<double>(buffer[i + j]);
            markPower += std::cos(2.0 * PI * MARK_FREQ * j / SAMPLE_RATE) * sample;
            spacePower += std::cos(2.0 * PI * SPACE_FREQ * j / SAMPLE_RATE) * sample;
        }
        
        if (markPower > spacePower) {
            fskTones.push_back(1);
        } else {
            fskTones.push_back(0);
        }
    }
    
    return fskTones;
}

void RTTYRxProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::RTTYRxConfigure)
        configure(*reinterpret_cast<const RTTYRxConfigureMessage*>(message));
}

void RTTYRxProcessor::configure(const RTTYRxConfigureMessage& message) {
    decim_0.configure(taps_6k0_decim_0.taps);
    decim_1.configure(taps_6k0_decim_1.taps);
    channel_filter.configure(taps_2k8_lsb_channel.taps, 2);
    
    // audio_output.configure(audio_24k_hpf_300hz_config, audio_24k_deemph_300_6_config, 0);
    audio_output.configure(audio_12k_hpf_300hz_config, audio_12k_deemph_300_6_config, 0);
    samples_per_bit = audio_fs / message.baudrate;

    phase_inc = (0x10000 * message.baudrate) / audio_fs;
    phase = 0;

    trigger_word = message.trigger_word;
    word_length = message.word_length;
    trigger_value = message.trigger_value;
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
