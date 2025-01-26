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
#include <vector>
#include <cmath>
#include "utility.hpp"
const double MARK_FREQ = 2125.0; 
const double SPACE_FREQ = 2295.0; 
const double SAMPLE_RATE = 48000.0; 
const double BAUD_RATE = 45.45; 
const double PI = 3.14159265358979323846;

std::vector<int> convertBufferToVector(const buffer_c8_t* buffer, size_t size) {
    std::vector<int> intVector(size * 2); // Each complex 8-bit value becomes two 16-bit integers
    for (size_t i = 0; i < size; ++i) {
        intVector[i * 2] = (int)buffer[i].p->real();
        intVector[i * 2 + 1] = (int)buffer[i].p->imag();
    }
    return intVector;
}

std::string decodeRTTY(const std::vector<int>& fskTones) { 
    std::string decodedText; 
    size_t bitsPerChar = 5; 
    size_t toneIndex = 0; 
    while (toneIndex + bitsPerChar <= fskTones.size()) { 
        int character = 0; 
        for (size_t bit = 0; bit < bitsPerChar; ++bit) { 
            character |= (fskTones[toneIndex + bit] << (bitsPerChar - 1 - bit)); 
        } 
        // if (baudotToAsciiMap.find(character) != baudotToAsciiMap.end()) { 
        //     decodedText += baudotToAsciiMap[character]; 
        // } else { 
        //     decodedText += '?'; // Unknown character 
        // } 
        // toneIndex += bitsPerChar; 
    } 
    return decodedText; 
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
                        data_message.value = word_bits & word_mask;
                        // data_message.value = word_bits;
                        shared_memory.application_queue.push(data_message);
                    }
                } else {
                    if ((word_bits & word_mask) == trigger_value) {
                    // if (word_bits == trigger_value) {
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

// 


// Function to decode RTTY signal
//int decode_rtty(int8_t* buffer, int length) {
     //Implement RTTY decoding logic here
     //This function should demodulate the signal, filter, and extract the bitstream

     //Initialize variables for RTTY decoding
//    int bitstream = 0;
//    int bits = 0;
//    int bit_count = 0;

     //Loop through the samples and decode the bits
//    for (int i = 0; i < length; i += samples_per_bit) {
         //Apply Goertzel algorithm to detect mark and space frequencies
//        float mark_power = goertzel(buffer + i, samples_per_bit, mark_frequency, SAMPLE_RATE);
//        float space_power = goertzel(buffer + i, samples_per_bit, space_frequency, SAMPLE_RATE);

         //Determine the current bit based on detected power
//        int bit = (mark_power > space_power) ? 1 : 0;

         //Accumulate the bits into a bitstream
//        bitstream = (bitstream >> 1) | (bit << 7);
//        bits++;
//        bit_count++;

         //If 8 bits are collected, decode the character
//        if (bits == 8) {
             //For now, just return the bitstream as the decoded data
//            int decoded_char = bitstream;

             //Reset the bitstream and bit count
//            bitstream = 0;
//            bits = 0;

             //Return the decoded character
//            return decoded_char;
//        }
//    }

     //Return dummy data if no valid character is decoded
//    return 0x00;
//}
//float goertzel(const int8_t* samples, int sample_count, int target_frequency, int sample_rate) {
//    float s_prev = 0.0;
//    float s_prev2 = 0.0;
//    float coeff = 2.0 * cosf(2.0 * M_PI * target_frequency / sample_rate);

//    for (int i = 0; i < sample_count; ++i) {
//        float s = samples[i] + coeff * s_prev - s_prev2;
//        s_prev2 = s_prev;
//        s_prev = s;
//    }

//    return s_prev2 * s_prev2 + s_prev * s_prev - coeff * s_prev * s_prev2;
//}