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
#include <vector>

#include "utility.hpp"

// Інверсія порядку бітів у 5-бітному символі Бодо
uint8_t RTTYRxProcessor::reverseBitsFunction(uint8_t val) {
    uint8_t result = 0;
    for (int i = 0; i < 5; ++i) {
        result <<= 1;
        result |= (val & 1);
        val >>= 1;
    }
    return result;
}

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

    // configured = false;
}

// Process a single audio sample and decode it into an RTTY bit
void RTTYRxProcessor::decodeRTTYBit(int32_t sample) {
    // Increment the phases for MARK and SPACE tones
    // markPhase   = (markPhase + markPhaseInc) & 0xFFFFFFFF;  // Wrap around 32 bits
    // spacePhase  = (spacePhase + spacePhaseInc) & 0xFFFFFFFF;  // Wrap around 32 bits
    markPhase = (markPhase + markPhaseInc) % 65536;
    spacePhase = (spacePhase + spacePhaseInc) % 65536;

    // Instead of indexing the huge table, we reduce the phase by shifting:
    // Since PHASE_RESOLUTION is 65536 and SINE_TABLE_SIZE is 256, we use the high 8 bits.
    uint16_t markIndex = markPhase >> 8;  // Equivalent to dividing by 256
    uint16_t spaceIndex = spacePhase >> 8;

    // Accumulate the contributions for the current sample.
    // Multiply the sample (int16) by the sine table value (Q15) then adjust back by dividing by SCALE.
    accumulatedMark += (sample * sine_table_q15[markIndex]) / SCALE;
    accumulatedSpace += (sample * sine_table_q15[spaceIndex]) / SCALE;
    // // Accumulate the contributions of the current sample
    // accumulatedMark += (sample * fastSin(markPhase)) / SCALE;
    // accumulatedSpace += (sample * fastSin(spacePhase)) / SCALE;

    // Process a bit after accumulating enough samples
    if (++sampleCount >= SAMPLES_PER_BIT) {
        bool bit = (accumulatedMark > accumulatedSpace);  // Determine MARK or SPACE
        if (reverseFreq) {
            bit = !bit;
        }
        // Handle start bit synchronization
        if (!isStartBit) {
            if (!bit) {  // Start bit must be SPACE (0)
                isStartBit = true;
                resetAccumulators();
            }
            return;
        }

        // Shift the detected bit into the current character
        currentChar >>= 1;
        if (bit) currentChar |= 0x10;  // Set the MSB if MARK (1)

        // If 5 data bits are complete, process stop bits
        if (++bitCount == 5) {
            stopBitCount = 0;  // Reset stop bit counter
            resetAccumulators();
            return;
        }

        log_message.cnt++;
        log_message.samples[log_message.cnt - 1] = currentChar;
        
        // Validate 1.5 stop bits
        if (bitCount == 5 && ++stopBitCount >= SAMPLES_STOP_BITS) {
            // char decodedChar = decodeBaudot(currentChar);
            // if (decodedChar != '\0') decodedMessage += decodedChar;
            if (reverseBits) {
                currentChar = reverseBitsFunction(currentChar);
            }
            if (log_message.cnt == 8) {
                shared_memory.application_queue.push(log_message);
                log_message.cnt = 0;
            }

            data_message.is_data = true;
            data_message.value = currentChar;
            shared_memory.application_queue.push(data_message);

            // Reset for the next character
            currentChar = 0;
            bitCount = 0;
            isStartBit = false;  // Wait for the next start bit
        }

        // Reset accumulators for the next bit
        resetAccumulators();
    }
}

int32_t RTTYRxProcessor::fastSin(uint32_t phase) {
    uint16_t index = (phase >> 16) & PHASE_MASK;    // Extract table index
    uint16_t nextIndex = (index + 1) & PHASE_MASK;  // Next index (wrap around)
    uint16_t fractional = (phase & 0xFFFF) >> 8;    // Fractional part (8-bit resolution)

    // Perform linear interpolation
    int16_t value1 = sine_table_q15[index];
    int16_t value2 = sine_table_q15[nextIndex];
    return value1 + ((value2 - value1) * fractional / 256);
}

// Calculate phase increment dynamically based on frequency
uint32_t RTTYRxProcessor::calculatePhaseIncrement(uint32_t frequency) {
    return (frequency * TABLE_SIZE) / SAMPLE_RATE;
}

// Reset the accumulators after processing each bit or character
void RTTYRxProcessor::resetAccumulators() {
    accumulatedMark = 0;
    accumulatedSpace = 0;
    sampleCount = 0;
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

    for (size_t c = 0; c < audio.count; c++) {
        // Scale and saturate the sample
        const int32_t sample_int = audio.p[c] * SCALE;

        int32_t current_sample = __SSAT(sample_int, 16);  // Scale to Q15 format
        decodeRTTYBit(sample_int);
    }
}

void RTTYRxProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::RTTYRxConfigure)
        configure(*reinterpret_cast<const RTTYRxConfigureMessage*>(message));
    if (message->id == Message::ID::CaptureConfig)
        capture_config(*reinterpret_cast<const CaptureConfigMessage*>(message));
}

void RTTYRxProcessor::configure(const RTTYRxConfigureMessage& message) {
    configured = false;
    markFreq = message.freq_mark;
    spaceFreq = message.freq_space;
    baudRate = message.baudrate;
    reverseBits = message.reverse_bits;
    reverseFreq = message.reverse_freq;

    decim_0.configure(taps_6k0_decim_0.taps);
    decim_1.configure(taps_6k0_decim_1.taps);
    decim_2.configure(taps_6k0_decim_2.taps, 4);
    channel_filter.configure(taps_2k8_lsb_channel.taps, 1);
    audio_output.configure(audio_12k_hpf_300hz_config);  //, audio_12k_deemph_300_6_config);

    markPhaseInc = calculatePhaseIncrement(markFreq);    // Calculate MARK phase increment dynamically
    spacePhaseInc = calculatePhaseIncrement(spaceFreq);  // Calculate SPACE phase increment dynamically

    configured = true;
}

void RTTYRxProcessor::capture_config(const CaptureConfigMessage& message) {
    if (message.config) {
        audio_output.set_stream(std::make_unique<StreamInput>(message.config));
    } else {
        audio_output.set_stream(nullptr);
    }
}

int main() {
    audio::dma::init_audio_out();

    EventDispatcher event_dispatcher{std::make_unique<RTTYRxProcessor>()};
    event_dispatcher.run();
    return 0;
}