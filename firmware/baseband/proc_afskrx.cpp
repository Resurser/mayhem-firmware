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
#include "proc_afskrx.hpp"
#include "portapack_shared_memory.hpp"
#include "audio_dma.hpp"
#include "event_m4.hpp"

void AFSKRxProcessor::execute(const buffer_c8_t& buffer) {
    // This is called at 3072000 / 2048 = 1500Hz

    if (!configured) return;

    // FM demodulation
    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);              // 2048 / 8 = 256 (512 I/Q samples)
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);         // 256 / 8 = 32 (64 I/Q samples)
    const auto decim_2_out = decim_2.execute(decim_1_out, dst_buffer);         // 256 / 8 = 32 (64 I/Q samples)
    const auto channel_out = channel_filter.execute(decim_2_out, dst_buffer);  // 32 / 2 = 16 (32 I/Q samples)

    feed_channel_stats(channel_out);

    auto audio = demod.execute(channel_out, audio_buffer);

    audio_output.write(audio);

    // Audio signal processing
    const uint32_t half_samples_per_bit = samples_per_bit / 2;
    const int32_t slice_threshold = -20;  // Threshold for slicing
    const uint32_t phase_wrap = 0x10000;

    for (size_t c = 0; c < audio.count; c++) {
        // Scale and saturate the sample
        const int32_t sample_int = audio.p[c] * 32768.0f;
        int32_t current_sample = __SSAT(sample_int, 16) / 128;

        
    }
}

// Helper to compute phase increment from frequency.
// uint16_t AFSKRxProcessor::calculatePhaseIncrement(uint32_t frequency) {
//     return (frequency * PHASE_RESOLUTION) / SAMPLE_RATE;
// }

// void AFSKRxProcessor::initSinTable() {
//     for (uint16_t i = 0; i < SINE_TABLE_SIZE; ++i) {
//         // Compute sine value and scale it to Q15 (i.e. values in [-SCALE, SCALE])
//         sinTable[i] = static_cast<int16_t>(SCALE * sin((2.0 * M_PI * i) / SINE_TABLE_SIZE));
//     }
// }

// void AFSKRxProcessor::audioCallback(const int16_t* audioBuffer, size_t bufferSize, uint32_t markFreq, uint32_t spaceFreq) {
//     uint16_t markPhaseInc = calculatePhaseIncrement(markFreq);
//     uint16_t spacePhaseInc = calculatePhaseIncrement(spaceFreq);

//     for (size_t i = 0; i < bufferSize; ++i) {
//         decodeRTTYBit(audioBuffer[i], markPhaseInc, spacePhaseInc);
//     }
// }

// void AFSKRxProcessor::decodeRTTYBit(int16_t sample, uint16_t markPhaseInc, uint16_t spacePhaseInc) {
//     // Update phases for MARK and SPACE; keep within 0..PHASE_RESOLUTION-1
//     markPhase = (markPhase + markPhaseInc) % PHASE_RESOLUTION;
//     spacePhase = (spacePhase + spacePhaseInc) % PHASE_RESOLUTION;

//     // Instead of indexing the huge table, we reduce the phase by shifting:
//     // Since PHASE_RESOLUTION is 65536 and SINE_TABLE_SIZE is 256, we use the high 8 bits.
//     uint16_t markIndex = markPhase >> 8;   // Equivalent to dividing by 256
//     uint16_t spaceIndex = spacePhase >> 8;

//     // Accumulate the contributions for the current sample.
//     // Multiply the sample (int16) by the sine table value (Q15) then adjust back by dividing by SCALE.
//     accumulatedMark += (sample * sinTable[markIndex]) / SCALE;
//     accumulatedSpace += (sample * sinTable[spaceIndex]) / SCALE;

//     // Once enough samples that constitute one bit are accumulated, decide on the bit.
//     if (++sampleCount >= SAMPLES_PER_BIT) {
//         bool bit = (accumulatedMark > accumulatedSpace);  // If MARK accumulator > SPACE, bit is 1.

//         // First, synchronize using the start bit. The convention here is that the start bit is SPACE (0).
//         if (!isStartBit) {
//             if (!bit) {  // Received start bit (0) as expected.
//                 isStartBit = true;
//                 resetAccumulators();
//             }
//             return;  // Do not process further until we've synchronized.
//         }

//         // Shift the detected bit into the current character buffer.
//         // We build the character from its 5 Baudot data bits.
//         currentChar >>= 1;  // Shift right to make room for the new bit.
//         if (bit) {
//             currentChar |= 0x10;  // Set the top bit (bit 4) if MARK (1) is detected.
//         }

//         // When 5 data bits are accumulated, proceed to process the stop bits.
//         if (++bitCount == 5) {
//             stopBitCount = 0;  // Prepare to validate the stop bits.
//             resetAccumulators();
//             return;
//         }

//         // Now, during stop bit processing, validate that we see a valid stop bit (ideally SPACE or MARK depending on protocol).
//         // For 1.5 stop bits, we require SAMPLES_STOP_BITS samples.
//         if (bitCount == 5 && ++stopBitCount >= SAMPLES_STOP_BITS) {
//             // For simplicity, assume the stop bit is valid if we reach here.
            

//             // Refresh the display with the decoded text.
//             gui_clear();
//             gui_draw_text(10, 10, decodedMessage.c_str(), GUI_COLOR_WHITE, GUI_COLOR_BLACK);

//             // Reset state to decode the next character.
//             currentChar = 0;
//             bitCount = 0;
//             isStartBit = false;  // Await the next start bit.
//         }

//         // Reset accumulators for the next bit period.
//         resetAccumulators();
//     }
// }
// decodeRTTYBit(int16_t sample, uint16_t markPhaseInc, uint16_t spacePhaseInc) {
//     // Update phases for MARK and SPACE; keep within 0..PHASE_RESOLUTION-1
//     markPhase = (markPhase + markPhaseInc) % PHASE_RESOLUTION;
//     spacePhase = (spacePhase + spacePhaseInc) % PHASE_RESOLUTION;

//     // Instead of indexing the huge table, we reduce the phase by shifting:
//     // Since PHASE_RESOLUTION is 65536 and SINE_TABLE_SIZE is 256, we use the high 8 bits.
//     uint16_t markIndex = markPhase >> 8;   // Equivalent to dividing by 256
//     uint16_t spaceIndex = spacePhase >> 8;

//     // Accumulate the contributions for the current sample.
//     // Multiply the sample (int16) by the sine table value (Q15) then adjust back by dividing by SCALE.
//     accumulatedMark += (sample * sinTable[markIndex]) / SCALE;
//     accumulatedSpace += (sample * sinTable[spaceIndex]) / SCALE;

//     // Once enough samples that constitute one bit are accumulated, decide on the bit.
//     if (++sampleCount >= SAMPLES_PER_BIT) {
//         bool bit = (accumulatedMark > accumulatedSpace);  // If MARK accumulator > SPACE, bit is 1.

//         // First, synchronize using the start bit. The convention here is that the start bit is SPACE (0).
//         if (!isStartBit) {
//             if (!bit) {  // Received start bit (0) as expected.
//                 isStartBit = true;
//                 resetAccumulators();
//             }
//             return;  // Do not process further until we've synchronized.
//         }

//         // Shift the detected bit into the current character buffer.
//         // We build the character from its 5 Baudot data bits.
//         currentChar >>= 1;  // Shift right to make room for the new bit.
//         if (bit) {
//             currentChar |= 0x10;  // Set the top bit (bit 4) if MARK (1) is detected.
//         }

//         // When 5 data bits are accumulated, proceed to process the stop bits.
//         if (++bitCount == 5) {
//             stopBitCount = 0;  // Prepare to validate the stop bits.
//             resetAccumulators();
//             return;
//         }

//         // Now, during stop bit processing, validate that we see a valid stop bit (ideally SPACE or MARK depending on protocol).
//         // For 1.5 stop bits, we require SAMPLES_STOP_BITS samples.
//         if (bitCount == 5 && ++stopBitCount >= SAMPLES_STOP_BITS) {
//             // For simplicity, assume the stop bit is valid if we reach here.
//             char decodedChar = decodeBaudot(currentChar);
//             if (decodedChar != '\0') {
//                 decodedMessage += decodedChar;
//             }

//             // Refresh the display with the decoded text.
//             // gui_clear();
//             //gui_draw_text(10, 10, decodedMessage.c_str(), GUI_COLOR_WHITE, GUI_COLOR_BLACK);

//             // Reset state to decode the next character.
//             currentChar = 0;
//             bitCount = 0;
//             isStartBit = false;  // Await the next start bit.
//         }

//         // Reset accumulators for the next bit period.
//         resetAccumulators();
//     }
// }

void AFSKRxProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::AFSKRxConfigure)
        configure(*reinterpret_cast<const AFSKRxConfigureMessage*>(message));
        // initSinTable();
}

void AFSKRxProcessor::configure(const AFSKRxConfigureMessage& message) {
    /*constexpr size_t decim_0_input_fs = baseband_fs;
        constexpr size_t decim_0_output_fs = decim_0_input_fs / decim_0.decimation_factor;

        constexpr size_t decim_1_input_fs = decim_0_output_fs;
        constexpr size_t decim_1_output_fs = decim_1_input_fs / decim_1.decimation_factor;

        constexpr size_t channel_filter_input_fs = decim_1_output_fs;
        const size_t channel_filter_output_fs = channel_filter_input_fs / 2;

        const size_t demod_input_fs = channel_filter_output_fs;*/

    decim_0.configure(taps_6k0_decim_0.taps);
    decim_1.configure(taps_6k0_narrow_decim_1.taps);
    decim_2.configure(taps_6k0_decim_2.taps, 4);
    channel_filter.configure(taps_2k8_lsb_channel.taps, 1);
    audio_output.configure(audio_12k_hpf_300hz_config);
    // decim_0.configure(taps_11k0_decim_0.taps);
    // decim_1.configure(taps_11k0_decim_1.taps);
    // channel_filter.configure(taps_11k0_channel.taps, 2);

    samples_per_bit = audio_fs / message.baudrate;

    phase_inc = (0x10000 * message.baudrate) / audio_fs;
    phase = 0;

    trigger_word = 0;  // message.trigger_word;
    word_length = message.word_length;
    trigger_value = 0;  // message.trigger_value;
    word_mask = (1 << word_length) - 1;

    // Delay line
    delay_line_index = 0;

    triggered = false;
    state = WAIT_START;

    configured = true;
}

// // Reset the accumulators for signal processing per bit
// void AFSKRxProcessor::resetAccumulators() {
//     accumulatedMark = 0;
//     accumulatedSpace = 0;
//     sampleCount = 0;
// }

int main() {
    audio::dma::init_audio_out();

    EventDispatcher event_dispatcher{std::make_unique<AFSKRxProcessor>()};
    event_dispatcher.run();
    return 0;
}
