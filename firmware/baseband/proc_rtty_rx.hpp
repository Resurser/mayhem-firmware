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

#ifndef __PROC_RTTYRX_H__
#define __PROC_RTTYRX_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"

#include "dsp_decimate.hpp"
#include "dsp_demodulate.hpp"

#include "audio_output.hpp"

#include "fifo.hpp"
#include "sine_table_int8.hpp"
#include "message.hpp"


// -------------------- Configuration Constants --------------------
#define SCALE 32768                       // Fixed-point scale factor (Q15)
#define TABLE_SIZE 256                    // Reduced sine table size
#define PHASE_MASK (TABLE_SIZE - 1)       // Mask to wrap phase index
#define SAMPLE_RATE 12000                 // Audio sample rate in Hz
#define DEFAULT_MARK_FREQ 1275
#define DEFAULT_SPACE_FREQ 1725
#define DEFAULT_BAUD_RATE 50                               // RTTY baud rate (bits per second)
#define SAMPLES_PER_BIT (SAMPLE_RATE / DEFAULT_BAUD_RATE)  // Samples per bit duration
#define SAMPLES_STOP_BITS (1.5 * SAMPLES_PER_BIT)          // 1.5 Stop bits duration

class RTTYRxProcessor : public BasebandProcessor {
   public:
    RTTYRxProcessor();

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    static constexpr size_t baseband_fs = 3072000;
    static constexpr size_t audio_fs = baseband_fs / 8 / 8 / 4;


    std::array<complex16_t, 512> dst{};
    const buffer_c16_t dst_buffer{
        dst.data(),
        dst.size()};

    std::array<float, 32> audio{};
    const buffer_f32_t audio_buffer{
        audio.data(),
        audio.size()
    };
    
    dsp::decimate::FIRC8xR16x24FS4Decim4 decim_0 { };
    dsp::decimate::FIRC16xR16x32Decim8 decim_1{};
    dsp::decimate::FIRAndDecimateComplex decim_2{};
    dsp::decimate::FIRAndDecimateComplex channel_filter{};

    dsp::demodulate::SSB demod{};

    AudioOutput audio_output{};

    // -------------------- Динамічні параметри --------------------
    uint16_t baudRate       = DEFAULT_BAUD_RATE;
    uint16_t markPhaseInc   = 0;
    uint16_t spacePhaseInc  = 0;
    uint16_t markFreq       = DEFAULT_MARK_FREQ;
    uint16_t spaceFreq      = DEFAULT_SPACE_FREQ;
    bool reverseBits = false;  // Чи потрібно перевертати біти
    bool reverseFreq = false;  // Чи потрібно міняти місцями маркерну і просторову частоту

    uint32_t markPhase = 0, spacePhase = 0;             // Fixed-point phases for MARK and SPACE tones
    int32_t accumulatedMark = 0, accumulatedSpace = 0;  // Accumulators for signal strength

    uint8_t currentChar = 0;                 // Character under construction (5-bit Baudot + stop bits)

    int bitCount = 0;                        // Bits processed for the current character
    size_t sampleCount = 0;                  // Samples processed for the current bit
    size_t stopBitCount = 0;                 // Counter for stop bit samples
    bool isStartBit = false;                 // Start bit synchronization flag

    bool configured{false};
    bool bit_value{};
    

    RTTYDataMessage data_message{false, 0};
    RSSIThread rssi_thread{};
    uint32_t calculatePhaseIncrement(uint32_t frequency);
    int16_t fastSin(uint32_t phase);
    void resetAccumulators();
    void decodeRTTYBit(int16_t sample);
    
    uint8_t reverseBitsFunction(uint8_t val);
    void configure(const RTTYRxConfigureMessage& message);


    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
};

#endif /*__PROC_RTTYRX_H__*/
