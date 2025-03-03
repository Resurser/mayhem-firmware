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
#include "message.hpp"

#define SAMPLE_RATE 3072000 // 3.072 МГц
#define DECIMATION_FACTOR 128
#define AUDIO_SAMPLE_RATE (SAMPLE_RATE / DECIMATION_FACTOR)
#define BAUD_RATE 50
#define MARK_FREQ 2125
#define SPACE_FREQ 2295
#define SAMPLES_PER_BIT (AUDIO_SAMPLE_RATE / BAUD_RATE)
#define STOP_BITS 1.5
#define BUFFER_SIZE 8192
#define PI 3.14159265358979323846
#define VGA_GAIN 20
#define AFC_BANDWIDTH 50 // Ширина смуги автопідстроювання частоти

class RTTYRxProcessor : public BasebandProcessor {
   public:
    RTTYRxProcessor ();

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;
   private:
   
    static constexpr size_t baseband_fs = 3072000;
    static constexpr size_t audio_fs = baseband_fs / 8 / 8 / 2;

    size_t samples_per_bit{};

    enum State {
        WAIT_START = 0,
        WAIT_STOP,
        RECEIVE
    };

    std::array<complex16_t, 512> dst{};
    const buffer_c16_t dst_buffer{
        dst.data(),
        dst.size()};

    std::array<float, 32> audio{};
    const buffer_f32_t audio_buffer{
        audio.data(),
        audio.size()
    };
    
    dsp::decimate::FIRC8xR16x24FS4Decim8 decim_0{};
    dsp::decimate::FIRC16xR16x32Decim8 decim_1{};
    dsp::decimate::FIRAndDecimateComplex channel_filter{};

    dsp::demodulate::SSB demod{};

    AudioOutput audio_output{};

    std::array<int32_t, 64> delay_line{0};
    uint32_t word_length{5};
    uint16_t freq_mark{2125};
    uint16_t freq_space{2295};

    State state{};
    
    size_t delay_line_index{};
    uint32_t bit_counter{0};
    uint32_t word_bits{0};
    uint32_t sample_bits{0};
    uint32_t phase{}, phase_inc{};
    int32_t sample_mixed{}, prev_mixed{}, sample_filtered{}, prev_filtered{};
    uint32_t word_mask{};
    uint32_t trigger_value{};

    bool configured{false};
    bool wait_start{};
    bool bit_value{};
    bool trigger_word{};
    bool triggered{};


    RTTYDataMessage data_message{false, 0};
    RSSIThread rssi_thread{};

    void configure(const RTTYRxConfigureMessage& message);
    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
};

#endif /*__PROC_RTTYRX_H__*/
