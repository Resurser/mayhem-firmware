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

// -------------------- Налаштування за замовчуванням --------------------
#define SAMPLE_RATE 12000        // Частота вибірки (Гц)
#define DEFAULT_BAUD_RATE 50  // Бодова швидкість (біти/сек)
#define DEFAULT_MARK_FREQ 1275   // Маркувальна частота (Гц)
#define DEFAULT_SPACE_FREQ 1725  // Просторова частота (Гц)
#define ADAPTIVE_WINDOW_SIZE 5   // Вікно для адаптивного порогу
#define PI 3.14159265358979323846
#define VGA_GAIN 20
#define AFC_STEP 1                      // Крок коригування частоти
#define AFC_BANDWIDTH 50                // Ширина смуги автопідстроювання частоти
#define NOISE_AMPLITUDE_THRESHOLD 5000  // Мінімальна амплітуда для фільтрації шуму

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
    uint16_t userBaudRate = DEFAULT_BAUD_RATE;
    uint16_t userMarkFreq = DEFAULT_MARK_FREQ;
    uint16_t userSpaceFreq = DEFAULT_SPACE_FREQ;
    #define SAMPLES_PER_BIT (SAMPLE_RATE / userBaudRate)
    #define SAMPLES_STOP_BITS (1.5 * SAMPLES_PER_BIT)
    uint32_t word_length{5};
    uint16_t freq_mark{2125};
    uint16_t freq_space{2295};
   
    bool configured{false};
    bool wait_start{};
    bool bit_value{};
    bool trigger_word{};
    bool triggered{};
    // -------------------- Змінні для декодування --------------------
    int32_t lastSample = 0;
    int zeroCrossings = 0;
    bool lastSign = false;

    uint8_t currentChar = 0;
    int bitCount = 0;
    size_t sampleCount = 0;
    size_t stopBitCount = 0;
    bool isStartBit = false;
    int zeroCrossHistory[ADAPTIVE_WINDOW_SIZE] = {0};
    int adaptiveThreshold = (DEFAULT_MARK_FREQ + DEFAULT_SPACE_FREQ) / 2;
    int32_t signalAmplitude = 0;

    RTTYDataMessage data_message{false, 0};
    RSSIThread rssi_thread{};
    void processRTTYBit(int16_t sample);
    void updateAdaptiveThreshold();
    void measureSignalAmplitude(int16_t sample);
    void adaptiveFrequencyCorrection();
    void configure(const RTTYRxConfigureMessage& message);

    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
};

#endif /*__PROC_RTTYRX_H__*/
