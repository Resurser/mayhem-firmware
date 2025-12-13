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

#include "message.hpp"

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
        audio.size()};

    dsp::decimate::FIRC8xR16x24FS4Decim4 decim_0{};
    dsp::decimate::FIRC16xR16x32Decim8 decim_1{};
    dsp::decimate::FIRAndDecimateComplex decim_2{};
    dsp::decimate::FIRAndDecimateComplex channel_filter{};

    dsp::demodulate::SSB demod{};

    AudioOutput audio_output{};

   // Нова частота дискретизації для розрахунків
    uint32_t sample_rate = 12000; 
    
    uint32_t mark_freq = 2125;      // Частота логічної одиниці (Mark)
    uint32_t space_freq = 2295;     // Частота логічного нуля (Space)
    float baud_rate = 50.0f;       // Швидкість передачі (Бод)
    
    // Кількість семплів на один біт при 12кГц (12000 / 45.45 ≈ 264)
    uint32_t samples_per_bit = 240; 

    // --- Змінні алгоритму Герцеля (Goertzel) ---
    float coeff_mark = 0.0f;    // Розрахований коефіцієнт для Mark
    float coeff_space = 0.0f;   // Розрахований коефіцієнт для Space
    float s1_mark = 0, s2_mark = 0; // Стан фільтру Mark
    float s1_space = 0, s2_space = 0; // Стан фільтру Space
    
    // --- Стан машини декодера (State Machine) ---
    enum State { IDLE, DATA, STOP }; // Можливі стани: Очікування, Дані, Стоп-біт
    State state = IDLE;              // Початковий стан
    uint32_t sample_count = 0;       // Лічильник семплів всередині поточного біта
    uint8_t bit_buffer = 0;          // Буфер для накопичення бітів символу
    uint8_t bits_received = 0;       // Кількість прийнятих бітів
    bool shift_figs = false;         // Прапор перемикання регістру (Цифри/Букви)
    
    float mark_energy_accumulator = 0.0f; 
    float space_energy_accumulator = 0.0f;
    uint32_t stats_send_counter = 0;
    static constexpr uint32_t STATS_SEND_PERIOD = 800; 

    // --- AFC (Автопідлаштування частоти) ---
    bool afc_enabled = true;           // Чи увімкнено AFC
    complex16_t last_iq_sample {0, 0}; // Попередній семпл для розрахунку дельти фази
    
    // Накопичувачі для виміряних частот
    float measured_freq_accumulator = 0.0f; 
    uint32_t measured_samples_count = 0;

    // Середні значення частот (Mark/Space), виміряні з ефіру
    float afc_mark_sum = 0.0f;
    uint32_t afc_mark_count = 0;
    float afc_space_sum = 0.0f;
    uint32_t afc_space_count = 0;

    // Лічильник для періодичного оновлення коефіцієнтів
    uint32_t afc_update_counter = 0;
    static constexpr uint32_t AFC_UPDATE_PERIOD = 12000; // Раз на секунду (12к семплів)

    // --- Внутрішні функції ---
    void configure(const RTTYConfigMessage& message); // Застосування налаштувань
    void process_sample_pair(int16_t audio_sample, complex16_t iq_sample); 
    void handle_bit(bool bit);                        // Логіка обробки біта (0 або 1)
    void decode_baudot(uint8_t bits);                 // Конвертація 5 біт в символ
    void update_coeffs();                             // Перерахунок коефіцієнтів Герцеля
    void send_stats();                             // Відправка статистики
    void apply_afc();                              // Застосування AFC
    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
    RSSIThread rssi_thread{};
};

#endif /*__PROC_RTTYRX_H__*/
