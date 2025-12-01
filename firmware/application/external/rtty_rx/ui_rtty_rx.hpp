/*
 * Copyright (C) 2014 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2017 Furrtek
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

#ifndef __UI_RTTY_RX_H__
#define __UI_RTTY_RX_H__

#include "ui.hpp"
#include "ui_language.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "ui_record_view.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"
#include "log_file.hpp"
#include "utility.hpp"

// Константи
#define RTTY_DEF_MARK_FREQ 2125
#define RTTY_DEF_SPACE_FREQ 2295
#define RTTY_DEF_BAUD 50

using namespace ui;

namespace ui::external_app::rtty_rx {

class RTTYRxView : public View {
   public:
    RTTYRxView(NavigationView& nav);
    ~RTTYRxView();

    void focus() override;

    std::string title() const override { return "RTTY RX"; };

   private:
    NavigationView& nav_;
    RxRadioState radio_state_{};

    app_settings::SettingsManager settings_{
        "rtty_rx",
        app_settings::Mode::RX,
        {
            {"mark"sv, &mark_index},
            {"space"sv, &shift_index},
            {"baud"sv, &reverse_bits},
        }};

    RFAmpField field_rf_amp{
        {13 * 8, 0 * 16}};
    LNAGainField field_lna{
        {15 * 8, 0 * 16}};
    VGAGainField field_vga{
        {18 * 8, 0 * 16}};
    RSSI rssi{
        {21 * 8, 0, 6 * 8, 4}};
    Channel channel{
        {21 * 8, 5, 6 * 8, 4}};


    AudioVolumeField field_volume{
        {28 * 8, 0 * 16}};

    RxFrequencyField field_frequency{
        {0 * 8, 0 * 16},
        nav_};

    Labels labels {
        { { 1 * 8, 2 * 16 }, "Mark :", Color::light_grey() },
        { { 1 * 8, 3 * 16 }, "Space:", Color::light_grey() },
        { { 1 * 8, 4 * 16 }, "Baud :", Color::light_grey() },
        { { 16 * 8, 2 * 16 }, "Signal:", Color::light_grey() } // Підпис для бару
    };
	NumberField field_mark { { 7 * 8, 2 * 16 }, 4, { 200, 3000 }, 1, ' ' };
    NumberField field_space { { 7 * 8, 3 * 16 }, 4, { 200, 3000 }, 1, ' ' };
    NumberField field_baud { { 7 * 8, 4 * 16 }, 3, { 45, 100 }, 1, ' ' };

    ProgressBar tuning_bar {
        { 16 * 8, 3 * 16, 11 * 8, 12 } 
    };

    Console console { { 0, 6 * 16, 240, 160 } };
  
    void on_freqchg(int64_t freq);
	
	void update_config();
    void on_tuning_frequency_changed(rf::Frequency f);
    
    // Обробники повідомлень від Baseband
    void on_char(const RTTYCharMessage& message);
    void on_stats(const RTTYStatsMessage& message); // [НОВЕ] Метод обробки статистики

    // Реєстрація обробника символів
    MessageHandlerRegistration message_handler_char {
        Message::ID::RTTYChar,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const RTTYCharMessage*>(p);
            this->on_char(message);
        }
    };

    // [НОВЕ] Реєстрація обробника статистики (енергії)
    MessageHandlerRegistration message_handler_stats {
        Message::ID::RTTYStats,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const RTTYStatsMessage*>(p);
            this->on_stats(message);
        }
    };

    MessageHandlerRegistration message_handler_freqchg{
        Message::ID::FreqChangeCommand,
        [this](Message* const p) {
            const auto message = static_cast<const FreqChangeCommandMessage*>(p);
            this->on_freqchg(message->freq);
        }
    };
};

}  // namespace ui::external_app::rtty_rx

#endif /*__UI_RTTY_RX_H__*/
