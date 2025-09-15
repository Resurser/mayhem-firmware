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

using namespace ui;

namespace ui::external_app::rtty_rx {

class RTTYLogger {
   public:
    Optional<File::Error> append(const std::filesystem::path& filename) {
        return log_file.append(filename);
    }

    void log_raw_data(const std::string& data);

   private:
    LogFile log_file{};
};

class RTTYRxView : public View {
   public:
    RTTYRxView(NavigationView& nav);
    ~RTTYRxView();

    void focus() override;

    std::string title() const override { return "RTTY RX"; };

   private:
    void on_data(uint8_t value, bool is_data);
    void on_log(RTTYRxLogMessage msg);

    NavigationView& nav_;
    RxRadioState radio_state_{};

    uint8_t mark_index{0};
    uint8_t shift_index{0};
    bool reverse_bits{false};

    app_settings::SettingsManager settings_{
        "rx_rtty",
        app_settings::Mode::RX,
        {
            {"mark_index"sv, &mark_index},
            {"shift_index"sv, &shift_index},
            {"reverse_bits"sv, &reverse_bits},
        }};
    uint8_t console_color{0};
    uint32_t prev_value{0};

    std::string str_log{""};
    uint16_t rxmode{1};  // LETTERS
    bool is_in_figures_mode = false;
    bool logging{true};
    std::unique_ptr<RTTYLogger> logger{};

    RFAmpField field_rf_amp{
        {13 * 8, 0 * 16}};
    LNAGainField field_lna{
        {15 * 8, 0 * 16}};
    VGAGainField field_vga{
        {18 * 8, 0 * 16}};
    Channel channel{
        {21 * 8, 5, 6 * 8, 4}};

    AudioVolumeField field_volume{
        {28 * 8, 0 * 16}};

    RxFrequencyField field_frequency{
        {0 * 8, 0 * 16},
        nav_};

    Labels labels{
        {{0 * 8, 1 * 16}, "S: ", Theme::getInstance()->fg_light->foreground},
        {{8 * 8, 1 * 16}, "M: ", Theme::getInstance()->fg_light->foreground},
    };

    OptionsField options_shift{
        {4 * 8, 1 * 16},
        4,
        {
            {" 85", 85},
            {" 170", 170},
            {" 450", 450},
            {" 850", 850},
            {" 225", 225},
            {" 425", 425},

            {"-85", -85},
            {"-170", -170},
            {"-450", -450},
            {"-850", -850},
            {"-225", -225},
            {"-425", -425},

        }};

    OptionsField options_mark{
        {12 * 8, 1 * 16},
        4,
        {
            {"1275", 1275},
            {"1445", 1445},
            {"2125", 2125},
            {"2225", 2225},
            {"2295", 2295},
            {"1700", 1700},
            {"800", 800},
        }};
    
    Checkbox checkbox_revert_bits{
        {20 * 8, 1 * 16},
        10,
        "Rev. bits"
    };
    

    Text text_debug{
        {0 * 8, 12 + 2 * 16, screen_width, 16},
        LanguageHelper::currentMessages[LANG_DEBUG]};
    Console console{
        {0, 4 * 16, screen_width, screen_width}};
    char BaudottoChar(const uint8_t data);
    void on_freqchg(int64_t freq);
    void apply_config();

    MessageHandlerRegistration message_handler_data{
        Message::ID::RTTYRxData,
        [this](Message* const p) {
            const auto message = static_cast<const RTTYRxDataMessage*>(p);
            this->on_data(message->value, message->is_data);
        }};

    MessageHandlerRegistration message_handler_freqchg{
        Message::ID::FreqChangeCommand,
        [this](Message* const p) {
            const auto message = static_cast<const FreqChangeCommandMessage*>(p);
            this->on_freqchg(message->freq);
        }};

    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            // this->on_timer();
        }};

    MessageHandlerRegistration message_handler_log_{
        Message::ID::RTTYRxLogData,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const RTTYRxLogMessage*>(p);
            this->on_log(message);
        }};
};

}  // namespace ui::external_app::rtty_rx

#endif /*__UI_RTTY_RX_H__*/
