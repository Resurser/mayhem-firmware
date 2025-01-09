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

static char letters [32] = {
	'\0',	'E',	'\n',	'A',	' ',	'S',	'I',	'U',
	'\r',	'D',	'R',	'J',	'N',	'F',	'C',	'K',
	'T',	'Z',	'L',	'W',	'H',	'Y',	'P',	'Q',
	'O',	'B',	'G',	'.',	'M',	'X',	'V',    '.',
};

// ///*
// // * ITA-2 version of the figures case.
// // */
// //static unsigned char figures[32] = {
// //	'\0',	'3',	'\n',	'-',	' ',	'\'',	'8',	'7',
// //	'\r',	'·',	'4',	'\a',	',',	'·',	':',	'(',
// //	'5',	'+',	')',	'2',	'·',	'6',	'0',	'1',
// //	'9',	'?',	'·',	'·',	'.',	'/',	'=',	'·'
// //};
// ///*
// // * U.S. version of the figures case.
// // */
// //static unsigned char figures[32] = {
// //	'\0',	'3',	'\n',	'-',	' ',	'\a',	'8',	'7',
// //	'\r',	'$',	'4',	'\'',	',',	'!',	':',	'(',
// //	'5',	'"',	')',	'2',	'#',	'6',	'0',	'1',
// //	'9',	'?',	'&',	'·',	'.',	'/',	';',	'·'
// //};
// /*
//  * A mix of the two. This is what seems to be what people actually use.
//  */
static unsigned char figures [32] = {
	'\0',	'3',	'\n',	'-',	' ',	'\'',	'8',	'7',
	'\r',	'$',	'4',	'\a',	',',	'!',	':',	'(',
	'5',	'+',	')',	'2',	'H',	'6',	'0',	'1',
	'9',	'?',	'&',	' ',	'.',	'/',	'=',	'.'
};

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
    void on_data(uint32_t value, bool is_data);

    NavigationView& nav_;
    RxRadioState radio_state_{};
    app_settings::SettingsManager settings_{
        "rx_rtty", app_settings::Mode::RX};

    uint8_t console_color{0};
    uint32_t prev_value{0};
    std::string str_log{""};
    bool logging{false};

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

    Checkbox check_log{
        {0 * 8, 1 * 16},
        3,
        LanguageHelper::currentMessages[LANG_LOG],
        false};

    Text text_debug{
        {0 * 8, 12 + 2 * 16, screen_width, 16},
        LanguageHelper::currentMessages[LANG_DEBUG]};

    Button button_modem_setup{
        {screen_width - 12 * 8, 1 * 16, 96, 24},
        LanguageHelper::currentMessages[LANG_MODEM_SETUP]};

    Console console{
        {0, 4 * 16, 240, screen_width}};

    void on_data_rtty(const RTTYDataMessage& message);

    std::unique_ptr<RTTYLogger> logger{};

    MessageHandlerRegistration message_handler_packet{
        Message::ID::RTTYData,
        [this](Message* const p) {
            const auto message = static_cast<const RTTYDataMessage*>(p);
            this->on_data(message->value, message->is_data);
        }};

    MessageHandlerRegistration message_handler_freqchg{
        Message::ID::FreqChangeCommand,
        [this](Message* const p) {
            const auto message = static_cast<const FreqChangeCommandMessage*>(p);
            this->on_freqchg(message->freq);
        }};

    void on_freqchg(int64_t freq);
};

}  // namespace ui::external_app::rtty_rx

#endif /*__UI_RTTY_RX_H__*/
