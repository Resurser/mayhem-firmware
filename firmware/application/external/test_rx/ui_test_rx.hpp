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

#ifndef __UI_TEST_RX_H__
#define __UI_TEST_RX_H__

#include "ui.hpp"
#include "ui_language.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "ui_record_view.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"

#include "test_packet.hpp"
#include "log_file.hpp"
#include "utility.hpp"

using namespace ui;

namespace ui::external_app::test_rx {
class TestRxLogger {
   public:
    Optional<File::Error> append(const std::filesystem::path& filename) {
        return log_file.append(filename);
    }

    void log_raw_data(const testapp::Packet& packet, const int32_t alt);

   private:
    LogFile log_file{};
};

class TestRxView : public View {
   public:
    TestRxView(NavigationView& nav);
    ~TestRxView();

    void focus() override;

    std::string title() const override { return "TEST RX"; };

   private:
    NavigationView& nav_;
    RxRadioState radio_state_{
        10100000 /* frequency */,
        1750000 /* bandwidth */,
        3072000 /* sampling rate */
    };

    Coord cur_x{0};
    uint32_t packet_count{0};
    uint32_t packets_lost{0};
    uint32_t prev_v{0};
    uint32_t raw_alt{0};
    uint32_t cal_value{0};
    bool logging{false};

    Labels labels{
        {{0 * 8, 1 * 16}, "Data:", Theme::getInstance()->fg_light->foreground}};

    RxFrequencyField field_frequency{
        {0 * 8, 0 * 8},
        nav_};
    RFAmpField field_rf_amp{
        {13 * 8, 0 * 16}};

    LNAGainField field_lna{
        {15 * 8, 0 * 16}};

    VGAGainField field_vga{
        {18 * 8, 0 * 16}};

    RSSI rssi{
        {21 * 8, 0, 6 * 8, 4},
    };

    Text text_debug_a{
        {0 * 8, 4 * 16, 30 * 8, 16},
        "..."};
    Text text_debug_b{
        {0 * 8, 5 * 16, 30 * 8, 16},
        "..."};

    Button button_cal{
        {17 * 8, 2 * 16, 5 * 8, 2 * 16},
        "CAL"};
    Checkbox check_log{
        {23 * 8, 2 * 16},
        3,
        "LOG"};

    std::unique_ptr<TestRxLogger> logger{};

    MessageHandlerRegistration message_handler_packet{
        Message::ID::TestRxPacket,
        [this](Message* const p) {
            const auto message = static_cast<const TestRxPacketMessage*>(p);
            const testapp::Packet packet{message->packet};
            on_packet(packet);
        }};

    void on_packet(const testapp::Packet& packet);
};


}  // namespace ui::external_app::test_rx

#endif /*__UI_TEST_RX_H__*/
