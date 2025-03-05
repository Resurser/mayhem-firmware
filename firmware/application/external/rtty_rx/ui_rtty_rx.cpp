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

#include "ui_rtty_rx.hpp"
#include "ui_modemsetup.hpp"
#include <unordered_map>

#include "modems.hpp"
#include "audio.hpp"
#include "rtc_time.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "portapack_persistent_memory.hpp"
#include "file_path.hpp"

using namespace portapack;
using namespace modems;
using namespace ui;

namespace ui::external_app::rtty_rx {



void RTTYLogger::log_raw_data(const std::string& data) {
    log_file.write_entry(data);
}

void RTTYRxView::focus() {
    field_frequency.focus();
}

RTTYRxView::RTTYRxView(NavigationView& nav)
    : nav_{nav} {
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({&rssi,
                  &channel,
                  &field_rf_amp,
                  &field_lna,
                  &field_vga,
                  &field_volume,
                  &field_frequency,
                  &check_log,
                  &text_debug,
                  &button_modem_setup,
                  &console});

    // Auto-configure modem for LCR RX (TODO remove)
    field_frequency.set_value(settings_.raw().rx_frequency);
   
    auto receiver_modem = &modem_defs[7];
    persistent_memory::set_modem_baudrate(receiver_modem->baudrate);
    serial_format_t serial_format;

    // serial_format.data_bits = 7;
    // serial_format.parity = EVEN;
    // serial_format.stop_bits = 2;
    // serial_format.bit_order = LSB_FIRST;
    serial_format.data_bits = 5;
    serial_format.parity = NONE;
    serial_format.stop_bits = 1;
    serial_format.bit_order = MSB_FIRST;
    
    persistent_memory::set_serial_format(serial_format);

    field_frequency.set_step(1000);

    check_log.set_value(logging);
    check_log.on_select = [this](Checkbox&, bool v) {
        logging = v;
    };

    button_modem_setup.on_select = [&nav](Button&) {
        nav.push<ModemSetupView>();
    };

    logger = std::make_unique<RTTYLogger>();
    if (logger)
        logger->append(logs_dir / u"RTTY.TXT");

    // Auto-configure modem for LCR RX (will be removed later)
    baseband::set_rtty(50, 5);
    audio::set_rate(audio::Rate::Hz_24000);
    audio::output::start();
    receiver_model.enable();
}

char RTTYRxView::BaudottoChar(const uint32_t data) {
    int out = 0;
    const char letters[32] = {
        '\0',	'E',	'\n',	'A',	' ',	'S',	'I',	'U',
        '\r',	'D',	'R',	'J',	'N',	'F',	'C',	'K',
        'T',	'Z',	'L',	'W',	'H',	'Y',	'P',	'Q',
        'O',	'B',	'G',	' ',	'M',	'X',	'V',	' '
    };
    const char figures[32] = {
        '\0',	'3',	'\n',	'-',	' ',	'\a',	'8',	'7',
        '\r',	'$',	'4',	'\'',	',',	'!',	':',	'(',
        '5',	'"',	')',	'2',	'#',	'6',	'0',	'1',
        '9',	'?',	'&',	' ',	'.',	'/',	';',	' '
    };

    switch (data) {
    case 0x1F:		/* letters */
        rxmode = 1;
        break;
    case 0x1B:		/* figures */
        rxmode = 2;
        break;
    case 0x04:		/* unshift-on-space */
//        if (progdefaults.UOSrx)
//            rxmode = LETTERS;
        return ' ';
        break;
    default:
        if (rxmode == 2)
            out = figures[data];
        else
            out = letters[data];
        break;
    }

    return out;
}

void RTTYRxView::on_data(uint32_t value, bool is_data) {
    std::string str_console = "\x1B";
    std::string str_byte = "";

    if (is_data) {
        // Colorize differently after message splits
        str_console += (char)((console_color & 3) + 9);

        // value = deframe_word(value);
        uint32_t alt_val2 = value;
        str_console += (char)BaudottoChar(value);
        
        uint32_t alt_val = value & 0x1F;   
        value &= 0xFF;                                          // ABCDEFGH
        // text_debug.set("<<" + to_string_dec_uint(value));

        value = ((value & 0xF0) >> 4) | ((value & 0x0F) << 4);  // EFGHABCD
        uint32_t alt_val3 = value;
        value = ((value & 0xCC) >> 2) | ((value & 0x33) << 2);  // GHEFCDAB
        uint32_t alt_val4 = value;
        value = ((value & 0xAA) >> 1) | ((value & 0x55) << 1);  // HGFEDCBA
        uint32_t alt_val5 = value;
        value &= 0x7F;                                          // Ignore parity, which is the MSB now
        
        text_debug.set("Origin: " + to_string_dec_uint(value, 2)+
            " :: "+to_string_dec_uint(alt_val, 2)+
            " > "+to_string_hex(alt_val2, 2)+
            " >> "+to_string_hex(alt_val3, 2)+
            " >>> "+to_string_hex(alt_val4, 2)+
            " >>>> "+to_string_hex(alt_val5, 2));
        

        if (logging){
            value = alt_val;
        } else { 

        }
        
        if ((value >= 32) && (value < 127)) {
            str_console += (char)value;  // Printable
            str_byte    += (char)value;
        } else if (value < 32) {
            str_console += (char)BaudottoChar(value);  // Printable
            str_byte    += (char)BaudottoChar(value);
        } else {
            str_console += "[" + to_string_hex(value, 2) + "]";  // Not printable
            str_byte    += "[" + to_string_hex(value, 2) + "]";
        }
    
        
        // str_byte = to_string_bin(value & 0xFF, 8) + "  ";

        console.write(str_console);
        if (logger && logging) str_log += str_byte;

        if ((value != 0x7F) && (prev_value == 0x7F)) {
            // Message split
            console.writeln("");
            console_color++;

            if (logger && logging) {
                logger->log_raw_data(str_log);
                str_log = "";
            }
        }
        prev_value = value;
    } else {
        // Baudrate estimation
        text_debug.set("Baudrate estimation: ~" + to_string_dec_uint(value));
    }
}

void RTTYRxView::on_freqchg(int64_t freq) {
    field_frequency.set_value(freq);
}

RTTYRxView::~RTTYRxView() {
    audio::output::stop();
    receiver_model.disable();
    baseband::shutdown();
}

}  // namespace ui::external_app::afsk_rx
