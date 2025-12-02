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
#include "modems.hpp"
#include "audio.hpp"
#include "baseband_api.hpp"
#include "portapack_persistent_memory.hpp"

using namespace portapack;
using namespace modems;
using namespace ui;

namespace ui::external_app::rtty_rx {

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
        &labels,
        &field_mark,
        &field_space,
        &field_baud,
        &tuning_bar, // [НОВЕ] Додаємо бар до списку дочірніх елементів
        &console
    });

    // Налаштування меж для бару (0 .. 65535, як приходить з Baseband)
    tuning_bar.set_max(65535); 
    tuning_bar.set_value(0);
    
    field_frequency.set_step(1000);
	field_frequency.set_value(settings_.raw().rx_frequency);

    // --- Ініціалізація значень ---
    //field_frequency.set_value(receiver_model.tuning_frequency());
    //field_lna.set_value(receiver_model.lna());
    //field_vga.set_value(receiver_model.vga());
    
    field_mark.set_value(RTTY_DEF_MARK_FREQ);
    field_space.set_value(RTTY_DEF_SPACE_FREQ);
    field_baud.set_value(RTTY_DEF_BAUD);

    //field_lna.on_change = [this](int32_t v) { receiver_model.set_lna(v); };
    //field_vga.on_change = [this](int32_t v) { receiver_model.set_vga(v); };
    //field_volume.on_change = [this](int32_t v) { audio::output::set_stream_volume(v); };
    field_mark.on_change = [this](int32_t) { update_config(); };
    field_space.on_change = [this](int32_t) { update_config(); };
    field_baud.on_change = [this](int32_t) { update_config(); };

    // --- Старт ---
    update_config();
    
    audio::set_rate(audio::Rate::Hz_12000);
    audio::output::start();
    
    //receiver_model.set_modulation(ReceiverModel::Mode::AMAudio);
    receiver_model.set_sampling_rate(3072000); 
    receiver_model.set_baseband_bandwidth(1750000);
    receiver_model.enable();
}


RTTYRxView::~RTTYRxView() {
    audio::output::stop();
    receiver_model.disable();
    baseband::shutdown();
}

void RTTYRxView::focus() {
    field_frequency.focus();
}

void RTTYRxView::on_tuning_frequency_changed(rf::Frequency f) {
    receiver_model.set_target_frequency(f);
}

void RTTYRxView::update_config() {
    baseband::set_rtty(field_mark.value(),
                       field_space.value(),
                       field_baud.value());
}

void RTTYRxView::on_char(const RTTYCharMessage& message) {
    std::string text(1, message.character);
    console.write(text);
}

// [НОВЕ] Обробка статистики для візуалізації
void RTTYRxView::on_stats(const RTTYStatsMessage& message) {
    // Відображаємо сильніший з двох сигналів (Mark або Space).
    // Це дає візуальне розуміння, що ми "піймали" тон RTTY.
    // Якщо приймається шум - значення буде малим.
    // Якщо приймається сигнал - смужка буде стрибати високо.
    
    uint32_t max_energy = message.mark_energy > message.space_energy
		? message.mark_energy
		: message.space_energy;
    // Оновлюємо значення бару
    tuning_bar.set_value(max_energy > 65535 ? 65535 : max_energy);
    
    // Опціонально: Можна змінювати колір смужки, якщо сигнал дуже сильний
    // (Але ProgressBar у Mayhem зазвичай одноколірний за замовчуванням)
}

void RTTYRxView::on_freqchg(int64_t freq) {
    field_frequency.set_value(freq);
}


}  // namespace ui::external_app::rtty_rx
