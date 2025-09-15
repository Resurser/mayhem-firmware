/*
 * Copyright (C) 2025 Belousov Oleg
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

#include "gradient.hpp"

#include "convert.hpp"
#include "file_reader.hpp"
#include "file_path.hpp"

namespace fs = std::filesystem;

const std::filesystem::path default_gradient_file = u"waterfall.txt";

Gradient::Gradient() {
    prev_index = 0;
    prev_r = 0;
    prev_g = 0;
    prev_b = 0;

    
    file_list = scan_root_files(waterfalls_dir, u"*.txt");
}

Gradient::~Gradient() {
    delete_file(default_gradient_file);
    copy_file(current_gradient_file, default_gradient_file);    
}

void Gradient::set_default(const uint8_t index) {
    if (file_list.size() > 0 && (index < file_list.size())) {
        current_gradient_file = waterfalls_dir / file_list[index];
        if (!fs::file_exists(current_gradient_file)) {
            current_gradient_file = default_gradient_file;
        }
        prev_index = 0;
        prev_r = 0;
        prev_g = 0;
        prev_b = 0;

        load_file(current_gradient_file);
        return;
    }
    
    step(1, 0, 0, 0);
    step(37, 0, 0, 80);
    step(74, 0, 48, 150);
    step(115, 0, 110, 220);
    step(153, 50, 160, 110);
    step(192, 150, 192, 50);
    step(216, 245, 200, 0);
    step(242, 245, 100, 0);
    step(255, 255, 0, 0); 
    // switch (index) {
    // case 1:
    //     step(1, 0, 0, 0);
    //     step(64, 0, 0, 155);
    //     step(96, 0, 5, 255);
    //     step(128, 0, 255, 20);
    //     step(192, 255, 225, 0);
    //     step(255, 240, 0, 0);
    //     break;
    
    // default:    
        
    //     break;
    // }
}

bool Gradient::load_file(const std::filesystem::path& file_path) {
    File gradient_file;
    auto error = gradient_file.open(file_path.string());

    if (error)
        return false;

    auto reader = FileLineReader(gradient_file);
    for (const auto& line : reader) {
        if (line.length() == 0 || line[0] == '#')
            continue;  // Empty or comment line.

        auto cols = split_string(line, ',');

        if (cols.size() == 4) {
            int16_t index, r, g, b;

            if (!parse_int(cols[0], index) || index < 0 || index > 255)
                continue;

            if (!parse_int(cols[1], r) || r < 0 || r > 255)
                continue;

            if (!parse_int(cols[2], g) || g < 0 || g > 255)
                continue;

            if (!parse_int(cols[3], b) || b < 0 || b > 255)
                continue;

            step(index, r, g, b);
        }
    }

    return true;
}

void Gradient::step(int16_t index, int16_t r, int16_t g, int16_t b) {
    for (int16_t i = prev_index; i <= index; i++) {
        float x = (float)(i - prev_index) / (index - prev_index);
        float y = 1.0f - x;

        int16_t new_r = prev_r * y + r * x;
        int16_t new_g = prev_g * y + g * x;
        int16_t new_b = prev_b * y + b * x;

        lut[i] = ui::Color(new_r, new_g, new_b);
    }

    prev_index = index;
    prev_r = r;
    prev_g = g;
    prev_b = b;
}
