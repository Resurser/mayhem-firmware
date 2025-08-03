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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef __SINE_TABLE_I16_H__
#define __SINE_TABLE_I16_H__

#include <cmath>

static const int32_t sine_table_i16[256] = {
    0, 804, 1607, 2410, 3211, 4011, 4808, 5602, 6392, 7179, 7961, 8739,
    9512, 10278, 11039, 11793, 12539, 13278, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868,
    19519, 20159, 20787, 21403, 22005, 22594, 23170, 23732, 24279, 24812, 25330, 25832, 26319, 26790,
    27245, 27684, 28106, 28511, 28898, 29269, 29621, 29956, 30273, 30572, 30852, 31114, 31357, 31581,
    31785, 31971, 32138, 32285, 32413, 32521, 32610, 32679, 32728, 32758, 32768, 32758, 32728, 32679,
    32610, 32521, 32413, 32285, 32138, 31971, 31785, 31581, 31357, 31114, 30852, 30572, 30273, 29956,
    29621, 29269, 28898, 28511, 28106, 27684, 27245, 26790, 26319, 25832, 25330, 24812, 24279, 23732,
    23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151, 15446, 14732,
    14010, 13278, 12539, 11793, 11039, 10278, 9512, 8739, 7961, 7179, 6392, 5602, 4808, 4011, 3211,
    2410, 1607, 804, 0, -805, -1608, -2411, -3212, -4012, -4809, -5603, -6393, -7180, -7962, -8740,
    -9513, -10279, -11040, -11794, -12540, -13279, -14011, -14733, -15447, -16152, -16847, -17531,
    -18205, -18869, -19520, -20160, -20788, -21404, -22006, -22595, -23171, -23733, -24280, -24813,
    -25331, -25833, -26320, -26791, -27246, -27685, -28107, -28512, -28899, -29270, -29622, -29957,
    -30274, -30573, -30853, -31115, -31358, -31582, -31786, -31972, -32139, -32286, -32414, -32522,
    -32611, -32680, -32729, -32759, -32768, -32759, -32729, -32680, -32611, -32522, -32414, -32286,
    -32139, -31972, -31786, -31582, -31358, -31115, -30853, -30573, -30274, -29957, -29622, -29270,
    -28899, -28512, -28107, -27685, -27246, -26791, -26320, -25833, -25331, -24813, -24280, -23733,
    -23171, -22595, -22006, -21404, -20788, -20160, -19520, -18869, -18205, -17531, -16847, -16152,
    -15447, -14733, -14011, -13279, -12540, -11794, -11040, -10279, -9513, -8740, -7962, -7180, -6393,
    -5603, -4809, -4012, -3212, -2411, -1608, -805};

inline int32_t fastSin(uint32_t phase) {
    uint16_t index = (phase >> 16) & 255;    // Extract table index
    uint16_t nextIndex = (index + 1) & 255;  // Next index (wrap around)
    uint16_t fractional = (phase & 0xFFFF) >> 8;    // Fractional part (8-bit resolution)

    // Perform linear interpolation
    int16_t value1 = sine_table_i16[index];
    int16_t value2 = sine_table_i16[nextIndex];
    return value1 + ((value2 - value1) * fractional / 256);
}
#endif /*__SINE_TABLE_I8_H__*/
