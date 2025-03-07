/*
 * Copyright (C) 2022 Belousov Oleg
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

#ifndef __DSP_DDC_H__
#define __DSP_DDC_H__

#include <cstdint>
#include <cstddef>
#include "dsp_types.hpp"

namespace dsp {

class DDC {

public:

    void set_sample_rate(const int32_t x);
    void set_freq(const int32_t x);
    
    buffer_c16_t execute(const buffer_c16_t& src, const buffer_c16_t& dst);

private:

	int32_t	sample_rate { 0 };
	int32_t	phase_inc { 0 };
	int32_t	phase { 0 };
};
}

#endif/*__DSP_DDC_H__*/
