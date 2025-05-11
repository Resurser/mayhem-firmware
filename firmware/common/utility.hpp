/*
 * Copyright (C) 2014 Jared Boone, ShareBrained Technology, Inc.
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

#ifndef __UTILITY_H__
#define __UTILITY_H__

#include <algorithm>
#include <array>
#include <complex>
#include <cstdint>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <string_view>
#include <type_traits>


// --- Константи ITA-2 ---
constexpr uint8_t ITA2_LTRS_SHIFT_CODE = 0x1F; // 11111
constexpr uint8_t ITA2_FIGS_SHIFT_CODE = 0x1B; // 11011
constexpr uint8_t ITA2_CODE_MASK = 0x1F; // Маска для 5 біт

// --- Таблиці символів ITA-2 (Baudot) ---
static const char ita2_letters_table[32] = {
 // 0    1    2    3    4    5    6    7     8    9    A    B    C    D    E    F
    '?', 'E', '\n','A', ' ', 'S', 'I', 'U', '\r','D', 'R', 'J', 'N', 'F', 'C', 'K', // 0x00 - 0x0F
    'T', 'Z', 'L', 'W', 'H', 'Y', 'P', 'Q', 'O', 'B', 'G', '?', 'M', 'X', 'V', '?'  // 0x10 - 0x1F (0x1F = LTRS)
};

static const char ita2_figures_table[32] = {
 // 0    1    2    3    4    5    6    7     8    9    A    B    C    D    E    F
    '?', '3', '\n','-', ' ', '\'', '8', '7', '\r','?', '4', '\a', ',', '!', ':', '(', // 0x00 - 0x0F (\a = BEL) ('?' для $ в US варіанті)
    '5', '+', ')', '2', '$', '6', '0', '1', '9', '?', '=', '?', '.', '/', ';', '?'  // 0x10 - 0x1F ('?' для £, =, FIG) (0x1B = FIGS)
};

// --- Функція пошуку символу ITA-2 ---
inline char lookup_ita2(uint8_t code, bool &is_in_figures_mode) {
    code &= ITA2_CODE_MASK;

    if (code == ITA2_LTRS_SHIFT_CODE) {
        is_in_figures_mode = false;
        return 0;
    }
    if (code == ITA2_FIGS_SHIFT_CODE) {
        is_in_figures_mode = true;
        return 0;
    }

    if (is_in_figures_mode) {
        return ita2_figures_table[code];
    } else {
        return ita2_letters_table[code];
    }
}


#define LOCATE_IN_RAM __attribute__((section(".ramtext")))

inline uint16_t fb_to_uint16(const std::string& fb) {
    return (fb[1] << 8) + fb[0];
}

inline uint32_t fb_to_uint32(const std::string& fb) {
    return (fb[3] << 24) + (fb[2] << 16) + (fb[1] << 8) + fb[0];
}

constexpr size_t operator"" _KiB(unsigned long long v) {
    return v * 1024;
}

constexpr size_t operator"" _MiB(unsigned long long v) {
    return v * 1024 * 1024;
}

template <typename E>
constexpr typename std::underlying_type<E>::type toUType(E enumerator) noexcept {
    /* Thanks, Scott Meyers! */
    return static_cast<typename std::underlying_type<E>::type>(enumerator);
}

/* Increments an enum value. Enumerator values are assumed to be serial. */
template <typename E>
void incr(E& e) {
    e = static_cast<E>(toUType(e) + 1);
}

inline uint32_t flp2(uint32_t v) {
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v - (v >> 1);
}

uint32_t gcd(const uint32_t u, const uint32_t v);

template <class T>
inline constexpr T pow(const T base, unsigned const exponent) {
    return (exponent == 0) ? 1 : (base * pow(base, exponent - 1));
}

constexpr bool power_of_two(const size_t n) {
    return (n & (n - 1)) == 0;
}

constexpr size_t log_2(const size_t n, const size_t p = 0) {
    return (n <= 1) ? p : log_2(n / 2, p + 1);
}

std::string bitsToText(const std::vector<int>& bits, const uint16_t word_length = 5) ;
float fast_log2(const float val);
float fast_pow2(const float val);

float mag2_to_dbv_norm(const float mag2);

inline float magnitude_squared(const std::complex<float> c) {
    const auto r = c.real();
    const auto r2 = r * r;
    const auto i = c.imag();
    const auto i2 = i * i;
    return r2 + i2;
}

/* Compute the duration in ms of a buffer. */
inline uint32_t ms_duration(
    uint64_t buffer_size,
    uint32_t sample_rate,
    uint32_t bytes_per_sample) {
    /* bytes * sample * second  * ms
     *         ------   -------   ------
     *         bytes    samples   seconds
     */
    if (sample_rate == 0 || bytes_per_sample == 0)
        return 0;

    return buffer_size * 1000 / (bytes_per_sample * sample_rate);
}

int fast_int_magnitude(int y, int x);
int int_atan2(int y, int x);
int32_t int16_sin_s4(int32_t x);

template <typename TEnum>
struct is_flags_type {
    static constexpr bool value = false;
};

template <typename TEnum>
constexpr bool is_flags_type_v = is_flags_type<TEnum>::value;

#define ENABLE_FLAGS_OPERATORS(type) \
    template <>                      \
    struct is_flags_type<type> { static constexpr bool value = true; };

template <typename TEnum>
constexpr std::enable_if_t<is_flags_type_v<TEnum>, TEnum> operator|(TEnum a, TEnum b) {
    using under_t = std::underlying_type_t<TEnum>;
    return static_cast<TEnum>(static_cast<under_t>(a) | static_cast<under_t>(b));
}

template <typename TEnum>
constexpr std::enable_if_t<is_flags_type_v<TEnum>, bool> flags_enabled(TEnum value, TEnum flags) {
    auto i_value = static_cast<std::underlying_type_t<TEnum>>(value);
    auto i_flags = static_cast<std::underlying_type_t<TEnum>>(flags);

    return (i_value & i_flags) == i_flags;
}

// TODO: Constrain to integrals?
/* Converts an integer into a byte array. */
template <typename T, size_t N = sizeof(T)>
constexpr std::array<uint8_t, N> to_byte_array(T value) {
    std::array<uint8_t, N> bytes{};
    for (size_t i = 0; i < N; ++i)
        bytes[i] = (value >> ((N - i - 1) * 8)) & 0xFF;
    return bytes;
}

/* Returns value constrained to min and max. */
template <class T>
constexpr const T& clip(const T& value, const T& minimum, const T& maximum) {
    return std::max(std::min(value, maximum), minimum);
}

/* Saves state on construction and reverts it when destroyed. */
template <typename T>
struct Stash {
    Stash(T& target)
        : target_{target}, prev_{target} {
    }

    ~Stash() {
        target_ = std::move(prev_);
    }

   private:
    T& target_;
    T prev_;
};

// TODO: need to decide if this is inclusive or exclusive.
// The implementations are different and cause the subtle
// bugs mentioned below.
template <class T>
struct range_t {
    const T minimum;
    const T maximum;

    constexpr const T& clip(const T& value) const {
        return ::clip(value, minimum, maximum);
    }

    constexpr void reset_if_outside(T& value, const T& reset_value) const {
        if ((value < minimum) ||
            (value > maximum)) {
            value = reset_value;
        }
    }

    constexpr bool below_range(const T& value) const {
        return value < minimum;
    }

    /* Exclusive of maximum. */
    constexpr bool contains(const T& value) const {
        return (value >= minimum) && (value < maximum);
    }

    /* Inclusive of maximum. */
    constexpr bool contains_inc(const T& value) const {
        return (value >= minimum) && (value <= maximum);
    }

    /* Exclusive of maximum. */
    constexpr bool out_of_range(const T& value) const {
        return !contains(value);
    }
};


std::string join(char c, std::initializer_list<std::string_view> strings);

uint32_t simple_checksum(uint32_t buffer_address, uint32_t length);

#endif /*__UTILITY_H__*/
