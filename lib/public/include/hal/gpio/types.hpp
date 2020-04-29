/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2020, Kuba Sejdak <kuba.sejdak@gmail.com>
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///
/// 1. Redistributions of source code must retain the above copyright notice, this
///    list of conditions and the following disclaimer.
///
/// 2. Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
/// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
/// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
/// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
/// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
/// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
/////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <bitset>
#include <cassert>
#include <cstdint>
#include <type_traits>
#include <vector>

namespace hal::gpio {

/// Represents possible bits from the GPIO ports.
enum class Pin {
    eBit0,
    eBit1,
    eBit2,
    eBit3,
    eBit4,
    eBit5,
    eBit6,
    eBit7,
    eBit8,
    eBit9,
    eBit10,
    eBit11,
    eBit12,
    eBit13,
    eBit14,
    eBit15,
    eBit16,
    eBit17,
    eBit18,
    eBit19,
    eBit20,
    eBit21,
    eBit22,
    eBit23,
    eBit24,
    eBit25,
    eBit26,
    eBit27,
    eBit28,
    eBit29,
    eBit30,
    eBit31,
    eBit32,
    eBit33,
    eBit34,
    eBit35,
    eBit36,
    eBit37,
    eBit38,
    eBit39,
    eBit40,
    eBit41,
    eBit42,
    eBit43,
    eBit44,
    eBit45,
    eBit46,
    eBit47,
    eBit48,
    eBit49,
    eBit50,
    eBit51,
    eBit52,
    eBit53,
    eBit54,
    eBit55,
    eBit56,
    eBit57,
    eBit58,
    eBit59,
    eBit60,
    eBit61,
    eBit62,
    eBit63
};

/// Converts enum representing GPIO pin id to its numeric representation.
/// @param pin              Pin id to be converted.
/// @return Numeric representation of the GPIO pin id.
constexpr int toInt(Pin pin)
{
    return static_cast<int>(pin);
}

/// Helper flag for checking if a given type can be used as WidthType type in the GPIO middleware.
/// @tparam WidthType       Unsigned type representing bitness of the given port (e.g. std::uint32_t is 32bit).
template <typename WidthType>
constexpr inline bool cIsValidWidthType
    = (std::is_unsigned_v<WidthType> && !std::is_same_v<WidthType, bool> && sizeof(WidthType) <= 8);

/// Returns maximal bit id that can be represented by the given width type.
/// @tparam WidthType       Unsigned type representing bitness of the given port (e.g. std::uint32_t is 32bit).
/// @return Maximal bit id that can be represented by the given width type.
template <typename WidthType>
constexpr Pin maxPin()
{
    switch (sizeof(WidthType)) {
        case 1: return Pin::eBit7;
        case 2: return Pin::eBit15;
        case 4: return Pin::eBit31;
        case 8: return Pin::eBit63; // NOLINT
        default: assert(false);
    }
}

/// Helper type representing bit mask with the specified width.
/// @tparam WidthType       Unsigned type representing bitness of the given port (e.g. std::uint32_t is 32bit).
template <typename WidthType>
using PinMask = std::bitset<toInt(maxPin<WidthType>())>;

} // namespace hal::gpio
