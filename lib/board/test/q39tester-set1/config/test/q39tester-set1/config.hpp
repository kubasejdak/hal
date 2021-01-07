/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2020-2021, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include "hal/gpio/types.hpp"

#include <utils/property.hpp>

#include <tuple>

using SpiConfig = std::tuple<const char*, const char*, hal::gpio::Pin>;

ADD_PROPERTY_2(RaspberryPi, Mcp23S17, (SpiConfig{"spi0.0", "gpio-stub", hal::gpio::Pin::eBit0}));
ADD_PROPERTY_2(RaspberryPi, Mcp23017, "i2c1");
ADD_PROPERTY_2(RaspberryPi, GenericEeprom, "i2c1");
ADD_PROPERTY_2(RaspberryPi, M41T82, "i2c1");

namespace hal::config {

using MainBoardType = utils::PropertyType<MainBoard>;
constexpr auto cQ39TesterSet1Mcp23S17 = utils::cPropertyValue<MainBoardType, Mcp23S17>;
constexpr auto cQ39TesterSet1Mcp23017 = utils::cPropertyValue<MainBoardType, Mcp23017>;
constexpr auto cQ39TesterSet1GenericEeprom = utils::cPropertyValue<MainBoardType, GenericEeprom>;
constexpr auto cQ39TesterSet1M41T82 = utils::cPropertyValue<MainBoardType, M41T82>;

} // namespace hal::config
