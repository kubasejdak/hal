/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2023, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include <hal/Hardware.hpp>
#include <hal/factory.hpp>
#include <hal/gpio/IPortInput.hpp>
#include <hal/gpio/IPortOutput.hpp>

#include <catch2/catch.hpp>

#include <cstdint>
#include <optional>

TEST_CASE("2. Set all combinations of bit patterns on 4bit ports", "[unit][gpio]")
{
    hal::ScopedHardware hardware;
    REQUIRE(hardware.initialized());

    // clang-format off
    auto outputSet0 = hal::getScopedDevice<hal::gpio::IPortOutput<std::uint8_t>>(hal::device_id::PlatformGpioId::ePortAPinSet0);
    auto outputSet1 = hal::getScopedDevice<hal::gpio::IPortOutput<std::uint8_t>>(hal::device_id::PlatformGpioId::ePortAPinSet1);

    auto inputSet0 = hal::getScopedDevice<hal::gpio::IPortInput<std::uint8_t>>(hal::device_id::PlatformGpioId::ePortBPinSet0);
    auto inputSet1 = hal::getScopedDevice<hal::gpio::IPortInput<std::uint8_t>>(hal::device_id::PlatformGpioId::ePortBPinSet1);
    // clang-format on

    constexpr std::uint8_t cPatternsCount = 15;
    constexpr std::uint8_t cPatternMask = 0x0f;
    for (std::uint8_t pattern = 0; pattern < cPatternsCount; ++pattern) {
        // Set pattern.
        auto error = outputSet0->set(pattern);
        REQUIRE(!error);

        error = outputSet1->set(~pattern);
        REQUIRE(!error);

        // Verify pattern.
        std::optional<std::uint8_t> getValue;
        std::tie(getValue, error) = inputSet0->get().toTuple();
        REQUIRE(!error);
        auto expectedValue = pattern & cPatternMask;
        REQUIRE(getValue == expectedValue);

        std::tie(getValue, error) = inputSet1->get().toTuple();
        REQUIRE(!error);
        expectedValue = std::uint8_t(~pattern) & cPatternMask;
        REQUIRE(*getValue == expectedValue);
    }
}
