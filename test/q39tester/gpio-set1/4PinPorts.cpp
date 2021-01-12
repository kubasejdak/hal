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

#include <hal/Hardware.hpp>
#include <hal/factory.hpp>
#include <hal/gpio/IPortInput.hpp>
#include <hal/gpio/IPortOutput.hpp>

#include <catch2/catch.hpp>

#include <cstdint>
#include <vector>

TEST_CASE("2. Set all combinations of pin patterns", "[unit][mcp23x17]")
{
    hal::ScopedHardware hardware;

    {
        auto normalOutputIds = {hal::device_id::eMcp23s17PortOut4BPA0u3, hal::device_id::eMcp23017PortOut4BPB0u3};
        auto invertedOutputIds = {hal::device_id::eMcp23017PortOut4BPA4u7, hal::device_id::eMcp23s17PortOut4BPB4u7};
        auto normalInputIds = {hal::device_id::eMcp23017PortIn4BPA0u3, hal::device_id::eMcp23s17PortIn4BPB0u3};
        auto invertedInputIds = {hal::device_id::eMcp23s17PortIn4BPA4u7, hal::device_id::eMcp23017PortIn4BPB4u7};

        std::vector<hal::ScopedDevice<hal::gpio::IPortOutput<std::uint8_t>>> normalOutputs;
        std::vector<hal::ScopedDevice<hal::gpio::IPortOutput<std::uint8_t>>> invertedOutputs;
        std::vector<hal::ScopedDevice<hal::gpio::IPortInput<std::uint8_t>>> normalInputs;
        std::vector<hal::ScopedDevice<hal::gpio::IPortInput<std::uint8_t>>> invertedInputs;

        for (const auto& cId : normalOutputIds)
            normalOutputs.emplace_back(hal::getDevice<hal::gpio::IPortOutput<std::uint8_t>>(cId));

        for (const auto& cId : invertedOutputIds)
            invertedOutputs.emplace_back(hal::getDevice<hal::gpio::IPortOutput<std::uint8_t>>(cId));

        for (const auto& cId : normalInputIds)
            normalInputs.emplace_back(hal::getDevice<hal::gpio::IPortInput<std::uint8_t>>(cId));

        for (const auto& cId : invertedInputIds)
            invertedInputs.emplace_back(hal::getDevice<hal::gpio::IPortInput<std::uint8_t>>(cId));

        constexpr std::uint8_t cPatternsCount = 16;
        constexpr std::uint8_t cPatternMask = 0x0f;
        for (std::uint8_t pattern = 0; pattern < cPatternsCount; ++pattern) {
            // Set normal pattern.
            for (auto& output : normalOutputs) {
                auto error = output->set(pattern);
                REQUIRE(!error);
            }

            // Set inverted pattern.
            for (auto& output : invertedOutputs) {
                auto error = output->set(~pattern);
                REQUIRE(!error);
            }

            // Verify normal pattern.
            for (auto& input : normalInputs) {
                std::uint8_t getValue{};
                auto error = input->get(getValue);
                REQUIRE(!error);
                auto expectedValue = pattern & cPatternMask;
                REQUIRE(getValue == expectedValue);
            }

            // Verify inverted pattern.
            for (auto& input : invertedInputs) {
                std::uint8_t getValue{};
                auto error = input->get(getValue);
                REQUIRE(!error);
                auto expectedValue = std::uint8_t(~pattern) & cPatternMask;
                REQUIRE(getValue == expectedValue);
            }
        }
    }
}
