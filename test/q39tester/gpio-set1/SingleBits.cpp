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
#include <hal/gpio/IPinInput.hpp>
#include <hal/gpio/IPinOutput.hpp>

#include <catch2/catch.hpp>

#include <vector>

TEST_CASE("1. Toggle values of single pins", "[unit][mcp23x17]")
{
    hal::ScopedHardware hardware;

    {
        // Every outputIds[i] pin in connected to the inputIds[i] pin, so order of those lists matter.
        auto outputIds = {hal::device_id::eMcp23017PinOutPA0,
                          hal::device_id::eMcp23017PinOutPA1,
                          hal::device_id::eMcp23017PinOutPA2,
                          hal::device_id::eMcp23017PinOutPA3,
                          hal::device_id::eMcp23s17PinOutPA4,
                          hal::device_id::eMcp23s17PinOutPA5,
                          hal::device_id::eMcp23s17PinOutPA6,
                          hal::device_id::eMcp23s17PinOutPA7,
                          hal::device_id::eMcp23s17PinOutPB0,
                          hal::device_id::eMcp23s17PinOutPB1,
                          hal::device_id::eMcp23s17PinOutPB2,
                          hal::device_id::eMcp23s17PinOutPB3,
                          hal::device_id::eMcp23017PinOutPB4,
                          hal::device_id::eMcp23017PinOutPB5,
                          hal::device_id::eMcp23017PinOutPB6,
                          hal::device_id::eMcp23017PinOutPB7};

        auto inputIds = {hal::device_id::eMcp23s17PinInPA0,
                         hal::device_id::eMcp23s17PinInPA1,
                         hal::device_id::eMcp23s17PinInPA2,
                         hal::device_id::eMcp23s17PinInPA3,
                         hal::device_id::eMcp23017PinInPA4,
                         hal::device_id::eMcp23017PinInPA5,
                         hal::device_id::eMcp23017PinInPA6,
                         hal::device_id::eMcp23017PinInPA7,
                         hal::device_id::eMcp23017PinInPB0,
                         hal::device_id::eMcp23017PinInPB1,
                         hal::device_id::eMcp23017PinInPB2,
                         hal::device_id::eMcp23017PinInPB3,
                         hal::device_id::eMcp23s17PinInPB4,
                         hal::device_id::eMcp23s17PinInPB5,
                         hal::device_id::eMcp23s17PinInPB6,
                         hal::device_id::eMcp23s17PinInPB7};

        std::vector<hal::ScopedDevice<hal::gpio::IPinOutput>> outputs;
        std::vector<hal::ScopedDevice<hal::gpio::IPinInput>> inputs;

        for (const auto& cId : outputIds)
            outputs.emplace_back(hal::getDevice<hal::gpio::IPinOutput>(cId));

        for (const auto& cId : inputIds)
            inputs.emplace_back(hal::getDevice<hal::gpio::IPinInput>(cId));

        SECTION("1.1. Multiple toggling")
        {
            bool setValue = true;
            constexpr int cTestIterations = 100;
            for (int i = 0; i < cTestIterations; ++i) {
                for (auto& output : outputs) {
                    auto error = output->set(setValue);
                    REQUIRE(!error);
                }

                for (auto& input : inputs) {
                    auto [getValue, error] = input->get();
                    REQUIRE(!error);
                    REQUIRE(*getValue == setValue);
                }

                setValue = !setValue;
            }
        }

        SECTION("1.2. Set one pin to 1 and rest to 0, check all")
        {
            for (std::size_t i = 0; i < outputs.size(); ++i) {
                for (std::size_t j = 0; j < outputs.size(); ++j) {
                    auto error = outputs[j]->set(i == j);
                    REQUIRE(!error);
                }

                for (std::size_t j = 0; j < outputs.size(); ++j) {
                    auto [getValue, error] = inputs[j]->get();
                    REQUIRE(!error);
                    REQUIRE(*getValue == (i == j));
                }
            }
        }
    }
}
