/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2022, Kuba Sejdak <kuba.sejdak@gmail.com>
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

TEST_CASE("1. Toggle values of single pins", "[unit][gpio]")
{
    hal::ScopedHardware hardware;
    REQUIRE(hardware.initialized());

    auto outputIds = {hal::device_id::ePortAPin0,
                      hal::device_id::ePortAPin1,
                      hal::device_id::ePortAPin2,
                      hal::device_id::ePortAPin3,
                      hal::device_id::ePortAPin4,
                      hal::device_id::ePortAPin5,
                      hal::device_id::ePortAPin6,
                      hal::device_id::ePortAPin7};

    auto inputIds = {hal::device_id::ePortBPin0,
                     hal::device_id::ePortBPin1,
                     hal::device_id::ePortBPin2,
                     hal::device_id::ePortBPin3,
                     hal::device_id::ePortBPin4,
                     hal::device_id::ePortBPin5,
                     hal::device_id::ePortBPin6,
                     hal::device_id::ePortBPin7};

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

    SECTION("1.2. Set first and last bit outputs to low, check all")
    {
        // Set all outputs to high.
        for (auto& output : outputs) {
            auto error = output->on();
            REQUIRE(!error);
        }

        // Verify initial setup.
        for (auto& input : inputs) {
            auto [getValue, error] = input->get();
            REQUIRE(!error);
            REQUIRE(*getValue);
        }

        // Set first and last bit outputs to low.
        outputs.front()->off();
        outputs.back()->off();

        // Verify test setup.
        for (std::size_t i = 0; i < inputs.size(); ++i) {
            auto [getValue, error] = inputs[i]->get();
            REQUIRE(!error);
            bool shouldBeLow = (i == 0 || i == (inputs.size() - 1));
            REQUIRE(*getValue == !shouldBeLow);
        }
    }

    SECTION("1.3. Toggle only one pin at a time")
    {
        for (std::size_t i = 0; i < outputs.size(); ++i) {
            for (std::size_t j = 0; j < outputs.size(); ++j) {
                bool value = (i == j);
                auto error = outputs[j]->set(value);
                REQUIRE(!error);
            }

            for (std::size_t j = 0; j < outputs.size(); ++j) {
                auto [getValue, error] = inputs[j]->get();
                REQUIRE(!error);

                bool expectedValue = (i == j);
                REQUIRE(expectedValue == *getValue);
            }
        }
    }
}
