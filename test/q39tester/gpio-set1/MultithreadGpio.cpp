/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2020-2022, Kuba Sejdak <kuba.sejdak@gmail.com>
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
#include <hal/gpio/IPortInput.hpp>
#include <hal/gpio/IPortOutput.hpp>
#include <osal/Semaphore.hpp>
#include <osal/Thread.hpp>

#include <catch2/catch.hpp>

#include <array>
#include <cstdint>

static void noiseThreadAFunc(bool& stop,
                             hal::device_id::Q39TesterSet1Id outputId,
                             hal::device_id::Q39TesterSet1Id inputId,
                             osal::Semaphore& semA,
                             osal::Semaphore& semB)
{
    auto output = hal::getScopedDevice<hal::gpio::IPinOutput>(outputId);
    auto input = hal::getScopedDevice<hal::gpio::IPinInput>(inputId);

    // Synchronize startup.
    if (auto error = output->off())
        REQUIRE(!error);

    semA.signal();
    semB.wait();

    // Play ping-pong with other thread.
    bool setValue = true;
    while (!stop) {
        if (auto error = output->set(setValue))
            REQUIRE(!error);

        bool readValue{};
        do {
            osal::thread::yield();
            auto [value, error] = input->get();
            if (error)
                REQUIRE(!error);

            readValue = *value;
        }
        while (!stop && (readValue != setValue));

        setValue = !setValue;
    }

    // Cleanup.
    if (auto error = output->off())
        REQUIRE(!error);
}

static void noiseThreadBFunc(bool& stop,
                             hal::device_id::Q39TesterSet1Id inputId,
                             hal::device_id::Q39TesterSet1Id outputId,
                             osal::Semaphore& semA,
                             osal::Semaphore& semB)
{
    auto input = hal::getScopedDevice<hal::gpio::IPinInput>(inputId);
    auto output = hal::getScopedDevice<hal::gpio::IPinOutput>(outputId);

    // Synchronize startup.
    if (auto error = output->off())
        REQUIRE(!error);

    semA.wait();
    semB.signal();

    // Play ping-pong with other thread.
    bool expectedValue = true;
    while (!stop) {
        bool readValue{};
        do {
            osal::thread::yield();
            auto [value, error] = input->get();
            if (error)
                REQUIRE(!error);

            readValue = *value;
        }
        while (!stop && (readValue != expectedValue));

        if (auto error = output->set(expectedValue))
            REQUIRE(!error);

        expectedValue = !expectedValue;
    }

    // Cleanup.
    if (auto error = output->off())
        REQUIRE(!error);
}

static bool isCounterOverflow(bool countUp, std::uint8_t counter)
{
    constexpr std::uint8_t cCounterMin = 0x00;
    constexpr std::uint8_t cCounterMax = 0x1f;
    bool reachedMax = countUp && (counter == cCounterMax);
    bool reachedMin = !countUp && (counter == cCounterMin);

    return reachedMax || reachedMin;
}

static void triggerThreadFunc(bool& stop,
                              hal::device_id::Q39TesterSet1Id triggerOutputId,
                              hal::device_id::Q39TesterSet1Id counterInputId,
                              osal::Semaphore& triggerSem,
                              osal::Semaphore& counterSem)
{
    auto triggerOutput = hal::getScopedDevice<hal::gpio::IPinOutput>(triggerOutputId);
    auto counterInput = hal::getScopedDevice<hal::gpio::IPortInput<std::uint8_t>>(counterInputId);

    // Synchronize startup.
    if (auto error = triggerOutput->off())
        REQUIRE(!error);

    triggerSem.signal();
    counterSem.wait();

    // Perform test.
    bool triggerValue = true;
    bool countUp = true;
    int overflowCount{};
    std::uint8_t prevCounter{};

    while (!stop) {
        if (auto error = triggerOutput->set(triggerValue))
            REQUIRE(!error);

        osal::thread::yield();

        auto expectedCounter = prevCounter + (countUp ? 1 : -1);
        std::uint8_t counter{};
        do {
            auto [value, error] = counterInput->get();
            if (error)
                REQUIRE(!error);

            counter = *value;
            if ((counter != expectedCounter && counter != prevCounter))
                REQUIRE((counter == expectedCounter || counter == prevCounter));

            osal::thread::yield();
        }
        while (counter != expectedCounter);

        if (isCounterOverflow(countUp, counter)) {
            countUp = !countUp;
            ++overflowCount;
            constexpr int cMaxOverflowsCount = 50;
            if (overflowCount == cMaxOverflowsCount)
                stop = true;
        }

        triggerValue = !triggerValue;
        prevCounter = counter;
    }
}

static void counterThreadFunc(bool& stop,
                              hal::device_id::Q39TesterSet1Id triggerInputId,
                              hal::device_id::Q39TesterSet1Id counterOutputId,
                              osal::Semaphore& triggerSem,
                              osal::Semaphore& counterSem)
{
    auto triggerInput = hal::getScopedDevice<hal::gpio::IPinInput>(triggerInputId);
    auto counterOutput = hal::getScopedDevice<hal::gpio::IPortOutput<std::uint8_t>>(counterOutputId);

    // Synchronize startup.
    if (auto error = counterOutput->set(0x00))
        REQUIRE(!error);

    triggerSem.wait();
    counterSem.signal();

    bool expectedTriggerValue = true;
    bool countUp = true;
    std::uint8_t prevCounter{};

    while (!stop) {
        bool triggerValue{};
        auto [value, error] = triggerInput->get();
        if (error)
            REQUIRE(!error);

        triggerValue = *value;
        if (triggerValue != expectedTriggerValue) {
            osal::thread::yield();
            continue;
        }

        auto counter = prevCounter + (countUp ? 1 : -1);
        if (auto setError = counterOutput->set(counter))
            REQUIRE(!setError);

        osal::thread::yield();

        if (isCounterOverflow(countUp, counter))
            countUp = !countUp;

        expectedTriggerValue = !expectedTriggerValue;
        prevCounter = counter;
    }
}

TEST_CASE("3. Multithread ping-pong and counter setting", "[unit][mcp23x17]")
{
    hal::ScopedHardware hardware;

    // Create noise threads.
    struct TestConfig {
        hal::device_id::Q39TesterSet1Id threadAOutputId;
        hal::device_id::Q39TesterSet1Id threadAInputId;
        hal::device_id::Q39TesterSet1Id threadBInputId;
        hal::device_id::Q39TesterSet1Id threadBOutputId;
        osal::Semaphore semaphoreA{0};
        osal::Semaphore semaphoreB{0};
        osal::Thread<> threadA;
        osal::Thread<> threadB;
    };

    {
        namespace Id = hal::device_id;
        constexpr int cPairsCount = 2;
        // clang-format off
        std::array<TestConfig, cPairsCount> noiseConfigs = {{
            {Id::eMcp23s17PingPongOutPA0, Id::eMcp23s17PingPongInPA1, Id::eMcp23017PingPongInPA0, Id::eMcp23017PingPongOutPA1, osal::Semaphore{0}, osal::Semaphore{0}, {}, {}},
            {Id::eMcp23017PingPongOutPB0, Id::eMcp23017PingPongInPB1, Id::eMcp23s17PingPongInPB0, Id::eMcp23s17PingPongOutPB1, osal::Semaphore{0}, osal::Semaphore{0}, {}, {}}
        }};
        std::array<TestConfig, cPairsCount> counterConfigs = {{
            {Id::eMcp23s17TriggerOutPA2, Id::eMcp23s17CounterInPA3u7, Id::eMcp23017TriggerInPA2, Id::eMcp23017CounterOutPA3u7, osal::Semaphore{0}, osal::Semaphore{0}, {}, {}},
            {Id::eMcp23017TriggerOutPB2, Id::eMcp23017CounterInPB3u7, Id::eMcp23s17TriggerInPB2, Id::eMcp23s17CounterOutPB3u7, osal::Semaphore{0}, osal::Semaphore{0}, {}, {}}
        }};
        std::array<bool, cPairsCount> stopFlags{};
        // clang-format on

        for (int i = 0; i < cPairsCount; ++i) {
            auto& noise = noiseConfigs[i];
            auto& counter = counterConfigs[i];
            auto& stop = stopFlags[i];

            // Create noise threads.
            noise.threadA.start(noiseThreadAFunc,
                                std::ref(stop),
                                noise.threadAOutputId,
                                noise.threadAInputId,
                                std::ref(noise.semaphoreA),
                                std::ref(noise.semaphoreB));
            noise.threadB.start(noiseThreadBFunc,
                                std::ref(stop),
                                noise.threadBInputId,
                                noise.threadBOutputId,
                                std::ref(noise.semaphoreA),
                                std::ref(noise.semaphoreB));

            // Create counter threads.
            counter.threadA.start(triggerThreadFunc,
                                  std::ref(stop),
                                  counter.threadAOutputId,
                                  counter.threadAInputId,
                                  std::ref(counter.semaphoreA),
                                  std::ref(counter.semaphoreB));
            counter.threadB.start(counterThreadFunc,
                                  std::ref(stop),
                                  counter.threadBInputId,
                                  counter.threadBOutputId,
                                  std::ref(counter.semaphoreA),
                                  std::ref(counter.semaphoreB));
        }
    }
}
