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

#include "hal/Board.hpp"
#include "hal/Error.hpp"
#include "hal/gpio/GpioPort.hpp"
#include "hal/gpio/Mcp23017.hpp"
#include "hal/gpio/Mcp23S17.hpp"
#include "hal/gpio/Mcp23x17Common.hpp"
#include "hal/gpio/PinInput.hpp"
#include "hal/gpio/PinOutput.hpp"
#include "hal/gpio/PortInput.hpp"
#include "hal/gpio/PortOutput.hpp"
#include "hal/gpio/types.hpp"
#include "hal/sensor/Sht3xDisHumidity.hpp"
#include "hal/sensor/Sht3xDisSensor.hpp"
#include "hal/sensor/Sht3xDisTemperature.hpp"
#include "hal/storage/GenericEeprom.hpp"
#include "hal/time/M41T82.hpp"
#include "product/config.hpp"
#include "test/q39tester-set1/DeviceId.hpp"

#include <chrono>

namespace hal {
namespace detail {

template <>
std::shared_ptr<Device> getDeviceImpl<device_id::Q39TesterSet1Id>(device_id::Q39TesterSet1Id id)
{
    return CurrentBoardVersion<device_id::Q39TesterSet1Id>::get()->getDevice(id);
}

} // namespace detail

static void initGpio(std::map<device_id::Q39TesterSet1Id, std::shared_ptr<Device>>& devices)
{
    using namespace std::chrono_literals;

    // clang-format off
    auto mcp23s17Spi = spi::Registry::get(std::get<0>(config::cQ39TesterSet1Mcp23S17));
    auto mcp23s17ChipSelectGpio = gpio::Registry<std::uint32_t>::get(std::get<1>(config::cQ39TesterSet1Mcp23S17));
    auto mcp23s17ChipSelectPin = std::get<2>(config::cQ39TesterSet1Mcp23S17);
    constexpr std::uint16_t cMcp23s17Addr = 0x20;
    constexpr auto cMcp23s17BusTimeout = 5s;
    std::shared_ptr<gpio::IPinOutput> mcp23s17ChipSelect = std::make_shared<gpio::PinOutput<std::uint32_t>>(mcp23s17ChipSelectGpio, mcp23s17ChipSelectPin, true);

    auto initMcp23s17PortA = [&] {
        constexpr std::uint8_t cDirection = 0x0f;
        auto mcp23s17 = std::make_shared<gpio::GpioPort<std::uint8_t>>(std::make_shared<gpio::Mcp23S17>(mcp23s17Spi, mcp23s17ChipSelect, cMcp23s17Addr, gpio::mcp23x17::Port::eGpioA, cMcp23s17BusTimeout, osal::Timeout::none(), osal::Timeout::none()), cDirection);

        devices[device_id::eMcp23s17PinInPA0] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit0);
        devices[device_id::eMcp23s17PinInPA1] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit1);
        devices[device_id::eMcp23s17PinInPA2] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit2);
        devices[device_id::eMcp23s17PinInPA3] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit3);
        devices[device_id::eMcp23s17PinOutPA4] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit4);
        devices[device_id::eMcp23s17PinOutPA5] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit5);
        devices[device_id::eMcp23s17PinOutPA6] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit6);
        devices[device_id::eMcp23s17PinOutPA7] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit7);

        constexpr std::uint8_t cLowBitsMask = 0x0f;
        constexpr std::uint8_t cHighBitsMask = 0xf0;
        devices[device_id::eMcp23s17PortOut4BPA0u3] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint8_t>>(mcp23s17, cLowBitsMask);
        devices[device_id::eMcp23s17PortIn4BPA4u7] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint8_t>>(mcp23s17, cHighBitsMask, [](std::uint8_t value, std::uint8_t /*unused*/) { return value >> 4U; });

        constexpr std::uint8_t cCounterBitsMask = 0xf8;
        devices[device_id::eMcp23s17PingPongOutPA0] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit0);
        devices[device_id::eMcp23s17PingPongInPA1] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit1);
        devices[device_id::eMcp23s17TriggerOutPA2] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit2);
        devices[device_id::eMcp23s17CounterInPA3u7] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint8_t>>(mcp23s17, cCounterBitsMask, [](std::uint8_t value, std::uint8_t /*unused*/) { return value >> 3U; });
    };

    auto initMcp23s17PortB = [&] {
        constexpr std::uint8_t cDirection = 0xf0;
        auto mcp23s17 = std::make_shared<gpio::GpioPort<std::uint8_t>>(std::make_shared<gpio::Mcp23S17>(mcp23s17Spi, mcp23s17ChipSelect, cMcp23s17Addr, gpio::mcp23x17::Port::eGpioB, cMcp23s17BusTimeout, osal::Timeout::none(), osal::Timeout::none()), cDirection);

        devices[device_id::eMcp23s17PinOutPB0] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit0);
        devices[device_id::eMcp23s17PinOutPB1] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit1);
        devices[device_id::eMcp23s17PinOutPB2] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit2);
        devices[device_id::eMcp23s17PinOutPB3] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit3);
        devices[device_id::eMcp23s17PinInPB4] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit4);
        devices[device_id::eMcp23s17PinInPB5] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit5);
        devices[device_id::eMcp23s17PinInPB6] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit6);
        devices[device_id::eMcp23s17PinInPB7] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit7);

        constexpr std::uint8_t cLowBitsMask = 0x0f;
        constexpr std::uint8_t cHighBitsMask = 0xf0;
        devices[device_id::eMcp23s17PortIn4BPB0u3] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint8_t>>(mcp23s17, cLowBitsMask);
        devices[device_id::eMcp23s17PortOut4BPB4u7] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint8_t>>(mcp23s17, cHighBitsMask, [](std::uint8_t value, std::uint8_t /*unused*/) { return value << 4U; });

        constexpr std::uint8_t cCounterBitsMask = 0xf8;
        devices[device_id::eMcp23s17PingPongInPB0] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit0);
        devices[device_id::eMcp23s17PingPongOutPB1] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit1);
        devices[device_id::eMcp23s17TriggerInPB2] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23s17, gpio::Pin::eBit2);
        devices[device_id::eMcp23s17CounterOutPB3u7] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint8_t>>(mcp23s17, cCounterBitsMask, [](std::uint8_t value, std::uint8_t /*unused*/) { return value << 3U; });
    };

    auto mcp23017I2c = i2c::Registry::get(config::cQ39TesterSet1Mcp23017);
    constexpr std::uint16_t cMcp23017Addr = 0x20;
    constexpr auto cMcp23017BusTimeout = 5s;

    auto initMcp23017PortA = [&] {
        constexpr auto cDirection = 0xf0;
        auto mcp23017 = std::make_shared<gpio::GpioPort<std::uint8_t>>(std::make_shared<gpio::Mcp23017>(mcp23017I2c, cMcp23017Addr, gpio::mcp23x17::Port::eGpioA, cMcp23017BusTimeout, osal::Timeout::none(), osal::Timeout::none()), cDirection);

        devices[device_id::eMcp23017PinOutPA0] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit0);
        devices[device_id::eMcp23017PinOutPA1] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit1);
        devices[device_id::eMcp23017PinOutPA2] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit2);
        devices[device_id::eMcp23017PinOutPA3] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit3);
        devices[device_id::eMcp23017PinInPA4] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit4);
        devices[device_id::eMcp23017PinInPA5] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit5);
        devices[device_id::eMcp23017PinInPA6] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit6);
        devices[device_id::eMcp23017PinInPA7] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit7);

        constexpr std::uint8_t cLowBitsMask = 0x0f;
        constexpr std::uint8_t cHighBitsMask = 0xf0;
        devices[device_id::eMcp23017PortIn4BPA0u3] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint8_t>>(mcp23017, cLowBitsMask);
        devices[device_id::eMcp23017PortOut4BPA4u7] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint8_t>>(mcp23017, cHighBitsMask, [](std::uint8_t value, std::uint8_t /*unused*/) { return value << 4U; });

        constexpr std::uint8_t cCounterBitsMask = 0xf8;
        devices[device_id::eMcp23017PingPongInPA0] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit0);
        devices[device_id::eMcp23017PingPongOutPA1] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit1);
        devices[device_id::eMcp23017TriggerInPA2] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit2);
        devices[device_id::eMcp23017CounterOutPA3u7] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint8_t>>(mcp23017, cCounterBitsMask, [](std::uint8_t value, std::uint8_t /*unused*/) { return value << 3U; });
    };

    auto initMcp23017PortB = [&] {
        constexpr auto cDirection = 0x0f;
        auto mcp23017 = std::make_shared<gpio::GpioPort<std::uint8_t>>(std::make_shared<gpio::Mcp23017>(mcp23017I2c, cMcp23017Addr, gpio::mcp23x17::Port::eGpioB, cMcp23017BusTimeout, osal::Timeout::none(), osal::Timeout::none()), cDirection);

        devices[device_id::eMcp23017PinInPB0] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit0);
        devices[device_id::eMcp23017PinInPB1] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit1);
        devices[device_id::eMcp23017PinInPB2] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit2);
        devices[device_id::eMcp23017PinInPB3] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit3);
        devices[device_id::eMcp23017PinOutPB4] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit4);
        devices[device_id::eMcp23017PinOutPB5] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit5);
        devices[device_id::eMcp23017PinOutPB6] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit6);
        devices[device_id::eMcp23017PinOutPB7] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit7);

        constexpr std::uint8_t cLowBitsMask = 0x0f;
        constexpr std::uint8_t cHighBitsMask = 0xf0;
        devices[device_id::eMcp23017PortOut4BPB0u3] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint8_t>>(mcp23017, cLowBitsMask);
        devices[device_id::eMcp23017PortIn4BPB4u7] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint8_t>>(mcp23017, cHighBitsMask, [](std::uint8_t value, std::uint8_t /*unused*/) { return value >> 4U; });

        constexpr std::uint8_t cCounterBitsMask = 0xf8;
        devices[device_id::eMcp23017PingPongOutPB0] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit0);
        devices[device_id::eMcp23017PingPongInPB1] = std::make_shared<gpio::PinInput<std::uint8_t>>(mcp23017, gpio::Pin::eBit1);
        devices[device_id::eMcp23017TriggerOutPB2] = std::make_shared<gpio::PinOutput<std::uint8_t>>(mcp23017, gpio::Pin::eBit2);
        devices[device_id::eMcp23017CounterInPB3u7] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint8_t>>(mcp23017, cCounterBitsMask, [](std::uint8_t value, std::uint8_t /*unused*/) { return value >> 3U; });
    };

    initMcp23s17PortA();
    initMcp23s17PortB();
    initMcp23017PortA();
    initMcp23017PortB();
    // clang-format on
}

static void initStorage(std::map<device_id::Q39TesterSet1Id, std::shared_ptr<Device>>& devices)
{
    // clang-format off
    auto initAt24Cm02 = [&] {
        auto i2c = i2c::Registry::get(config::cQ39TesterSet1GenericEeprom);
        constexpr std::uint16_t cEepromAddr1 = 0x50;
        constexpr std::uint16_t cEepromAddr2 = 0x54;
        constexpr std::size_t cSize = 256 * 1024;
        constexpr std::size_t cPageSize = 256;
        constexpr auto cWriteDelay = 10ms;

        devices[device_id::eAt24Cm02Eeprom1] = std::make_shared<storage::GenericEeprom>(i2c, cEepromAddr1, i2c::AddressingMode::e7bit, cSize, cPageSize, cWriteDelay);
        devices[device_id::eAt24Cm02Eeprom2] = std::make_shared<storage::GenericEeprom>(i2c, cEepromAddr2, i2c::AddressingMode::e7bit, cSize, cPageSize, cWriteDelay);
    };

    initAt24Cm02();
    // clang-format on
}

static void initRtc(std::map<device_id::Q39TesterSet1Id, std::shared_ptr<Device>>& devices)
{
    auto i2c = i2c::Registry::get(config::cQ39TesterSet1M41T82);
    constexpr std::uint16_t cRtcAddr = 0x68;

    devices[device_id::eM41T82Rtc] = std::make_shared<time::M41T82>(i2c, cRtcAddr);
}

static void initSensor(std::map<device_id::Q39TesterSet1Id, std::shared_ptr<Device>>& devices)
{
    auto i2c = i2c::Registry::get(config::cQ39TesterSet1SHT3xDIS);
    constexpr std::uint16_t cSensorAddr = 0x44;
    auto sht3xdis = std::make_shared<sensor::Sht3xDisSensor>(i2c, cSensorAddr);

    devices[device_id::eSht3xDisHumidity] = std::make_shared<sensor::Sht3xDisHumidity>(sht3xdis);
    devices[device_id::eSht3xDisTemperature] = std::make_shared<sensor::Sht3xDisTemperature>(sht3xdis);
}

template <>
std::error_code Board<device_id::Q39TesterSet1Id>::initImpl()
{
    initGpio(m_devices);
    initStorage(m_devices);
    initRtc(m_devices);
    initSensor(m_devices);

    return Error::eOk;
}

} // namespace hal
