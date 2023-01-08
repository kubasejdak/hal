/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2020-2023, Kuba Sejdak <kuba.sejdak@gmail.com>
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
#include "hal/gpio/IGpioPort.hpp"
#include "hal/gpio/PinInput.hpp"
#include "hal/gpio/PinOutput.hpp"
#include "hal/gpio/PortInput.hpp"
#include "hal/gpio/PortOutput.hpp"
#include "product/config.hpp"
#include "test/platform-gpio/DeviceId.hpp"

#include <bitset>
#include <climits>

namespace hal {
namespace detail {

template <>
std::shared_ptr<Device> getDeviceImpl<device_id::PlatformGpioId>(device_id::PlatformGpioId id)
{
    return CurrentBoardVersion<device_id::PlatformGpioId>::get()->getDevice(id);
}

} // namespace detail

/// Converts client data to the physical pin configuration of the pin set 0.
/// @param value        Client data to be set in the GPIO port.
/// @return Physical representation of the client's data.
static std::uint32_t portAPinSet0Modifier(std::uint8_t value, std::uint32_t /*unused*/)
{
    std::bitset<sizeof(std::uint8_t) * CHAR_BIT> valueBits(value);
    std::bitset<sizeof(std::uint32_t) * CHAR_BIT> returnBits(0);

    returnBits[4] = valueBits[0];  // NOLINT
    returnBits[5] = valueBits[1];  // NOLINT
    returnBits[6] = valueBits[2];  // NOLINT
    returnBits[13] = valueBits[3]; // NOLINT

    return returnBits.to_ulong();
}

/// Converts client data to the physical pin configuration of the pin set 1.
/// @param value        Client data to be set in the GPIO port.
/// @return Physical representation of the client's data.
static std::uint32_t portAPinSet1Modifier(std::uint8_t value, std::uint32_t /*unused*/)
{
    std::bitset<sizeof(std::uint8_t) * CHAR_BIT> valueBits(value);
    std::bitset<sizeof(std::uint32_t) * CHAR_BIT> returnBits(0);

    returnBits[17] = valueBits[0]; // NOLINT
    returnBits[22] = valueBits[1]; // NOLINT
    returnBits[26] = valueBits[2]; // NOLINT
    returnBits[27] = valueBits[3]; // NOLINT

    return returnBits.to_ulong();
}

/// Converts physical data of the pin set 0 to the client's value.
/// @param value        Physical data read from theGPIO port.
/// @return Client's value of the pin set 0 data.
static std::uint8_t portBPinSet0Modifier(std::uint32_t value, std::uint32_t /*unused*/)
{
    std::bitset<sizeof(std::uint32_t) * CHAR_BIT> valueBits(value);
    std::bitset<sizeof(std::uint8_t) * CHAR_BIT> returnBits(0);

    returnBits[0] = valueBits[18]; // NOLINT
    returnBits[1] = valueBits[12]; // NOLINT
    returnBits[2] = valueBits[16]; // NOLINT
    returnBits[3] = valueBits[20]; // NOLINT

    return returnBits.to_ulong();
}

/// Converts physical data of the pin set 1 to the client's value.
/// @param value        Physical data read from theGPIO port.
/// @return Client's value of the pin set 1 data.
static std::uint8_t portBPinSet1Modifier(std::uint32_t value, std::uint32_t /*unused*/)
{
    std::bitset<sizeof(std::uint32_t) * CHAR_BIT> valueBits(value);
    std::bitset<sizeof(std::uint8_t) * CHAR_BIT> returnBits(0);

    returnBits[0] = valueBits[23]; // NOLINT
    returnBits[1] = valueBits[25]; // NOLINT
    returnBits[2] = valueBits[21]; // NOLINT
    returnBits[3] = valueBits[24]; // NOLINT

    return returnBits.to_ulong();
}

template <>
std::error_code Board<device_id::PlatformGpioId>::initImpl()
{
    // clang-format off
    auto gpioA = gpio::Registry<std::uint32_t>::get(config::cPlatformGpioA);

    auto initPortA = [&] {
        m_devices[device_id::ePortAPin0] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpioA, gpio::Pin::eBit4);
        m_devices[device_id::ePortAPin1] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpioA, gpio::Pin::eBit17);
        m_devices[device_id::ePortAPin2] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpioA, gpio::Pin::eBit27);
        m_devices[device_id::ePortAPin3] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpioA, gpio::Pin::eBit22);
        m_devices[device_id::ePortAPin4] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpioA, gpio::Pin::eBit5);
        m_devices[device_id::ePortAPin5] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpioA, gpio::Pin::eBit6);
        m_devices[device_id::ePortAPin6] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpioA, gpio::Pin::eBit13);
        m_devices[device_id::ePortAPin7] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpioA, gpio::Pin::eBit26);

        constexpr std::uint32_t cPinSet0Mask = 0x2070;
        constexpr std::uint32_t cPinSet1Mask = 0xc420000;
        m_devices[device_id::ePortAPinSet0] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint32_t>>(gpioA, cPinSet0Mask, portAPinSet0Modifier);
        m_devices[device_id::ePortAPinSet1] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint32_t>>(gpioA, cPinSet1Mask, portAPinSet1Modifier);
    };

    auto initPortB = [&] {
        m_devices[device_id::ePortBPin0] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpioA, gpio::Pin::eBit18);
        m_devices[device_id::ePortBPin1] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpioA, gpio::Pin::eBit23);
        m_devices[device_id::ePortBPin2] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpioA, gpio::Pin::eBit24);
        m_devices[device_id::ePortBPin3] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpioA, gpio::Pin::eBit25);
        m_devices[device_id::ePortBPin4] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpioA, gpio::Pin::eBit12);
        m_devices[device_id::ePortBPin5] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpioA, gpio::Pin::eBit16);
        m_devices[device_id::ePortBPin6] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpioA, gpio::Pin::eBit20);
        m_devices[device_id::ePortBPin7] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpioA, gpio::Pin::eBit21);

        constexpr std::uint32_t cPinSet0Mask = 0x151000;
        constexpr std::uint32_t cPinSet1Mask = 0x3a00000;
        m_devices[device_id::ePortBPinSet0] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint32_t>>(gpioA, cPinSet0Mask, portBPinSet0Modifier);
        m_devices[device_id::ePortBPinSet1] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint32_t>>(gpioA, cPinSet1Mask, portBPinSet1Modifier);
    };
    // clang-format on

    initPortA();
    initPortB();

    return Error::eOk;
}

} // namespace hal
