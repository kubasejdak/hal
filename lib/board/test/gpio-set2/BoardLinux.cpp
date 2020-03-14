/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2020-2020, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include "LinuxGpio.hpp"
#include "gpio-set2/DeviceId.hpp"
#include "hal/Board.hpp"
#include "hal/Error.hpp"
#include "hal/gpio/PortInput.hpp"
#include "hal/gpio/PortOutput.hpp"

#include <bitset>
#include <climits>

namespace hal {
namespace detail {

template <>
std::shared_ptr<Device> getDeviceImpl<device_id::GpioSet2Id>(device_id::GpioSet2Id id)
{
    return Board<device_id::GpioSet2Id>::instance().getDevice(id);
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
std::error_code Board<device_id::GpioSet2Id>::initImpl()
{
    // clang-format off
    auto cGpio0Lines      = {4, 5, 6, 12, 13, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 27}; // NOLINT
    auto cGpio0Directions = {0, 0, 0,  1,  0,  1,  0,  1,  1,  1,  0,  1,  1,  1,  0,  0};
    gpio::LinuxGpioDriver::init(gpio::LinuxGpio("gpio0", "pinctrl-bcm2835", cGpio0Lines, cGpio0Directions));

    auto gpio0 = gpio::LinuxGpioDriver::get("gpio0");

    constexpr std::uint32_t cPortAPinSet0Mask = 0x2070;
    constexpr std::uint32_t cPortAPinSet1Mask = 0xc420000;
    m_devices[device_id::ePortAPinSet0] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint32_t>>(gpio0, cPortAPinSet0Mask, portAPinSet0Modifier);
    m_devices[device_id::ePortAPinSet1] = std::make_shared<gpio::PortOutput<std::uint8_t, std::uint32_t>>(gpio0, cPortAPinSet1Mask, portAPinSet1Modifier);

    constexpr std::uint32_t cPortBPinSetLowMask = 0x151000;
    constexpr std::uint32_t cPortBPinSetHighMask = 0x3a00000;
    m_devices[device_id::ePortBPinSet0] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint32_t>>(gpio0, cPortBPinSetLowMask, portBPinSet0Modifier);
    m_devices[device_id::ePortBPinSet1] = std::make_shared<gpio::PortInput<std::uint8_t, std::uint32_t>>(gpio0, cPortBPinSetHighMask, portBPinSet1Modifier);
    // clang-format on

    return Error::eOk;
}

} // namespace hal
