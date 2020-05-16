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

#include "hal/Board.hpp"
#include "hal/Error.hpp"
#include "hal/gpio/IGpioPort.hpp"
#include "hal/gpio/PinInput.hpp"
#include "hal/gpio/PinOutput.hpp"
#include "test/gpio-set1/DeviceId.hpp"

namespace hal {
namespace detail {

template <>
std::shared_ptr<Device> getDeviceImpl<device_id::GpioSet1Id>(device_id::GpioSet1Id id)
{
    return Board<device_id::GpioSet1Id>::instance().getDevice(id);
}

} // namespace detail

template <>
std::error_code Board<device_id::GpioSet1Id>::initImpl()
{
    // clang-format off
    auto gpio0 = gpio::Registry<std::uint32_t>::get("gpio0");

    m_devices[device_id::ePortAPin0] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpio0, gpio::Pin::eBit4);
    m_devices[device_id::ePortAPin1] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpio0, gpio::Pin::eBit17);
    m_devices[device_id::ePortAPin2] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpio0, gpio::Pin::eBit27);
    m_devices[device_id::ePortAPin3] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpio0, gpio::Pin::eBit22);
    m_devices[device_id::ePortAPin4] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpio0, gpio::Pin::eBit5);
    m_devices[device_id::ePortAPin5] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpio0, gpio::Pin::eBit6);
    m_devices[device_id::ePortAPin6] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpio0, gpio::Pin::eBit13);
    m_devices[device_id::ePortAPin7] = std::make_shared<gpio::PinOutput<std::uint32_t>>(gpio0, gpio::Pin::eBit26);

    m_devices[device_id::ePortBPin0] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpio0, gpio::Pin::eBit18);
    m_devices[device_id::ePortBPin1] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpio0, gpio::Pin::eBit23);
    m_devices[device_id::ePortBPin2] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpio0, gpio::Pin::eBit24);
    m_devices[device_id::ePortBPin3] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpio0, gpio::Pin::eBit25);
    m_devices[device_id::ePortBPin4] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpio0, gpio::Pin::eBit12);
    m_devices[device_id::ePortBPin5] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpio0, gpio::Pin::eBit16);
    m_devices[device_id::ePortBPin6] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpio0, gpio::Pin::eBit20);
    m_devices[device_id::ePortBPin7] = std::make_shared<gpio::PinInput<std::uint32_t>>(gpio0, gpio::Pin::eBit21);
    // clang-format on

    return Error::eOk;
}

} // namespace hal
