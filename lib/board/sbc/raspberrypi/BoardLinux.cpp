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
#include "hal/Board.hpp"
#include "hal/Error.hpp"
#include "sbc/raspberrypi/DeviceId.hpp"

namespace hal {
namespace detail {

template <>
std::shared_ptr<Device> getDeviceImpl<device_id::RaspberryPiId>(device_id::RaspberryPiId id)
{
    return Board<device_id::RaspberryPiId>::instance().getDevice(id);
}

} // namespace detail

template <>
std::error_code Board<device_id::RaspberryPiId>::initImpl()
{
    // clang-format off
    auto cGpio0Lines      = {4, 5, 6, 12, 13, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 27}; // NOLINT
    auto cGpio0Directions = {0, 0, 0,  1,  0,  1,  0,  1,  1,  1,  0,  1,  1,  1,  0,  0};
    gpio::Registry<std::uint32_t>::init({{"gpio0", gpio::LinuxGpio("gpio0", "pinctrl-bcm2835", cGpio0Lines, cGpio0Directions)}});

    // clang-format on

    return Error::eOk;
}

template <>
std::error_code Board<device_id::RaspberryPiId>::deinitImpl()
{
    gpio::Registry<std::uint32_t>::clear();

    return Error::eOk;
}

} // namespace hal