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

#include "hal/Board.hpp"
#include "hal/Error.hpp"
#include "hal/test/TestDevice.hpp"
#include "middleware/DeviceId.hpp"

namespace hal {
namespace detail {

template <>
std::shared_ptr<Device> getDeviceImpl<device_id::MiddlewareId>(device_id::MiddlewareId id)
{
    return CurrentBoardVersion<device_id::MiddlewareId>::get()->getDevice(id);
}

} // namespace detail

template <>
std::error_code Board<device_id::MiddlewareId>::initImpl()
{
    m_devices[device_id::eTestDeviceSingle1] = std::make_shared<test::TestDevice>(SharingPolicy::eSingle);
    m_devices[device_id::eTestDeviceSingle2] = std::make_shared<test::TestDevice>(SharingPolicy::eSingle);
    m_devices[device_id::eTestDeviceShared1] = std::make_shared<test::TestDevice>(SharingPolicy::eShared);
    m_devices[device_id::eTestDeviceShared2] = std::make_shared<test::TestDevice>(SharingPolicy::eShared);
    return Error::eOk;
}

} // namespace hal
