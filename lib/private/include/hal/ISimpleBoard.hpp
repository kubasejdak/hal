/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2020, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#pragma once

#include "hal/Device.hpp"
#include "hal/Error.hpp"
#include "hal/IBoard.hpp"

#include <boost/range/adaptors.hpp>

#include <map>
#include <memory>
#include <system_error>

namespace hal {

template <typename IdType>
class ISimpleBoard : public IBoard {
private:
    std::error_code init() override
    {
        if (auto error = initImpl())
            return error;

        for (auto& device : m_devices | boost::adaptors::map_values)
            setBoard(device);

        return Error::eOk;
    }

    std::error_code deinit() override
    {
        for (const auto& device : m_devices | boost::adaptors::map_values) {
            if (device->ownersCount() != 0)
                return Error::eDeviceTaken;
        }

        m_devices.clear();
        return Error::eOk;
    }

    std::shared_ptr<Device> getDeviceImpl(int id) override
    {
        auto deviceId = static_cast<IdType>(id);
        if (m_devices.find(deviceId) == std::end(m_devices))
            return nullptr;

        return m_devices[deviceId];
    }

    std::error_code returnDeviceImpl(std::shared_ptr<Device>&) override { return Error::eOk; }

    virtual std::error_code initImpl() = 0;

protected:
    std::map<IdType, std::shared_ptr<Device>> m_devices;
};

} // namespace hal
