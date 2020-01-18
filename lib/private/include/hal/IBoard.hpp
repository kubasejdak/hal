/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include <memory>
#include <system_error>

namespace hal {

class IBoard {
public:
    virtual std::error_code init() = 0;
    virtual std::error_code deinit() = 0;

    template <typename IdType>
    std::shared_ptr<Device> getDevice(IdType id)
    {
        if (auto device = getDeviceImpl(id)) {
            if (auto error = device->take())
                return nullptr;

            return device;
        }

        return nullptr;
    }

    std::error_code returnDevice(std::shared_ptr<Device>& device)
    {
        if (auto error = returnDeviceImpl(device))
            return error;

        return device->give();
    }

    static IBoard* getBoard(const std::shared_ptr<Device>& device) { return device->board(); }

protected:
    void setBoard(std::shared_ptr<Device>& device) { device->setBoard(this); }

private:
    virtual std::shared_ptr<Device> getDeviceImpl(int id) = 0;
    virtual std::error_code returnDeviceImpl(std::shared_ptr<Device>& device) = 0;
};

} // namespace hal
