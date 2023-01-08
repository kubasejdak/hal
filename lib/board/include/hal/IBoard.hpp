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

#pragma once

#include "hal/Device.hpp"
#include "hal/Error.hpp"

#include <memory>
#include <system_error>

namespace hal {

/// Interface for objects objects representing physical hardware boards.
/// @note Main goal of the board interface is to be the container for device drivers and provide a mechanism to
///       access them via special enum identifier.
class IBoard {
public:
    /// Default constructor.
    IBoard() = default;

    /// Copy constructor.
    /// @note This constructor is deleted, because IBoard is not meant to be copy-constructed.
    IBoard(const IBoard&) = delete;

    /// Move constructor.
    /// @note This constructor is deleted, because IBoard is not meant to be move-constructed.
    IBoard(IBoard&&) noexcept = delete;

    /// Virtual destructor.
    virtual ~IBoard() = default;

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because IBoard is not meant to be copy-assigned.
    IBoard& operator=(const IBoard&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because IBoard is not meant to be move-assigned.
    IBoard& operator=(IBoard&&) noexcept = delete;

    /// Initializes the board.
    /// @return Error code of the operation.
    /// @note This function will usually create and initialize all device drivers that will be available to access
    ///       from this board.
    virtual std::error_code init() = 0;

    /// Deinitializes the board.
    /// @return Error code of the operation.
    virtual std::error_code deinit() = 0;

    /// Returns device handle associated with the given device id from the current board.
    /// @tparam IdType          Type of the device identifier (usually related to the concrete board implementation).
    /// @param id               Identifier of the device to be returned.
    /// @return Generic device handle associated with the given device id.
    /// @note Before returning handle internal device reference counter is incremented. If maximal references has
    ///       been reached, then nullptr will be returned.
    template <typename IdType>
    std::shared_ptr<Device> getDevice(IdType id)
    {
        if (auto device = getDeviceImpl(id)) {
            if (device->take())
                return nullptr;

            return device;
        }

        return nullptr;
    }

    /// Returns given device handle to the board.
    /// @param device           Device to be returned.
    /// @return Error code of the operation.
    /// @note After returning internal device reference counter is decremented, making it possible to get this device
    ///       by other clients.
    std::error_code returnDevice(std::shared_ptr<Device>& device)
    {
        if (auto error = returnDeviceImpl(device))
            return error;

        return device->give();
    }

    /// Returns board containing given device.
    /// @param device           Device for which containing board should be returned.
    /// @return Board containing given device.
    static IBoard* getBoard(const std::shared_ptr<Device>& device) { return device->board(); }

protected:
    /// Sets reference to the current board in the given device.
    /// @param device           Device for which containing board should be set.
    void setBoard(std::shared_ptr<Device>& device) { device->setBoard(this); }

private:
    /// Board-specific implementation of the method returning device handle associated with the given device id.
    /// @param id               Identifier of the device to be returned.
    /// @return Generic device handle associated with the given device id.
    virtual std::shared_ptr<Device> getDeviceImpl(int id) = 0;

    /// Board-specific implementation of the method returning given device handle to the board.
    /// @param device           Device to be returned.
    /// @return Error code of the operation.
    virtual std::error_code returnDeviceImpl(std::shared_ptr<Device>& device) = 0;
};

} // namespace hal
