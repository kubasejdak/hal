/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Grzegorz Heldt
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2021, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include <cstdint>
#include <memory>
#include <system_error>

namespace hal {

// clang-format off
/// Represents possible sharing policies of the HAL devices. Single devices can be only returned once from HAL,
/// while shared devices can be returned multiple times and are internally synchronized if needed.
enum class SharingPolicy {
    eSingle,
    eShared
};
// clang-format on

class IBoard;

/// Base class for all device drivers provided by HAL. Provides basic mechanics to support SharingPolicy.
class Device {
    friend class IBoard;

public:
    /// Constructor.
    /// @param sharingPolicy         SharingPolicy to be set for the current device.
    explicit Device(SharingPolicy sharingPolicy);

    /// Copy constructor.
    /// @note This constructor is deleted, because Device is not meant to be copy-constructed.
    Device(const Device&) = delete;

    /// Move constructor.
    /// @note This constructor is deleted, because Device is not meant to be move-constructed.
    Device(Device&&) noexcept = default;

    /// Virtual destructor.
    virtual ~Device() = default;

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because Device is not meant to be copy-assigned.
    Device& operator=(const Device&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because Device is not meant to be move-assigned.
    Device& operator=(Device&&) = delete;

    /// Returns current number of clients outside HAL (how many times this driver has been returned from HAL).
    /// @return Current number of clients outside HAL.
    [[nodiscard]] std::size_t ownersCount() const { return m_ownersCount; }

private:
    /// Checks if device has reached its owners limit according to its SharingPolicy. If not, then the internal
    /// owners counter is incremented. If limit is reached, then appropriate error is returned.
    /// @return Error code of the operation.
    std::error_code take();

    /// Decrements the internal owners counter.
    /// @return Error code of the operation.
    std::error_code give();

    /// Sets the given board in the current device.
    /// @param board                Board to be set in the current device.
    /// @note Given board should be the one that is currently holding this device.
    void setBoard(IBoard* board) { m_board = board; }

    /// Returns board which is holding this device.
    /// @return Board which is holding this device.
    [[nodiscard]] IBoard* board() const { return m_board; }

private:
    SharingPolicy m_sharingPolicy;
    IBoard* m_board{};
    std::size_t m_ownersCount{};
};

namespace detail {

/// Internal board implementation of the method returning device handle associated with the given device id.
/// @tparam IdType                  Type of the device to be returned.
/// @param id                       Identifier of the device to be returned.
/// @return Device handle associated with the given device id.
template <typename IdType>
std::shared_ptr<Device> getDeviceImpl(IdType id);

} // namespace detail

/// Return given device to HAL.
/// @param device                   Device to be returned.
/// @return Error code of the operation.
/// @note After this call device handle is cleared.
std::error_code returnDevice(std::shared_ptr<Device>&& device);

} // namespace hal
