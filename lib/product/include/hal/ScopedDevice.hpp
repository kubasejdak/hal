/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2021-2022, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include <algorithm>
#include <memory>
#include <type_traits>

namespace hal {

class Device;

/// RAII wrapper for the Device handle returned from HAL, which automatically returns wrapped device to HAL upon
/// destruction. It has special conversion operator to std::shared_ptr<T> to allow easy passing to other functions.
/// @tparam T               Type of the wrapped device handle.
/// @note The semantics of this type is close to std::unique_ptr. It is a move-only type.It is assumed, that user will
///       never hold another copy of given handle in other places.
template <typename T>
class ScopedDevice {
    static_assert(std::is_base_of_v<Device, T>, "ScopedDevice can hold only types derived from hal::Device");

public:
    /// Default constructor.
    ScopedDevice() = default;

    /// Constructor. Takes ownership of the given device handle in terms of its lifetime in user code.
    /// @param device       Device handle to be managed by this wrapper.
    ScopedDevice(std::shared_ptr<T> device) // NOLINT
        : m_device(std::move(device))
    {}

    /// Copy constructor.
    /// @note This constructor is deleted, because ScopedDevice is not meant to be copy-constructed.
    ScopedDevice(const ScopedDevice&) = delete;

    /// Move constructor
    ScopedDevice(ScopedDevice&& other) noexcept = default;

    /// Destructor. Automatically returns wrapped device handle back to HAL.
    ~ScopedDevice() { reset(); }

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because ScopedDevice is not meant to be copy-assigned.
    ScopedDevice<T>& operator=(const ScopedDevice&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    ScopedDevice<T>& operator=(ScopedDevice&&) noexcept = default;

    /// Conversion operator to bool.
    /// @return Flag indicating if the underlying pointer is nullptr or not.
    operator bool() const { return static_cast<bool>(m_device); } // NOLINT

    /// Dereferences the underlying object.
    /// @return Copy of the underlying object in a form of std::shared_ptr<T>.
    auto& operator->() const { return m_device; }

    /// Returns wrapped device handle back to HAL and clears the underlying object.
    void reset()
    {
        if (m_device) {
            returnDevice(m_device);
            m_device.reset();
        }
    }

    /// Returns reference to the underlying object.
    /// @return Reference to the underlying object.
    [[nodiscard]] auto& get() const { return m_device; }

private:
    std::shared_ptr<T> m_device;
};

} // namespace hal
