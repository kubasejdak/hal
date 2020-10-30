/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Grzegorz Heldt
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

#pragma once

#include "hal/Error.hpp"

#include <system_error>

namespace hal::gpio {

/// Represents the GPIO register device, which doesn't support setting individual pins (all must be re-set each time).
/// Depending on the implementation it can represent for example GPIO port of the CPU or I2C/SPI GPIO expander.
/// @tparam WidthType       Type representing the bit-width of the port (e.g. std::uint32_t means that port is 32-bit).
/// @note If register is read/write, driver to provide get(), set(), and setDirection() methods.
///       If register is read only, driver has to provide get() method.
///       If register is write only, driver has to provide set() method.
template <typename WidthType>
class IGpioRegister {
public:
    /// Default constructor.
    IGpioRegister() = default;

    /// Copy constructor.
    /// @note This constructor is deleted, because IGpioRegister is not meant to be copy-constructed.
    IGpioRegister(const IGpioRegister&) = delete;

    /// Move constructor.
    IGpioRegister(IGpioRegister&&) noexcept = default;

    /// Virtual destructor.
    virtual ~IGpioRegister() = default;

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because IGpioRegister is not meant to be copy-assigned.
    IGpioRegister& operator=(const IGpioRegister&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because IGpioRegister is not meant to be move-assigned.
    IGpioRegister& operator=(IGpioRegister&&) = delete;

    /// Sets direction mask for each register's pin.
    /// @param mask          Direction mask for each pin in the register (1 for input, 0 for output).
    /// @return Error code of the operation.
    virtual std::error_code setDirection(WidthType /*unused*/) { return Error::eOk; }

    /// Reads whole value from the GPIO register.
    /// @param value        Output parameter with value read from the register.
    /// @return Error code of the operation.
    virtual std::error_code get(WidthType& /*unused*/) { return Error::eOk; }

    /// Writes whole value to the GPIO register.
    /// @param value        Value to be set in the register.
    /// @return Error code of the operation.
    virtual std::error_code set(WidthType /*unused*/) { return Error::eOk; }
};

} // namespace hal::gpio
