/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Grzegorz Heldt
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

#include "hal/Error.hpp"
#include "hal/gpio/types.hpp"

#include <system_error>

namespace hal::gpio {

/// @class IGpioPort
/// Represents the GPIO port with the defined width and access type.
/// @tparam WidthType       Type representing the bit-width of the port (e.g. std::uint32_t means that port is 32-bit).
template <typename WidthType>
class IGpioPort {
    static_assert(cIsValidWidthType<WidthType>);

public:
    /// Default constructor.
    IGpioPort() = default;

    /// Copy constructor.
    /// @note This constructor is deleted, because IGpioPort is not meant for copying.
    IGpioPort(const IGpioPort&) = delete;

    /// Move constructor.
    IGpioPort(IGpioPort&&) noexcept = default;

    /// Virtual destructor.
    virtual ~IGpioPort() = default;

    /// Copy assignment operator.
    /// @note This operator is deleted, because IGpioPort is not meant for copying.
    IGpioPort& operator=(const IGpioPort&) = delete;

    /// Move assignment operator.
    IGpioPort& operator=(IGpioPort&&) noexcept = default;

    /// Sets the direction of each pin in the GPIO port.
    /// @param direction    Direction mask to be set.
    /// @param mask         Mask indicating which pins should be affected.
    /// @return Error code of the operation.
    std::error_code setDirection(WidthType direction, WidthType mask) { return drvSetDirection(direction, mask); }

    /// Reads the demanded set of GPIO port bits defined by the mask.
    /// @param value        Output argument where the read value will be stored.
    /// @param mask         Mask defining which port bits should be read.
    /// @return Error code of the operation.
    std::error_code read(WidthType& value, WidthType mask) { return drvRead(value, mask); }

    /// Writes the demanded set of GPIO port bits defined by the mask.
    /// @param value        Value to be written to the GPIO port.
    /// @param mask         Mask defining which port bits should be written.
    /// @return Error code of the operation.
    std::error_code write(WidthType value, WidthType mask) { return drvWrite(value, mask); }

private:
    /// Driver specific implementation of setting the GPIO port direction.
    /// @return Error code of the operation.
    virtual std::error_code drvSetDirection(WidthType, WidthType) = 0;

    /// Driver specific implementation of GPIO port reading.
    /// @param value        Output argument where the read value will be stored.
    /// @param mask         Mask defining which port bits should be read.
    /// @return Error code of the operation.
    virtual std::error_code drvRead(WidthType& value, WidthType mask) = 0;

    /// Driver specific implementation of GPIO port writing.
    /// @param value        Value to be written to the GPIO port.
    /// @param mask         Mask defining which port bits should be written.
    /// @return Error code of the operation.
    virtual std::error_code drvWrite(WidthType value, WidthType mask) = 0;
};

} // namespace hal::gpio
