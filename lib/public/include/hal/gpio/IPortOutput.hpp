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

#include "hal/Device.hpp"
#include "hal/Error.hpp"
#include "hal/gpio/types.hpp"

#include <system_error>

namespace hal::gpio {

/// @class IPortOutput
/// Represents an abstraction for the set of the GPIO output pins, which are part of the same GPIO port.
/// This set can represent any combination of the pins and is defined by the board configuration.
/// @tparam WidthType               Unsigned type representing bitness of the given port (e.g. std::uint32_t is 32bit).
template <typename WidthType>
class IPortOutput : public Device {
    static_assert(cIsValidWidthType<WidthType>, "IPortOutput can be parametrized only with unsigned arithmetic types");

public:
    /// Constructor.
    /// @param sharingPolicy        Sharing policy of this device.
    explicit IPortOutput(SharingPolicy sharingPolicy)
        : Device(sharingPolicy)
    {}

    /// Writes given value to this GPIO pin set.
    /// @param value                Bit values of each pin from this port.
    /// @return Error code of the operation.
    virtual std::error_code write(WidthType value) = 0;
};

} // namespace hal::gpio
