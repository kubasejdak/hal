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
#include "hal/gpio/IGpioPort.hpp"
#include "hal/gpio/IPinInput.hpp"
#include "hal/gpio/types.hpp"

#include <cassert>
#include <memory>
#include <type_traits>

namespace hal::gpio {

/// @class PinInput
/// Represents a single pin input device.
/// This class isolates a single bit input from the whole digital port.
/// @tparam WidthType           Type representing the bit-width of the port.
/// @tparam access              Demanded GPIO access type.
template <typename WidthType, Access access>
class PinInput : public IPinInput {
    static_assert(cIsValidWidthType<WidthType>, "PinInput can be parametrized only with unsigned arithmetic types");
    static_assert(std::negation<detail::IsWriteOnly<access>>::value, "Cannot use PinInput with eWriteOnly port");

public:
    /// Constructor.
    /// @param port             Underlying GPIO port, that contains the given pin.
    /// @param pin              Pin id of the GPIO port used by this bit input instance.
    /// @param negated          Flag indicating if all operations on this pin input instance should be inverted.
    /// @param sharingPolicy    Flag indicating sharing policy of this pin input instance.
    PinInput(std::shared_ptr<IGpioPort<WidthType, access>> port,
             Pin pin,
             bool negated = false,
             SharingPolicy sharingPolicy = SharingPolicy::eSingle)
        : IPinInput(sharingPolicy)
        , m_port(port)
        , m_mask(WidthType{1} << static_cast<WidthType>(pin))
        , m_negated(negated)
    {
        assert(pin <= maxPin<WidthType>() && "Requested pin exceeds the width of the underlying port");

        m_port->initPin(pin);
        m_port->setDirection(~WidthType{0}, m_mask);
    }

private:
    /// Gets the boolean value of the current pin input.
    /// @param value            Output argument where the read value will be saved.
    /// @return Error code of the operation.
    std::error_code get(bool& value) override
    {
        WidthType data;
        auto result = m_port->read(&data, m_mask);

        value = (data == 0 == m_negated);
        return result;
    }

private:
    std::shared_ptr<IGpioPort<WidthType, access>> m_port;
    WidthType m_mask;
    bool m_negated;
};

} // namespace hal::gpio
