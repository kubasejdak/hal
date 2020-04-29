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
#include "hal/IBoard.hpp"
#include "hal/gpio/IGpioPort.hpp"
#include "hal/gpio/IPinInput.hpp"
#include "hal/gpio/types.hpp"

#include <cassert>
#include <memory>
#include <utility>

namespace hal::gpio {

/// Represents a single pin input device.
/// This class isolates a single pin input from the whole GPIO port.
/// @tparam WidthType           Type representing the bit-width of the port.
template <typename WidthType>
class PinInput : public IPinInput {
    static_assert(cIsValidWidthType<WidthType>);

public:
    /// Constructor.
    /// @param port             Underlying GPIO port, that contains the given pin.
    /// @param pin              Pin id of the GPIO port used by this bit input instance.
    /// @param negated          Flag indicating if all operations on this input pin instance should be inverted.
    /// @param sharingPolicy    Flag indicating sharing policy of this input pin instance.
    PinInput(std::shared_ptr<IGpioPort<WidthType>> port,
             Pin pin,
             bool negated = false,
             SharingPolicy sharingPolicy = SharingPolicy::eSingle)
        : IPinInput(sharingPolicy)
        , m_port(std::move(port))
        , m_mask(WidthType{1} << static_cast<WidthType>(pin))
        , m_negated(negated)
    {
        assert(pin <= maxPin<WidthType>());

        m_port->setDirection(~WidthType{0}, m_mask);
    }

private:
    /// @see IPinInput::get().
    std::error_code get(bool& value) override
    {
        WidthType data{};
        if (auto error = m_port->read(data, m_mask))
            return error;

        value = ((data == 0) == m_negated);
        return Error::eOk;
    }

private:
    std::shared_ptr<IGpioPort<WidthType>> m_port;
    WidthType m_mask;
    bool m_negated;
};

} // namespace hal::gpio
