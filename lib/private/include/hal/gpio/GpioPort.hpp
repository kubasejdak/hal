/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Grzegorz Heldt
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2020-2021, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include "hal/gpio/IGpioPort.hpp"
#include "hal/gpio/IGpioRegister.hpp"
#include "hal/gpio/types.hpp"

#include <osal/Mutex.hpp>
#include <osal/ScopedLock.hpp>

#include <memory>
#include <system_error>
#include <utility>

namespace hal::gpio {

/// Represents the GPIO port with the defined width and access type.
/// @tparam WidthType       Type representing the bit-width of the port (e.g. std::uint32_t means that port is 32-bit).
template <typename WidthType>
class GpioPort : public IGpioPort<WidthType> {
    static_assert(cIsValidWidthType<WidthType>, "GpioPort can be parametrized only with unsigned arithmetic types");

public:
    /// Constructor.
    /// @param portRegister         Underlying GPIO register, which should be managed by this port instance.
    /// @param direction            Initial GPIO port direction mask.
    /// @param value                Initial GPIO port value.
    explicit GpioPort(std::shared_ptr<IGpioRegister<WidthType>> portRegister,
                      WidthType direction = WidthType{0},
                      WidthType value = WidthType{0})
        : m_register(std::move(portRegister))
        , m_direction(direction)
        , m_value(value)
    {
        m_register->set(m_value);
        m_register->setDirection(m_direction);
    }

    /// @see IGpioPort::get().
    std::error_code get(WidthType& data, WidthType mask) override
    {
        osal::ScopedLock lock(m_mutex);
        m_direction |= mask;

        if (auto error = m_register->setDirection(m_direction))
            return error;

        WidthType tmp;
        auto error = m_register->get(tmp);
        if (!error)
            data = tmp & mask;

        return error;
    }

    /// @see IGpioPort::set().
    std::error_code set(WidthType value, WidthType mask) override
    {
        osal::ScopedLock lock(m_mutex);
        WidthType toSet = mask & value;
        WidthType toPreserve = m_value & WidthType(~mask);
        WidthType newValue = toSet | toPreserve;
        WidthType newDirection = m_direction & WidthType(~mask);

        auto error = m_register->set(newValue);
        if (!error) {
            error = m_register->setDirection(newDirection);
            if (!error) {
                m_direction = newDirection;
                m_value = newValue;
            }
        }

        return error;
    }

private:
    osal::Mutex m_mutex;
    std::shared_ptr<IGpioRegister<WidthType>> m_register;
    WidthType m_direction;
    WidthType m_value;
};

} // namespace hal::gpio
