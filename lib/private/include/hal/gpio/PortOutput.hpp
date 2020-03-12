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
#include "hal/gpio/IGpioPort.hpp"
#include "hal/gpio/IPortOutput.hpp"

#include <functional>
#include <memory>

namespace hal::gpio {

template <typename WidthType, typename WidthTypeUnderlying>
class PortOutput : public IPortOutput<WidthType> {
    static_assert(cIsValidWidthType<WidthType>);
    static_assert(cIsValidWidthType<WidthTypeUnderlying>);

public:
    using ModifierCallback = std::function<WidthTypeUnderlying(WidthType, WidthTypeUnderlying)>;

    PortOutput(std::shared_ptr<IGpioPort<WidthTypeUnderlying>> port,
               WidthTypeUnderlying mask,
               ModifierCallback modifier = nullptr,
               SharingPolicy sharingPolicy = SharingPolicy::eSingle)
        : IPortOutput<WidthType>(sharingPolicy)
        , m_port(port)
        , m_mask(~mask)
        , m_modifier(modifier)
    {}

    /// @see IPortOutput::write
    std::error_code write(WidthType value) override
    {
        WidthTypeUnderlying modifiedValue = m_modifier ? m_modifier(value, m_mask) : value;
        return m_port->write(modifiedValue, m_mask);
    }

private:
    std::shared_ptr<IGpioPort<WidthTypeUnderlying>> m_port;
    WidthTypeUnderlying m_mask;
    ModifierCallback m_modifier;
};

} // namespace hal::gpio
