/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
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

#include "IMcp23x17.hpp"

#include "hal/Error.hpp"
#include "hal/utils/logger.hpp"

namespace hal::gpio {

IMcp23x17::IMcp23x17(mcp23x17::Port port,
                     std::chrono::milliseconds busTimeout,
                     std::chrono::milliseconds timeoutIn,
                     std::chrono::milliseconds timeoutOut)
    : m_cPort(port)
    , m_cBusTimeout(busTimeout)
    , m_timeoutIn(timeoutIn, true)
    , m_timeoutOut(timeoutOut, true)
{}

std::error_code IMcp23x17::setDirection(std::uint8_t mask)
{
    if (!isInitialized()) {
        Mcp23x17Logger::error("Driver is not initialized");
        return Error::eDeviceNotOpened;
    }

    if (m_direction != mask || !m_directionInitialized) {
        auto regAddr = (m_cPort == mcp23x17::Port::eGpioA) ? mcp23x17::Bank0::eIODIRA : mcp23x17::Bank0::eIODIRB;
        Mcp23x17Logger::trace("Setting direction {:#04x} to register {:#04x}", mask, regAddr);
        if (auto error = writeCommand(regAddr, mask, m_cBusTimeout)) {
            Mcp23x17Logger::error("Failed to set direction: err={}", error.message());
            return error;
        }

        m_directionInitialized = true;
        m_direction = mask;
    }

    return Error::eOk;
}

std::error_code IMcp23x17::get(std::uint8_t& value)
{
    if (!isInitialized()) {
        Mcp23x17Logger::error("Driver is not initialized");
        return Error::eDeviceNotOpened;
    }

    bool cacheExpired = m_timeoutIn.isExpired();
    if (cacheExpired) {
        Mcp23x17Logger::trace("Read transfer required: cacheExpired={}", cacheExpired);

        auto regAddr = (m_cPort == mcp23x17::Port::eGpioA) ? mcp23x17::Bank0::eGPIOA : mcp23x17::Bank0::eGPIOB;
        if (auto error = readCommand(regAddr, m_cacheIn, m_cBusTimeout)) {
            Mcp23x17Logger::error("Failed to read value: err={}", error.message());
            return error;
        }

        Mcp23x17Logger::trace("Read port value {:#04x} from register {:#04x}", m_cacheIn, regAddr);

        m_timeoutIn.reset();
    }

    value = m_cacheIn;
    return Error::eOk;
}

std::error_code IMcp23x17::set(std::uint8_t value)
{
    if (!isInitialized()) {
        Mcp23x17Logger::error("Driver is not initialized");
        return Error::eDeviceNotOpened;
    }

    std::uint8_t invertedDirection = ~m_direction;
    bool cacheExpired = m_timeoutOut.isExpired();
    bool valueChanged = (m_cacheOut & invertedDirection) != (value & invertedDirection);
    if (cacheExpired || valueChanged) {
        Mcp23x17Logger::trace("Write transfer required: cacheExpired={}, valueChange={}", cacheExpired, valueChanged);

        auto regAddr = (m_cPort == mcp23x17::Port::eGpioA) ? mcp23x17::Bank0::eGPIOA : mcp23x17::Bank0::eGPIOB;
        Mcp23x17Logger::trace("Setting port value {:#04x} to register {:#04x}", value, regAddr);
        if (auto error = writeCommand(regAddr, value, m_cBusTimeout)) {
            Mcp23x17Logger::error("Failed to set value: err={}", error.message());
            return error;
        }

        m_cacheOut = value & invertedDirection;
        m_timeoutOut.reset();
    }

    return Error::eOk;
}

} // namespace hal::gpio
