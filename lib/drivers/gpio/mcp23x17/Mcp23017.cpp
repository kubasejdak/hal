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

#include "hal/gpio/Mcp23017.hpp"

#include "hal/Error.hpp"
#include "hal/i2c/ScopedI2c.hpp"
#include "hal/utils/logger.hpp"

#include <cassert>
#include <utility>

namespace hal::gpio {

Mcp23017::Mcp23017(std::shared_ptr<i2c::II2c> i2c,
                   std::uint16_t address,
                   mcp23x17::Port port,
                   std::chrono::milliseconds busTimeout,
                   std::chrono::milliseconds timeoutIn,
                   std::chrono::milliseconds timeoutOut)
    : IMcp23x17(port, busTimeout, timeoutIn, timeoutOut)
    , m_i2c(std::move(i2c))
    , m_cAddress(address)
{
    if (!i2c::verifyAddress(i2c::AddressingMode::e7bit, m_cAddress)) {
        Mcp23017Logger::critical("Failed to create MCP23017 GPIO: bad parameters");
        assert(false);
        return;
    }

    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        Mcp23017Logger::critical("Failed to create MCP23017 GPIO: timeout when locking I2C bus (timeout=default)");
        assert(false);
        return;
    }

    m_i2c->open();
    IMcp23x17::setInitialized();

    Mcp23017Logger::info("Created MCP23017 (I2C) GPIO expander with the following parameters:");
    Mcp23017Logger::info("  address      : {:#04x}", m_cAddress);
    Mcp23017Logger::info("  port         : {}", port == mcp23x17::Port::eGpioA ? "A" : "B");
    Mcp23017Logger::info("  busTimeoutMs : {} ms", busTimeout.count());
    Mcp23017Logger::info("  timeoutIn    : {} ms", timeoutIn.count());
    Mcp23017Logger::info("  timeoutOut   : {} ms", timeoutOut.count());
}

Mcp23017::~Mcp23017()
{
    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        Mcp23017Logger::error("Failed to close I2C device: timeout when locking I2C bus (timeout=default)");
        return;
    }

    m_i2c->close();
}

std::error_code Mcp23017::readCommand(std::uint8_t address, std::uint8_t& response, osal::Timeout timeout)
{
    return i2cRead(address, response, timeout);
}

std::error_code Mcp23017::writeCommand(std::uint8_t address, std::uint8_t value, osal::Timeout timeout)
{
    return i2cWrite(address, value, timeout);
}

std::error_code Mcp23017::i2cRead(std::uint8_t address, std::uint8_t& response, osal::Timeout timeout)
{
    i2c::ScopedI2c lock(m_i2c, timeout);
    if (!lock.isAcquired()) {
        Mcp23017Logger::error("Failed to acquire the I2C lock");
        return Error::eTimeout;
    }

    auto error = m_i2c->write(m_cAddress, {address}, true, timeout);
    if (!error) {
        std::size_t actualReadSize{};
        error = m_i2c->read(m_cAddress, &response, 1, timeout, actualReadSize);
    }

    return error;
}

std::error_code Mcp23017::i2cWrite(std::uint8_t address, std::uint8_t value, osal::Timeout timeout)
{
    i2c::ScopedI2c lock(m_i2c, timeout);
    if (!lock.isAcquired()) {
        Mcp23017Logger::error("Failed to acquire the I2C lock");
        return Error::eTimeout;
    }

    return m_i2c->write(m_cAddress, {address, value}, true, timeout);
}

} // namespace hal::gpio
