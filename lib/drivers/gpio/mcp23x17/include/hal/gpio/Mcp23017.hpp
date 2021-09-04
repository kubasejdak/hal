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

#pragma once

#include "hal/gpio/IMcp23x17.hpp"
#include "hal/i2c/II2c.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <system_error>

namespace hal::gpio {

/// Represents 8-bit port of the MCP23017 driver with the IGpioRegister interface.
class Mcp23017 : public IMcp23x17 {
public:
    /// Constructor.
    /// @param i2c              Reference to the I2C bus that should be used.
    /// @param address          Address of the device on the I2C bus.
    /// @param port             Id of the 8-bit port in the device.
    /// @param busTimeout       Maximal time to wait for the bus except for read/write operations.
    /// @param timeoutIn        Time after which cached input value should expire and trigger transmission.
    /// @param timeoutOut       Time after which cached output value should expire and trigger transmission.
    Mcp23017(std::shared_ptr<i2c::II2c> i2c,
             std::uint16_t address,
             mcp23x17::Port port,
             std::chrono::milliseconds busTimeout,
             std::chrono::milliseconds timeoutIn = 5ms,
             std::chrono::milliseconds timeoutOut = 5ms);

    /// Copy constructor.
    /// @note This constructor is deleted, because Mcp23017 is not meant to be copy-constructed.
    Mcp23017(const Mcp23017&) = delete;

    /// Copy constructor.
    /// @note This constructor is deleted, because Mcp23017 is not meant to be move-constructed.
    Mcp23017(Mcp23017&&) noexcept = delete;

    /// Destructor.
    ~Mcp23017() override;

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because Mcp23017 is not meant to be copy-assigned.
    Mcp23017& operator=(const Mcp23017&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because Mcp23017 is not meant to be move-assigned.
    Mcp23017& operator=(Mcp23017&&) noexcept = delete;

private:
    /// @see IMcp23x17::readCommand().
    std::error_code readCommand(std::uint8_t address, std::uint8_t& response, osal::Timeout timeout) override;

    /// @see IMcp23x17::writeCommand().
    std::error_code writeCommand(std::uint8_t address, std::uint8_t value, osal::Timeout timeout) override;

    /// Performs read command from the MCP23017 register over I2C bus.
    /// @param address          Address of the register.
    /// @param response         Output parameter with the response from the device.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code i2cRead(std::uint8_t address, std::uint8_t& response, osal::Timeout timeout);

    /// Performs write command to the MCP23017 register over I2C bus.
    /// @param address          Address of the register.
    /// @param value            Value to be stored in the register.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code i2cWrite(std::uint8_t address, std::uint8_t value, osal::Timeout timeout);

private:
    std::shared_ptr<i2c::II2c> m_i2c;
    const std::uint16_t m_cAddress;
};

} // namespace hal::gpio
