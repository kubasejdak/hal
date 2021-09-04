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

#include "hal/gpio/IGpioRegister.hpp"
#include "hal/gpio/Mcp23x17Common.hpp"

#include <osal/Timeout.hpp>

#include <chrono>
#include <cstdint>
#include <system_error>

namespace hal::gpio {

/// Default direction mask of the MCP23x17 port (input).
constexpr std::uint8_t cDefaultInputDirection = 0xff;

/// Represents common part of the MCP23017/MCP23017 driver with the IGpioRegister interface.
class IMcp23x17 : public IGpioRegister<std::uint8_t> {
public:
    /// Constructor.
    /// @param port             Id of the 8-bit port in the device.
    /// @param busTimeout       Maximal time to wait for the bus except for read/write operations.
    /// @param timeoutIn        Time after which cached input value should expire and trigger transmission.
    /// @param timeoutOut       Time after which cached output value should expire and trigger transmission.
    IMcp23x17(mcp23x17::Port port,
              std::chrono::milliseconds busTimeout,
              std::chrono::milliseconds timeoutIn,
              std::chrono::milliseconds timeoutOut);

    /// Returns flag indicating if diver is initialized.
    /// @return Flag indicating if diver is initialized.
    /// @retval true            Driver is initialized.
    /// @retval false           Driver is not initialized.
    [[nodiscard]] bool isInitialized() const { return m_initialized; }

    /// Marks driver instance as initialized.
    void setInitialized() { m_initialized = true; }

    /// Performs read command from the MCP23x17 register over underlying bus.
    /// @param address          Address of the register.
    /// @param response         Output parameter with the response from the device.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    virtual std::error_code readCommand(std::uint8_t address, std::uint8_t& response, osal::Timeout timeout) = 0;

    /// Performs write command to the MCP23x17 register over underlying bus.
    /// @param address          Address of the register.
    /// @param value            Value to be stored in the register.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    virtual std::error_code writeCommand(std::uint8_t address, std::uint8_t value, osal::Timeout timeout) = 0;

private:
    /// @see IGpioRegister::setDirection().
    std::error_code setDirection(std::uint8_t mask) override;

    /// @see IGpioRegister::get().
    std::error_code get(std::uint8_t& value) override;

    /// @see IGpioRegister::set().
    std::error_code set(std::uint8_t value) override;

private:
    const mcp23x17::Port m_cPort;
    const std::chrono::milliseconds m_cBusTimeout;
    bool m_initialized{};
    bool m_directionInitialized{};
    std::uint8_t m_direction{cDefaultInputDirection};
    std::uint8_t m_cacheIn{};
    std::uint8_t m_cacheOut{};
    osal::Timeout m_timeoutIn;
    osal::Timeout m_timeoutOut;
};

} // namespace hal::gpio
