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
#include "hal/gpio/IPinOutput.hpp"
#include "hal/spi/ISpi.hpp"

#include <osal/Timeout.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <system_error>

namespace hal::gpio {

/// Represents 8-bit port of the MCP23S17 driver with the IGpioRegister interface.
class Mcp23S17 : public IMcp23x17 {
public:
    /// Constructor.
    /// @param spi              Reference to the SPI bus that should be used.
    /// @param chipSelect       Reference to the chip select that should be used.
    /// @param address          Address of the device on the bus.
    /// @param port             Id of the 8-bit port in the device.
    /// @param busTimeout       Maximal time to wait for the bus except for read/write operations.
    /// @param timeoutIn        Time after which cached input value should expire and trigger transmission.
    /// @param timeoutOut       Time after which cached output value should expire and trigger transmission.
    Mcp23S17(std::shared_ptr<spi::ISpi> spi,
             std::shared_ptr<gpio::IPinOutput> chipSelect,
             std::uint8_t address,
             mcp23x17::Port port,
             std::chrono::milliseconds busTimeout,
             std::chrono::milliseconds timeoutIn = 5ms,
             std::chrono::milliseconds timeoutOut = 5ms);

    /// Copy constructor.
    /// @note This constructor is deleted, because Mcp23S17 is not meant to be copy-constructed.
    Mcp23S17(const Mcp23S17&) = delete;

    /// Copy constructor.
    /// @note This constructor is deleted, because Mcp23S17 is not meant to be move-constructed.
    Mcp23S17(Mcp23S17&& other) = delete;

    /// Destructor.
    ~Mcp23S17() override;

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because Mcp23S17 is not meant to be copy-assigned.
    Mcp23S17& operator=(const Mcp23S17&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because Mcp23S17 is not meant to be move-assigned.
    Mcp23S17& operator=(Mcp23S17&&) = delete;

private:
    /// @see IMcp23x17::readCommand().
    std::error_code readCommand(std::uint8_t address, std::uint8_t& response, osal::Timeout timeout) override;

    /// @see IMcp23x17::writeCommand().
    std::error_code writeCommand(std::uint8_t address, std::uint8_t value, osal::Timeout timeout) override;

    /// Performs read command from the MCP23S17 register over SPI bus.
    /// @param address          Address of the register.
    /// @param response         Output parameter with the response from the device.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code spiRead(std::uint8_t address, std::uint8_t& response, osal::Timeout timeout);

    /// Performs write command to the MCP23S17 register over SPI bus.
    /// @param address          Address of the register.
    /// @param value            Value to be stored in the register.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code spiWrite(std::uint8_t address, std::uint8_t value, osal::Timeout timeout);

private:
    const hal::spi::SpiParams m_cSpiParams{2000000, hal::spi::Mode::eMode0, 8};

    std::shared_ptr<spi::ISpi> m_spi;
    std::shared_ptr<gpio::IPinOutput> m_chipSelect;
    const std::uint8_t m_cAddress;
};

} // namespace hal::gpio
