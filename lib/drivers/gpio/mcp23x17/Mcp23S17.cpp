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

#include "hal/gpio/Mcp23S17.hpp"

#include "hal/Error.hpp"
#include "hal/spi/ScopedSpi.hpp"
#include "hal/utils/logger.hpp"

#include <array>
#include <utility>

namespace hal::gpio {

Mcp23S17::Mcp23S17(std::shared_ptr<spi::ISpi> spi,
                   std::shared_ptr<gpio::IPinOutput> chipSelect,
                   std::uint8_t address,
                   mcp23x17::Port port,
                   std::chrono::milliseconds busTimeout,
                   std::chrono::milliseconds timeoutIn,
                   std::chrono::milliseconds timeoutOut)
    : IMcp23x17(port, busTimeout, timeoutIn, timeoutOut)
    , m_spi(std::move(spi))
    , m_chipSelect(std::move(chipSelect))
    , m_cAddress(address)
{
    m_chipSelect->off();

    m_spi->lock(osal::Timeout::infinity());
    m_spi->open();
    m_spi->unlock();

    spiWrite(mcp23x17::Bank0::eIOCON, mcp23x17::IOCON::eHAEN, busTimeout);

    IMcp23x17::setInitialized();

    Mcp23S17Logger::info("Created MCP23S17 (SPI) GPIO expander with the following parameters:");
    Mcp23S17Logger::info("  address      : {:#04x}", m_cAddress);
    Mcp23S17Logger::info("  port         : {}", port == mcp23x17::Port::eGpioA ? "A" : "B");
    Mcp23S17Logger::info("  busTimeoutMs : {} ms", busTimeout.count());
    Mcp23S17Logger::info("  timeoutIn    : {} ms", timeoutIn.count());
    Mcp23S17Logger::info("  timeoutOut   : {} ms", timeoutOut.count());
}

Mcp23S17::~Mcp23S17()
{
    spi::ScopedSpi lock(m_spi, m_cSpiParams, m_chipSelect);
    if (lock.isAcquired())
        m_spi->close();
}

std::error_code Mcp23S17::readCommand(std::uint8_t address, std::uint8_t& response, osal::Timeout timeout)
{
    return spiRead(address, response, timeout);
}

std::error_code Mcp23S17::writeCommand(std::uint8_t address, std::uint8_t value, osal::Timeout timeout)
{
    return spiWrite(address, value, timeout);
}

std::error_code Mcp23S17::spiRead(std::uint8_t address, std::uint8_t& response, osal::Timeout timeout)
{
    std::array<std::uint8_t, 3> spiCmd{};
    constexpr std::uint8_t cReadCmd = 0x01;
    spiCmd[0] = std::uint8_t(m_cAddress << 1U) | cReadCmd;
    spiCmd[1] = address;
    spiCmd[2] = 0;

    spi::ScopedSpi lock(m_spi, m_cSpiParams, m_chipSelect, timeout);
    if (!lock.isAcquired()) {
        Mcp23S17Logger::error("Failed to acquire the SPI lock");
        return Error::eTimeout;
    }

    std::array<std::uint8_t, 3> spiResponse{};
    std::size_t actualReadSize{};
    auto result = m_spi->transfer(spiCmd.data(), spiResponse.data(), spiCmd.size(), timeout, actualReadSize);
    response = spiResponse[2];
    return result;
}

std::error_code Mcp23S17::spiWrite(std::uint8_t address, std::uint8_t value, osal::Timeout timeout)
{
    std::array<std::uint8_t, 3> spiCmd{};
    constexpr std::uint8_t cWriteCmd = 0x00;
    spiCmd[0] = std::uint8_t(m_cAddress << 1U) | cWriteCmd;
    spiCmd[1] = address;
    spiCmd[2] = value;

    spi::ScopedSpi lock(m_spi, m_cSpiParams, m_chipSelect, timeout);
    if (!lock.isAcquired()) {
        Mcp23S17Logger::error("Failed to acquire the SPI lock");
        return Error::eTimeout;
    }

    return m_spi->write(spiCmd.data(), spiCmd.size(), timeout);
}

} // namespace hal::gpio
