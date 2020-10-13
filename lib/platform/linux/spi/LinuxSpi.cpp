/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2020-2020, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include "LinuxSpi.hpp"

#include "hal/Error.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>

namespace hal::spi {

LinuxSpi::LinuxSpi(std::string device)
    : m_device(std::move(device))
{}

std::error_code LinuxSpi::drvOpen()
{
    m_fd = ::open(m_device.c_str(), O_RDWR | O_CLOEXEC); // NOLINT
    if (m_fd < 0)
        return Error::eFilesystemError;

    return Error::eOk;
}

std::error_code LinuxSpi::drvClose()
{
    if (::close(m_fd) < 0)
        return Error::eFilesystemError;

    return Error::eOk;
}

std::error_code LinuxSpi::drvSetParams(SpiParams params)
{
    if (auto error = setFrequency(params.frequencyHz))
        return error;

    if (auto error = setMode(params.clockMode))
        return error;

    if (auto error = setLength(params.wordLength))
        return error;

    return setSignificantBits(SignificantBits::eMsb);
}

std::error_code LinuxSpi::drvWrite(const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout)
{
    std::size_t actualReadSize{};
    auto error = drvTransfer(bytes, nullptr, size, timeout, actualReadSize);
    (void) actualReadSize;
    return error;
}

std::error_code
LinuxSpi::drvRead(std::uint8_t* bytes, std::size_t size, osal::Timeout timeout, std::size_t& actualReadSize)
{
    return drvTransfer(nullptr, bytes, size, timeout, actualReadSize);
}

std::error_code LinuxSpi::drvTransfer(const std::uint8_t* txBytes,
                                      std::uint8_t* rxBytes,
                                      std::size_t size,
                                      osal::Timeout /*unused*/,
                                      std::size_t& actualReadSize)
{
    std::vector<std::uint8_t> buffer(size);
    if (txBytes != nullptr)
        std::memcpy(buffer.data(), txBytes, size);

    spi_ioc_transfer ioc{};
    ioc.tx_buf = reinterpret_cast<std::uint64_t>(buffer.data()); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    ioc.rx_buf = reinterpret_cast<std::uint64_t>(buffer.data()); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    ioc.len = buffer.size();
    ioc.speed_hz = m_frequencyHz;
    ioc.bits_per_word = m_wordLength;
    if (ioctl(m_fd, SPI_IOC_MESSAGE(1), &ioc) < 0) // NOLINT
        return Error::eHardwareError;

    if (rxBytes != nullptr)
        std::memcpy(rxBytes, buffer.data(), buffer.size());

    actualReadSize = size;
    return Error::eOk;
}

std::error_code LinuxSpi::setFrequency(std::uint32_t frequencyHz)
{
    if (ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &frequencyHz) < 0) // NOLINT
        return Error::eHardwareError;

    m_frequencyHz = frequencyHz;
    return Error::eOk;
}

std::error_code LinuxSpi::setMode(Mode mode) const
{
    if (ioctl(m_fd, SPI_IOC_WR_MODE, &mode) < 0) // NOLINT
        return Error::eHardwareError;

    return Error::eOk;
}

std::error_code LinuxSpi::setLength(std::uint8_t wordLength)
{
    if (ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &wordLength) < 0) // NOLINT
        return Error::eHardwareError;

    m_wordLength = wordLength;
    return Error::eOk;
}

std::error_code LinuxSpi::setSignificantBits(SignificantBits significantBits) const
{
    if (ioctl(m_fd, SPI_IOC_WR_LSB_FIRST, &significantBits) < 0) // NOLINT
        return Error::eHardwareError;

    return Error::eOk;
}

} // namespace hal::spi
