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

#include "hal/spi/ISpi.hpp"

#include <osal/Timeout.hpp>

#include <cstdint>
#include <string>
#include <system_error>

namespace hal::spi {

// clang-format off
/// Represents the possible SPI significant bits mode.
enum class SignificantBits {
    eMsb = 0,
    eLsb = 1
};
// clang-format on

/// Represents Linux driver for the II2c interface.
class LinuxSpi : public ISpi {
public:
    //// Constructor.
    /// @param device               Device path.
    explicit LinuxSpi(std::string device);

private:
    /// @see ISpi::drvOpen().
    std::error_code drvOpen() override;

    /// @see ISpi::drvClose().
    std::error_code drvClose() override;

    /// @see ISpi::drvSetParams().
    std::error_code drvSetParams(SpiParams params) override;

    /// @see ISpi::drvWrite().
    std::error_code drvWrite(const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout) override;

    /// @see ISpi::drvRead().
    std::error_code
    drvRead(std::uint8_t* bytes, std::size_t size, osal::Timeout timeout, std::size_t& actualReadSize) override;

    /// @see ISpi::drvTransfer().
    std::error_code drvTransfer(const std::uint8_t* txBytes,
                                std::uint8_t* rxBytes,
                                std::size_t size,
                                osal::Timeout timeout,
                                std::size_t& actualReadSize) override;

    /// Sets frequency of the SPI bus.
    /// @param frequencyHz          Frequency to be set in Hz.
    /// @return Error code of the operation.
    std::error_code setFrequency(std::uint32_t frequencyHz);

    /// Sets mode of the SPI bus.
    /// @param mode                 Clock mode ot be set.
    /// @return Error code of the operation.
    [[nodiscard]] std::error_code setMode(Mode mode) const;

    /// Sets word length of the SPI bus.
    /// @param wordLength           Word length to be set.
    /// @return Error code of the operation.
    std::error_code setLength(std::uint8_t wordLength);

    /// Sets significant bits mode of the SPI bus.
    /// @param significantBits      Significant bits mode to be set.
    /// @return Error code of the operation.
    [[nodiscard]] std::error_code setSignificantBits(SignificantBits significantBits) const;

private:
    int m_fd{};
    std::uint32_t m_frequencyHz{};
    std::uint8_t m_wordLength{};
    std::string m_device;
};

} // namespace hal::spi
