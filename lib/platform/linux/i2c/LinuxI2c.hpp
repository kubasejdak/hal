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

#include "hal/i2c/II2c.hpp"

#include <osal/Timeout.hpp>

#include <string>
#include <vector>

namespace hal::i2c {

/// Represents Linux driver for the II2c interface.
class LinuxI2c : public II2c {
public:
    /// Constructor.
    /// @param device           Device path.
    explicit LinuxI2c(std::string device);

private:
    /// @see II2c::drvOpen().
    std::error_code drvOpen() override;

    /// @see II2c::drvClose().
    std::error_code drvClose() override;

    /// @see II2c::drvWrite().
    std::error_code drvWrite(std::uint16_t address,
                             const std::uint8_t* bytes,
                             std::size_t size,
                             bool stop,
                             osal::Timeout timeout) override;

    /// @see II2c::drvRead().
    std::error_code drvRead(std::uint16_t address,
                            std::uint8_t* bytes,
                            std::size_t size,
                            osal::Timeout timeout,
                            std::size_t& actualReadSize) override;

    /// Sets slave address.
    /// @param address          Address to be set.
    /// @return Error code of the operation.
    [[nodiscard]] std::error_code setSlaveAddress(std::uint16_t address) const;

private:
    int m_fd{};
    std::string m_device;
    std::vector<std::vector<std::uint8_t>> m_writeData;
};

} // namespace hal::i2c
