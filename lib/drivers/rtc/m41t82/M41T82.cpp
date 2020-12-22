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

#include "M41T82.hpp"

#include "hal/Error.hpp"
#include "hal/i2c/ScopedI2c.hpp"
#include "hal/utils/logger.hpp"

#include <osal/Timeout.hpp>

#include <array>
#include <cassert>
#include <utility>

namespace hal::time {

// clang-format off
/// Names of the M41T82 RTC registers related to time values and their associated offsets.
enum TimeRegisterName {
    eHundredthsSeconds,
    eSeconds,
    eMinutes,
    eHours,
    eWeekDay,
    eDay,
    eMonth,
    eYear
};
// clang-format on

M41T82::M41T82(std::shared_ptr<i2c::II2c> i2c, std::uint16_t address)
    : m_i2c(std::move(i2c))
    , m_cAddress(address)
{
    if (!i2c::verifyAddress(i2c::AddressingMode::e7bit, m_cAddress)) {
        M41T82Logger::critical("Failed to create M41T82 RTC: bad parameters");
        assert(false);
        return;
    }

    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        M41T82Logger::critical("Failed to create M41T82 RTC: timeout when locking I2C bus (timeout=default)");
        assert(false);
        return;
    }

    m_i2c->open();

    M41T82Logger::info("Created M41T82 RTC with the following parameters:");
    M41T82Logger::info("  address      : {:#04x}", m_cAddress);
}

M41T82::~M41T82()
{
    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        M41T82Logger::error("Failed to close I2C device: timeout when locking I2C bus (timeout=default)");
        return;
    }

    m_i2c->close();
}

std::error_code M41T82::drvGetTime(std::tm& tm)
{
    osal::Timeout timeout = 100ms;
    i2c::ScopedI2c lock(m_i2c, timeout);
    if (!lock.isAcquired()) {
        M41T82Logger::warn("Failed to read: locking I2C bus (timeout={} ms)", osal::durationMs(timeout));
        return Error::eTimeout;
    }

    // Write only the address, don't send the STOP. STOP will be sent after the read.
    std::array<std::uint8_t, 1> offset{};
    if (auto error = m_i2c->write(m_cAddress, offset.data(), offset.size(), false, timeout)) {
        M41T82Logger::warn("Failed to read: I2C write() returned err={} (timeout={} ms)",
                           error.message(),
                           osal::durationMs(timeout));
        return error;
    }

    constexpr std::size_t cReadSize = 8;
    std::array<std::uint8_t, cReadSize> registers{};
    std::size_t successfulReadSize{};
    if (auto error = m_i2c->read(m_cAddress, registers.data(), registers.size(), timeout, successfulReadSize)) {
        M41T82Logger::warn("Failed to read: I2C read() returned err={} (timeout={} ms)",
                           error.message(),
                           osal::durationMs(timeout));
        return error;
    }

    if (successfulReadSize != registers.size()) {
        M41T82Logger::error("Failed to read: wrong read size: expected={}, actual={}",
                            registers.size(),
                            successfulReadSize);
        return Error::eHardwareError;
    }

    // Assume year is 20YY, not 19YY and ignore the century bit.
    auto bcdToBin = [](std::uint8_t x) { return (x & 0x0f) + (x >> 4) * 10; }; // NOLINT
    tm.tm_sec = bcdToBin(registers[eSeconds] & std::uint8_t(0x7f));            // NOLINT
    tm.tm_min = bcdToBin(registers[eMinutes] & 0x7f);                          // NOLINT
    tm.tm_hour = bcdToBin(registers[eHours] & 0x3f);                           // NOLINT
    tm.tm_mday = bcdToBin(registers[eDay] & 0x3f);                             // NOLINT
    tm.tm_mon = bcdToBin(registers[eMonth] & 0x1f) - 1;                        // NOLINT
    tm.tm_year = bcdToBin(registers[eYear]) + 100;                             // NOLINT
    tm.tm_wday = registers[eWeekDay] & 0x07;                                   // NOLINT

    return Error::eOk;
}

std::error_code M41T82::drvSetTime(const std::tm& tm)
{
    constexpr std::size_t cWriteSize = 8;
    std::array<std::uint8_t, cWriteSize> registers{};

    auto binToBcd = [](std::uint8_t x) { return ((x / 10) << 4) + x % 10; }; // NOLINT
    registers[eHundredthsSeconds] = 0;
    registers[eSeconds] = binToBcd(tm.tm_sec);
    registers[eMinutes] = binToBcd(tm.tm_min);
    registers[eHours] = binToBcd(tm.tm_hour);
    registers[eDay] = binToBcd(tm.tm_mday);
    registers[eMonth] = binToBcd(tm.tm_mon + 1);
    registers[eYear] = binToBcd(tm.tm_year - 100); // NOLINT
    registers[eWeekDay] = tm.tm_wday;

    osal::Timeout timeout = 100ms;
    i2c::ScopedI2c lock(m_i2c, timeout);
    if (!lock.isAcquired()) {
        M41T82Logger::warn("Failed to write: locking I2C bus (timeout={} ms)", osal::durationMs(timeout));
        return Error::eTimeout;
    }

    BytesVector data{0};
    data.insert(data.end(), registers.begin(), registers.end());

    if (auto error = m_i2c->write(m_cAddress, data.data(), data.size(), true, timeout)) {
        M41T82Logger::warn("Failed to write: I2C write() returned err={} (timeout={} ms)",
                           error.message(),
                           osal::durationMs(timeout));
        return error;
    }

    return Error::eOk;
}

} // namespace hal::time
