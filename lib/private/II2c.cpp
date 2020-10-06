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

#include "hal/utils/logger.hpp"

#include <cassert>

namespace hal::i2c {

bool verifyAddress(AddressingMode addressingMode, std::uint16_t address)
{
    constexpr std::uint16_t cBitMask7 = 0xff80;
    constexpr std::uint16_t cBitMask10 = 0xfc00;

    switch (addressingMode) {
        case AddressingMode::e7bit: return ((address & cBitMask7) == 0);
        case AddressingMode::e10bit: return ((address & cBitMask10) == 0);
        default: break;
    }

    I2cLogger::critical("Bad I2C address: addressingMode={}, address={}", addressingMode, address);
    return false;
}

II2c::II2c(II2c&& other) noexcept
    : m_mutex(std::move(other.m_mutex))
{}

II2c::~II2c()
{
    assert(m_userCount == 0 && m_locked);
}

std::error_code II2c::open()
{
    if (!isLocked()) {
        I2cLogger::error("Failed to open: bus is not locked");
        return Error::eWrongState;
    }

    ++m_userCount;

    if (isOpened()) {
        I2cLogger::info("Failed to open: device is already opened");
        return Error::eDeviceOpened;
    }

    auto error = drvOpen();
    m_opened = !error;
    return error;
}

std::error_code II2c::close()
{
    if (!isLocked()) {
        I2cLogger::error("Failed to close: bus is not locked");
        return Error::eWrongState;
    }

    if (!isOpened()) {
        I2cLogger::error("Failed to close: device is not opened");
        return Error::eDeviceNotOpened;
    }

    --m_userCount;
    if (m_userCount == 0) {
        auto error = drvClose();
        m_opened = static_cast<bool>(error);
        return error;
    }

    I2cLogger::info("Skipping close: device has other users");
    return Error::eOk;
}

std::error_code II2c::lock(osal::Timeout timeout)
{
    if (!isLocked()) {
        if (auto error = m_mutex.timedLock(timeout)) {
            I2cLogger::warn("Failed to lock I2C bus: err={} (timeout={} ms)",
                            error.message(),
                            osal::durationMs(timeout));
            return error;
        }

        m_locked = true;

        I2cLogger::trace("Bus successfully locked");
        return Error::eOk;
    }

    I2cLogger::warn("Failed to lock I2C bus: device is already locked");
    return Error::eWrongState;
}

std::error_code II2c::unlock()
{
    if (!isLocked()) {
        I2cLogger::error("Failed to unlock: bus is not locked");
        return Error::eWrongState;
    }

    m_locked = false;

    if (auto error = m_mutex.unlock()) {
        I2cLogger::critical("Failed to unlock I2C bus: mutex error");
        assert(false);
        return error;
    }

    I2cLogger::trace("Bus successfully unlocked");
    return Error::eOk;
}

std::error_code II2c::write(std::uint16_t address, const BytesVector& bytes, bool stop, osal::Timeout timeout)
{
    return write(address, bytes.data(), bytes.size(), stop, timeout);
}

std::error_code
II2c::write(std::uint16_t address, const std::uint8_t* bytes, std::size_t size, bool stop, osal::Timeout timeout)
{
    if (bytes == nullptr) {
        I2cLogger::error("Failed to write: bytes=nullptr");
        return Error::eInvalidArgument;
    }

    if (auto error = checkState()) {
        I2cLogger::error("Failed to write: invalid state err={}", error.message());
        return error;
    }

    return drvWrite(address, bytes, size, stop, timeout);
}

std::error_code II2c::read(std::uint16_t address, BytesVector& bytes, std::size_t size, osal::Timeout timeout)
{
    bytes.resize(size);
    if (bytes.size() != size) {
        I2cLogger::error("Failed to read: cannot resize output vector");
        return Error::eNoMemory;
    }

    std::size_t actualReadSize{};
    auto error = read(address, bytes.data(), size, timeout, actualReadSize);
    bytes.resize(actualReadSize);
    if (error)
        I2cLogger::error("Failed to read: err={}", error.message());

    return error;
}

std::error_code II2c::read(std::uint16_t address,
                           std::uint8_t* bytes,
                           std::size_t size,
                           osal::Timeout timeout,
                           std::size_t& actualReadSize)
{
    if (bytes == nullptr) {
        I2cLogger::error("Failed to read: bytes=nullptr");
        return Error::eInvalidArgument;
    }

    if (auto error = checkState()) {
        I2cLogger::error("Failed to read: invalid state err={}", error.message());
        return error;
    }

    actualReadSize = 0;
    return drvRead(address, bytes, size, timeout, actualReadSize);
}

bool II2c::isLocked()
{
    if (auto error = m_mutex.tryLock()) {
        I2cLogger::trace("isLocked(): Failed to lock the mutex, err={}", error.message());
        return false;
    }

    bool result = m_locked;
    m_mutex.unlock();

    return result;
}

std::error_code II2c::checkState()
{
    if (!isLocked()) {
        I2cLogger::error("Checking state: bus is not locked");
        return Error::eWrongState;
    }

    if (!isOpened()) {
        I2cLogger::error("Checking state: device is not opened");
        return Error::eDeviceNotOpened;
    }

    return Error::eOk;
}

} // namespace hal::i2c
