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

#include "hal/utils/logger.hpp"

#include <algorithm>
#include <cassert>

namespace hal::spi {

ISpi::ISpi(ISpi&& other) noexcept
    : m_mutex(std::move(other.m_mutex))
{
    std::swap(m_userCount, other.m_userCount);
    std::swap(m_opened, other.m_opened);
    std::swap(m_locked, other.m_locked);
}

ISpi::~ISpi()
{
    assert(m_userCount == 0 && !m_locked);
}

std::error_code ISpi::open()
{
    if (!isLocked()) {
        SpiLogger::error("Failed to open: bus is not locked");
        return Error::eWrongState;
    }

    ++m_userCount;

    if (isOpened()) {
        SpiLogger::info("Device is already opened");
        return Error::eDeviceOpened;
    }

    auto error = drvOpen();
    m_opened = !error;
    return error;
}

std::error_code ISpi::close()
{
    if (!isLocked()) {
        SpiLogger::error("Failed to close: bus is not locked");
        return Error::eWrongState;
    }

    if (!isOpened()) {
        SpiLogger::error("Failed to close: device is not opened");
        return Error::eDeviceNotOpened;
    }

    --m_userCount;
    if (m_userCount == 0) {
        auto error = drvClose();
        m_opened = static_cast<bool>(error);
        return error;
    }

    SpiLogger::info("Skipping close: device has other users");
    return Error::eOk;
}

std::error_code ISpi::setParams(SpiParams params)
{
    if (auto error = checkState()) {
        SpiLogger::error("Failed to set params: invalid state err={}", error.message());
        return error;
    }

    return drvSetParams(params);
}

std::error_code ISpi::lock(osal::Timeout timeout)
{
    if (!isLocked()) {
        if (auto error = m_mutex.timedLock(timeout)) {
            SpiLogger::warn("Failed to lock SPI bus: err={} (timeout={} ms)",
                            error.message(),
                            osal::durationMs(timeout));
            return error;
        }

        m_locked = true;

        SpiLogger::trace("Bus successfully locked");
        return Error::eOk;
    }

    SpiLogger::warn("Failed to lock SPI bus: device is already locked");
    return Error::eWrongState;
}

std::error_code ISpi::unlock()
{
    if (!isLocked()) {
        SpiLogger::error("Failed to unlock: bus is not locked");
        return Error::eWrongState;
    }

    m_locked = false;

    if (auto error = m_mutex.unlock()) {
        SpiLogger::critical("Failed to unlock SPI bus: mutex error (err={})", error.message());
        assert(false);
        return error;
    }

    SpiLogger::trace("Bus successfully unlocked");
    return Error::eOk;
}

std::error_code ISpi::write(const BytesVector& bytes, osal::Timeout timeout)
{
    return write(bytes.data(), bytes.size(), timeout);
}

std::error_code ISpi::write(const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout)
{
    if (bytes == nullptr) {
        SpiLogger::error("Failed to write: bytes=nullptr");
        return Error::eInvalidArgument;
    }

    if (auto error = checkState()) {
        SpiLogger::error("Failed to write: invalid state err={}", error.message());
        return error;
    }

    return drvWrite(bytes, size, timeout);
}

std::error_code ISpi::read(BytesVector& bytes, std::size_t size, osal::Timeout timeout)
{
    bytes.resize(size);
    if (bytes.size() != size) {
        SpiLogger::error("Failed to read: cannot resize output vector");
        return Error::eNoMemory;
    }

    std::size_t actualReadSize{};
    auto error = read(bytes.data(), size, timeout, actualReadSize);
    bytes.resize(actualReadSize);
    return error;
}

std::error_code ISpi::read(std::uint8_t* bytes, std::size_t size, osal::Timeout timeout, std::size_t& actualReadSize)
{
    if (auto error = checkState()) {
        SpiLogger::error("Failed to read: invalid state err={}", error.message());
        return error;
    }

    actualReadSize = 0;
    return drvRead(bytes, size, timeout, actualReadSize);
}

std::error_code ISpi::transfer(const BytesVector& txBytes, BytesVector& rxBytes, osal::Timeout timeout)
{
    rxBytes.resize(txBytes.size());
    if (rxBytes.size() != txBytes.size()) {
        SpiLogger::error("Failed to transfer: cannot resize output vector");
        return Error::eNoMemory;
    }

    std::size_t actualReadSize{};
    if (auto error = transfer(txBytes.data(), rxBytes.data(), txBytes.size(), timeout, actualReadSize))
        return error;

    rxBytes.resize(actualReadSize);
    return Error::eOk;
}

std::error_code ISpi::transfer(const std::uint8_t* txBytes,
                               std::uint8_t* rxBytes,
                               std::size_t size,
                               osal::Timeout timeout,
                               std::size_t& actualReadSize)
{
    if ((txBytes == nullptr) && (rxBytes == nullptr)) {
        SpiLogger::error("Failed to transfer: txBytes={}, rxBytes={}", txBytes, rxBytes);
        return Error::eInvalidArgument;
    }

    if (auto error = checkState()) {
        SpiLogger::error("Failed to transfer: invalid state err={}", error.message());
        return error;
    }

    actualReadSize = 0;
    return drvTransfer(txBytes, rxBytes, size, timeout, actualReadSize);
}

bool ISpi::isLocked()
{
    if (auto error = m_mutex.tryLock()) {
        SpiLogger::trace("isLocked(): Failed to lock the mutex (err={})", error.message());
        return false;
    }

    bool result = m_locked;
    m_mutex.unlock();
    return result;
}

std::error_code ISpi::checkState()
{
    if (!isLocked()) {
        SpiLogger::error("Checking state: bus is not locked");
        return Error::eWrongState;
    }

    if (!isOpened()) {
        SpiLogger::error("Checking state: device is not opened");
        return Error::eDeviceNotOpened;
    }

    return Error::eOk;
}

} // namespace hal::spi
