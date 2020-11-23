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

#include "GenericEeprom.hpp"

#include "hal/Error.hpp"
#include "hal/i2c/ScopedI2c.hpp"
#include "hal/utils/logger.hpp"

#include <utils/bits.hpp>

#include <cassert>
#include <limits>
#include <utility>

namespace hal::storage {

/// Returns pair of normalized EEPROM address and byte address. If address of the byte is greater than 16bit number,
/// than most significant bits of that address are or-ed with the device I2C address.
/// @param baseDeviceAddress        Base I2C address of the EEPROM device.
/// @param baseByteAddress          Address of the byte in EEPROM device to be accessed.
/// @return Pair of normalized EEPROM address and byte address.
static inline auto normalizeAddresses(std::uint16_t baseDeviceAddress, std::uint32_t baseByteAddress)
{
    std::uint16_t deviceAddress = baseDeviceAddress | (baseByteAddress >> (sizeof(std::uint16_t) * CHAR_BIT));
    std::uint32_t byteAddress = baseByteAddress & std::numeric_limits<std::uint16_t>::max();

    return std::make_pair(deviceAddress, byteAddress);
}

GenericEeprom::GenericEeprom(std::shared_ptr<i2c::II2c> i2c,
                             std::uint16_t address,
                             i2c::AddressingMode addressingMode,
                             std::size_t size,
                             std::size_t pageSize,
                             std::chrono::milliseconds writeDelay)
    : IEeprom(size, pageSize)
    , m_i2c(std::move(i2c))
    , m_address(address)
    , m_writeDelay(writeDelay, true)
{
    m_initialized = utils::isPowerOf2(pageSize) && i2c::verifyAddress(addressingMode, m_address);
    if (!m_initialized) {
        GenericEepromLogger::critical("Failed to create generic EEPROM: bad parameters");
        assert(false);
        return;
    }

    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        GenericEepromLogger::critical(
            "Failed to create generic EEPROM: timeout when locking I2C bus (timeout=default)");
        assert(false);
        return;
    }

    m_i2c->open();

    GenericEepromLogger::info("Created generic EEPROM with the following parameters:");
    GenericEepromLogger::info("  addressingMode : {}",
                              (addressingMode == i2c::AddressingMode::e7bit) ? "7-bit" : "10-bit");
    GenericEepromLogger::info("  address        : {:#x}", address);
    GenericEepromLogger::info("  size           : {:#x}", getSize());
    GenericEepromLogger::info("  pageSize       : {:#x}", getPageSize());
}

GenericEeprom::~GenericEeprom()
{
    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        GenericEepromLogger::error("Failed to close I2C device: timeout when locking I2C bus (timeout=default)");
        return;
    }

    m_i2c->close();
}

std::error_code
GenericEeprom::drvWrite(std::uint32_t address, const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout)
{
    if (!isInitialized()) {
        GenericEepromLogger::error("Failed to write: driver is not initialized");
        return Error::eDeviceNotOpened;
    }

    GenericEepromLogger::debug("Starting write transaction of {:#x} bytes to {:#x} address", size, address);

    auto currentAddress = address;
    const auto* currentBytes = bytes;
    std::size_t toWrite = size;
    std::size_t writeSize = getPageSize() - (address & ((getPageSize() - 1)));
    writeSize = std::min(writeSize, size);

    while (toWrite != 0) {
        if (auto error = writePage(currentAddress, currentBytes, writeSize, timeout)) {
            GenericEepromLogger::warn("Failed to write: err={}", error.message());
            return error;
        }

        currentAddress += writeSize;
        currentBytes += writeSize;
        toWrite -= writeSize;
        writeSize = std::min(getPageSize(), toWrite);
    }

    return Error::eOk;
}

std::error_code GenericEeprom::drvRead(std::uint32_t address,
                                       std::uint8_t* bytes,
                                       std::size_t size,
                                       osal::Timeout timeout,
                                       std::size_t& actualReadSize)
{
    if (!isInitialized()) {
        GenericEepromLogger::error("Failed to read: driver is not initialized");
        return Error::eDeviceNotOpened;
    }

    osal::sleepUntilExpired(m_writeDelay);

    actualReadSize = 0;
    GenericEepromLogger::debug("Starting read transaction of {:#x} bytes from {:#x} address", size, address);

    auto currentAddress = address;
    auto* currentBytes = bytes;
    std::size_t toRead = size;

    while (toRead != 0) {
        auto [deviceAddress, readAddress] = normalizeAddresses(m_address, currentAddress);
        std::size_t readSize
            = std::min(toRead, std::size_t(std::numeric_limits<std::uint16_t>::max()) + 1 - readAddress);

        GenericEepromLogger::trace("Normalized read: deviceAddress={:#x}, readAddress={:#x}, readSize={:#x}",
                                   deviceAddress,
                                   readAddress,
                                   readSize);

        i2c::ScopedI2c lock(m_i2c, timeout);
        if (!lock.isAcquired()) {
            GenericEepromLogger::warn("Failed to read: locking I2C bus (timeout={} ms)", osal::durationMs(timeout));
            return Error::eTimeout;
        }

        // Write only the address, don't send the STOP. STOP will be sent after the read.
        auto addressArray = utils::toBytesArray(utils::toBigEndian<std::uint16_t>(readAddress));
        if (auto error = m_i2c->write(deviceAddress, addressArray.data(), addressArray.size(), false, timeout)) {
            GenericEepromLogger::warn("Failed to read: I2C write() returned err={} (timeout={} ms)",
                                      error.message(),
                                      osal::durationMs(timeout));
            return error;
        }

        std::size_t successfulReadSize{};
        if (auto error = m_i2c->read(deviceAddress, currentBytes, readSize, timeout, successfulReadSize)) {
            GenericEepromLogger::warn("Failed to read: I2C read() returned err={} (timeout={} ms)",
                                      error.message(),
                                      osal::durationMs(timeout));
            return error;
        }

        currentAddress += successfulReadSize;
        currentBytes += successfulReadSize;
        toRead -= successfulReadSize;
    }

    actualReadSize = size;

    GenericEepromLogger::trace("Successfully read {:#x} bytes", actualReadSize);
    return Error::eOk;
}

std::error_code
GenericEeprom::writePage(std::uint32_t address, const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout)
{
    assert(size <= getPageSize());
    if (size == 0)
        return Error::eOk;

    osal::sleepUntilExpired(m_writeDelay);

    GenericEepromLogger::trace("Write page: address={:#x}, size={:#x}", address, size);

    auto [deviceAddress, writeAddress] = normalizeAddresses(m_address, address);
    auto addressArray = utils::toBytesArray(utils::toBigEndian<std::uint16_t>(writeAddress));

    GenericEepromLogger::trace("Normalized write: deviceAddress={:#x}, writeAddress={:#x}",
                               deviceAddress,
                               writeAddress);

    BytesVector data;
    data.insert(data.end(), addressArray.begin(), addressArray.end());
    data.insert(data.end(), bytes, bytes + size);

    i2c::ScopedI2c lock(m_i2c, timeout);
    if (!lock.isAcquired()) {
        GenericEepromLogger::warn("Failed to write page: timeout when locking I2C bus (timeout={} ms)",
                                  osal::durationMs(timeout));
        return Error::eTimeout;
    }

    // One contiguous write, send STOP when finished.
    auto error = m_i2c->write(deviceAddress, data, true, timeout);
    if (error) {
        GenericEepromLogger::error("Failed to write page: I2C write() returned err={} (timeout={} ms)",
                                   error.message(),
                                   osal::durationMs(timeout));
    }

    m_writeDelay.reset();
    return error;
}

} // namespace hal::storage
