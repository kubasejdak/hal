/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Grzegorz Heldt
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

#pragma once

#include "hal/types.hpp"

#include <osal/Mutex.hpp>
#include <osal/Timeout.hpp>
#include <utils/GlobalRegistry.hpp>

#include <cstdint>
#include <system_error>

namespace hal::i2c {

/// Represents the I2C addressing mode.
enum class AddressingMode {
    e7bit, // NOLINT(readability-identifier-naming)
    e10bit // NOLINT(readability-identifier-naming)
};

/// Verifies if the given I2C device address is valid in terms of the given addressing mode.
/// @param address              Address to be checked.
/// @return Flag indicating if the given address is valid.
/// @retval true                Given address is valid.
/// @retval false               Given address is invalid.
bool verifyAddress(AddressingMode addressingMode, std::uint16_t address);

/// Represents the I2C bus controller. There should be one instance for each bus.
class II2c {
public:
    /// Default constructor.
    II2c() = default;

    /// Copy constructor.
    /// @note This constructor is deleted, because II2c is not meant to be copy-constructed.
    II2c(const II2c&) = delete;

    /// Move constructor.
    /// @param other            II2c object to be moved into current instance.
    II2c(II2c&& other) noexcept;

    /// Virtual destructor.
    virtual ~II2c();

    /// Assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because II2c is not meant to be copy-assigned.
    II2c& operator=(const II2c&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because II2c is not meant to be move-assigned.
    II2c& operator=(II2c&&) = delete;

    /// Opens the transmission channel. If the configuration of this device is valid, then after a successful
    /// call to this method device will be able to transmit data according to the settings.
    /// @return Error code of the operation.
    /// @note This method should be called only right after client obtains this device from the factory, not before
    ///       each transfer.
    std::error_code open();

    /// Closes the transmission channel. After a successful call to this method device will not be able
    /// to transmit any data.
    /// @return Error code of the operation.
    /// @note This method should be called only when client is about to return this device to the factory, not after
    ///       each transfer.
    std::error_code close();

    /// Locks the I2C bus for the current device.
    /// @param timeout          Maximal time to wait for the bus in ms.
    /// @return Error code of the operation.
    std::error_code lock(osal::Timeout timeout);

    /// Unlocks the I2C bus.
    /// @return Error code of the operation.
    std::error_code unlock();

    /// Transmits given vector of bytes to the current I2C device.
    /// @param address          Address of the I2C slave device.
    /// @param bytes            Vector of raw bytes to be transmitted.
    /// @param stop             Flag indicating if stop condition should be generated after the transfer.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code write(std::uint16_t address, const BytesVector& bytes, bool stop, osal::Timeout timeout);

    /// Transmits given memory block of bytes to the current I2C device.
    /// @param address          Address of the I2C slave device.
    /// @param bytes            Memory block of raw bytes to be transmitted.
    /// @param size             Size of the memory block to be transmitted.
    /// @param stop             Flag indicating if stop condition should be generated after the transfer.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code
    write(std::uint16_t address, const std::uint8_t* bytes, std::size_t size, bool stop, osal::Timeout timeout);

    /// Receives demanded number of bytes from the current I2C device.
    /// @param address          Address of the I2C slave device.
    /// @param bytes            Vector where the received data will be placed by this method.
    /// @param size             Number of bytes to be received.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code read(std::uint16_t address, BytesVector& bytes, std::size_t size, osal::Timeout timeout);

    /// Receives demanded number of bytes from the current I2C device.
    /// @param address          Address of the I2C slave device.
    /// @param bytes            Memory block where the received data will be placed by this method.
    /// @param size             Number of bytes to be received.
    /// @param timeout          Maximal time to wait for the bus.
    /// @param actualReadSize   Actual number of received bytes.
    /// @return Error code of the operation.
    std::error_code read(std::uint16_t address,
                         std::uint8_t* bytes,
                         std::size_t size,
                         osal::Timeout timeout,
                         std::size_t& actualReadSize);

private:
    /// Checks if the I2C bus is locked.
    /// @return Flag indicating if the I2C bus is locked.
    /// @retval true            Bus is opened.
    /// @retval false           Bus is not opened.
    bool isLocked();

    /// Checks if the device is opened.
    /// @return Flag indicating if the device is opened.
    /// @retval true            Device is opened.
    /// @retval false           Device is not opened.
    [[nodiscard]] bool isOpened() const { return m_opened; }

    /// Checks the operating state of the driver, which includes checking if it is properly initialized, if the device
    /// is opened and if the I2C bus is locked by this instance of the driver.
    /// @return Error code of the operation.
    std::error_code checkState();

    /// Driver specific implementation of opening the transmission channel. If the configuration of this device is
    /// valid, then after a successful call to this method device will be able to transmit data according
    /// to the settings.
    /// @return Error code of the operation.
    virtual std::error_code drvOpen() = 0;

    /// Driver specific implementation of closing the transmission channel. After a successful call to this
    /// method device will not be able to transmit any data.
    /// @return Error code of the operation.
    virtual std::error_code drvClose() = 0;

    /// Driver specific implementation of sending the memory block of bytes.
    /// @param address          Address of the I2C slave device.
    /// @param bytes            Memory block of raw bytes to be transmitted.
    /// @param size             Size of the memory block to be transmitted.
    /// @param stop             Flag indicating if stop condition should be generated after the transfer.
    /// @param timeout          Maximal time to wait for the bus.
    /// @return Error code of the operation.
    virtual std::error_code
    drvWrite(std::uint16_t address, const std::uint8_t* bytes, std::size_t size, bool stop, osal::Timeout timeout)
        = 0;

    /// Driver specific implementation of reading demanded number of bytes.
    /// @param address               Address of the I2C slave device.
    /// @param bytes                 Memory block where the received data will be placed by this method.
    /// @param size                  Number of bytes to be received.
    /// @param timeout               Maximal time to wait for the bus.
    /// @param actualReadSize        Actual number of received bytes.
    /// @return Error code of the operation.
    virtual std::error_code drvRead(std::uint16_t address,
                                    std::uint8_t* bytes,
                                    std::size_t size,
                                    osal::Timeout timeout,
                                    std::size_t& actualReadSize)
        = 0;

private:
    std::uint32_t m_userCount{};
    bool m_opened{};
    bool m_locked{};
    osal::Mutex m_mutex{OsalMutexType::eRecursive};
};

/// Represents GlobalRegistry of II2c instances.
using Registry = utils::GlobalRegistry<II2c>;

} // namespace hal::i2c
