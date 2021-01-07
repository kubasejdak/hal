/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Grzegorz Heldt
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

#include "hal/types.hpp"

#include <osal/Mutex.hpp>
#include <osal/Timeout.hpp>
#include <utils/GlobalRegistry.hpp>

#include <system_error>

namespace hal::spi {

// clang-format off
/// Represents the clock phase and polarity selected to be used in the SPI transmission.
enum class Mode {
    eMode0,
    eMode1,
    eMode2,
    eMode3
};
// clang-format on

/// Represents SPI parameters that can be set by each driver, which uses SPI.
struct SpiParams {
    std::uint32_t frequencyHz{};
    Mode clockMode{};
    std::uint8_t wordLength{};
};

/// Represents the SPI bus controller. There should be one instance for each bus.
class ISpi {
public:
    /// Default constructor.
    ISpi() = default;

    /// Copy constructor.
    /// @note This constructor is deleted, because ISpi is not meant to be copy-constructed.
    ISpi(const ISpi&) = delete;

    /// Move constructor.
    /// @param other                ISpi object to be moved into current instance.
    ISpi(ISpi&& other) noexcept;

    /// Virtual destructor.
    virtual ~ISpi();

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because ISpi is not meant to be copy-assigned.
    ISpi& operator=(const ISpi&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because ISpi is not meant to be move-assigned.
    ISpi& operator=(ISpi&&) = delete;

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

    /// Sets the transmission parameters.
    /// @param params               Set of transmission parameters.
    /// @return Error code of the operation.
    std::error_code setParams(SpiParams params);

    /// Locks the SPI bus for the current device.
    /// @param timeout              Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code lock(osal::Timeout timeout);

    /// Unlocks the SPI bus.
    /// @return Error code of the operation.
    std::error_code unlock();

    /// Transmits given vector of bytes to the SPI device.
    /// @param bytes                Vector of raw bytes to be transmitted.
    /// @param timeout              Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code write(const BytesVector& bytes, osal::Timeout timeout);

    /// Transmits given memory block of bytes to the SPI device.
    /// @param bytes                Memory block of raw bytes to be transmitted.
    /// @param size                 Size of the memory block to be transmitted.
    /// @param timeout              Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code write(const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout);

    /// Receives the demanded number of bytes from the SPI device.
    /// @param bytes                Vector where the received data will be placed.
    /// @param size                 Number of bytes to be received.
    /// @param timeoutMs            Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code read(BytesVector& bytes, std::size_t size, osal::Timeout timeout);

    /// Receives the demanded number of bytes from the SPI device.
    /// @param bytes                Memory block where the received data will be placed.
    /// @param size                 Number of bytes to be received.
    /// @param timeout              Maximal time to wait for the bus.
    /// @param actualReadSize       Actual number of received bytes.
    /// @return Error code of the operation.
    std::error_code read(std::uint8_t* bytes, std::size_t size, osal::Timeout timeout, std::size_t& actualReadSize);

    /// Transmits given vector of bytes to the SPI device and concurrently reads its response.
    /// @param txBytes              Vector of raw bytes to be transmitted.
    /// @param rxBytes              Vector where the received data will be placed.
    /// @param timeout              Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code transfer(const BytesVector& txBytes, BytesVector& rxBytes, osal::Timeout timeout);

    /// Transmits given memory block of bytes to the SPI device and concurrently reads its response.
    /// @param txBytes              Memory block of raw bytes to be transmitted.
    /// @param rxBytes              Memory block where the received data will be placed.
    /// @param size                 Size of the memory block to be transferred.
    /// @param timeout              Maximal time to wait for the bus.
    /// @param actualReadSize       Actual number of received bytes.
    /// @return Error code of the operation.
    std::error_code transfer(const std::uint8_t* txBytes,
                             std::uint8_t* rxBytes,
                             std::size_t size,
                             osal::Timeout timeout,
                             std::size_t& actualReadSize);

private:
    /// Checks if the SPI bus is locked.
    /// @return Flag indicating if the SPI bus is locked.
    /// @retval true                Bus is locked.
    /// @retval false               Bus is not locked.
    bool isLocked();

    /// Checks if the device is opened.
    /// @return Flag indicating if the device is opened.
    /// @retval true                Device is opened.
    /// @retval false               Device is not opened.
    [[nodiscard]] bool isOpened() const { return m_opened; }

    /// Checks the operating state of the driver, which includes checking if it is properly initialized, if the device
    /// is opened and if the SPI bus is locked by this instance of the driver.
    /// @return Error code of the operation.
    std::error_code checkState();

    /// Driver specific implementation of opening the transmission channel. If the configuration of this device is
    /// valid, then after a successful call to this method the device will be able to transmit the data according
    /// to the settings.
    /// @return Error code of the operation.
    virtual std::error_code drvOpen() = 0;

    /// Driver specific implementation of closing the transmission channel. After a successful call to this
    /// method the device will not be able to transmit any data.
    /// @return Error code of the operation.
    virtual std::error_code drvClose() = 0;

    /// Driver specific implementation of setting the transmission params.
    /// @param params               Set of transmission parameters.
    /// @return Error code of the operation.
    virtual std::error_code drvSetParams(SpiParams params) = 0;

    /// Driver specific implementation of transmitting given memory block of bytes to the SPI device.
    /// @param data                 Data to be transmitted.
    /// @param size                 Number of data to transmit.
    /// @param timeout              Maximal time to wait for the bus.
    /// @return Error code of the operation.
    virtual std::error_code drvWrite(const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout) = 0;

    /// Driver specific implementation of the receiving demanded number of bytes from the SPI device.
    /// @param data                 Place where received data should be placed.
    /// @param size                 Number of data to receive.
    /// @param timeout              Maximal time to wait for the bus.
    /// @return Error code of the operation.
    virtual std::error_code
    drvRead(std::uint8_t* bytes, std::size_t size, osal::Timeout timeout, std::size_t& actualReadSize)
        = 0;

    /// Driver specific implementation of the sending specific number of words.
    /// @param txData               Data to be transmitted.
    /// @param rxData               Place where received byte should be placed.
    /// @param size                 Number of data to transmit and/or receive.
    /// @param timeout              Maximal time to wait for the bus.
    /// @param actualReadSize       Actual number of received bytes.
    /// @return Error code of the operation.
    virtual std::error_code drvTransfer(const std::uint8_t* txBytes,
                                        std::uint8_t* rxBytes,
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

/// Represents GlobalRegistry of ISpi instances.
using Registry = utils::GlobalRegistry<ISpi>;

} // namespace hal::spi
