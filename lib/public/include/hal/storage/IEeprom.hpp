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

#pragma once

#include "hal/Device.hpp"
#include "hal/types.hpp"

#include <osal/Timeout.hpp>

#include <system_error>

namespace hal::storage {

/// Represents a single EEPROM device. All operations will be limited to the given instance of this class.
class IEeprom : public Device {
public:
    /// Constructor.
    /// @param size                 Size of the physical EEPROM storage.
    /// @param pageSize             Page size of this EEPROM device.
    IEeprom(std::size_t size, std::size_t pageSize);

    /// Returns the size of the physical EEPROM storage in bytes.
    /// @return Size of the physical EEPROM storage in bytes.
    [[nodiscard]] std::size_t getSize() const { return m_size; }

    /// Returns the size of the page of the EEPROM device in bytes.
    /// @return Size of the page of the EEPROM storage in bytes.
    [[nodiscard]] std::size_t getPageSize() const { return m_pageSize; }

    /// Stores given vector of bytes in the current EEPROM device at the given location.
    /// @param address              Location address, where the data should be stored.
    /// @param bytes                Vector of raw bytes to be stored.
    /// @param timeout              Maximal time to wait for the data.
    /// @return Error code of the operation.
    /// @note This method will block until all data has been transferred to the driver.
    ///       It is up to the driver to decide if the data will be buffered (queued) or stored immediately.
    std::error_code write(std::uint32_t address, const BytesVector& bytes, osal::Timeout timeout);

    /// Stores given memory block of bytes in the current EEPROM device at the given location.
    /// @param address              Location address, where the data should be stored.
    /// @param bytes                Memory block of raw bytes to be stored.
    /// @param size                 Size of the memory block to be stored.
    /// @param timeout              Maximal time to wait for the data.
    /// @return Error code of the operation.
    /// @note This method will block until all data has been transferred to the driver.
    ///       It is up to the driver to decide if the data will be buffered (queued) or stored immediately.
    std::error_code write(std::uint32_t address, const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout);

    /// Reads the demanded number of bytes from the current EEPROM device.
    /// @param address              Location address, from where the data should be read.
    /// @param bytes                Vector where the read data will be placed by this method.
    /// @param size                 Number of bytes to be read from the current EEPROM device.
    /// @param timeout              Maximal time to wait for the data.
    /// @return Error code of the operation.
    /// @note This method does not assume, that the output vector has the proper capacity. It will be
    ///       automatically expanded, if needed, by the container itself. Size of the vector after call
    ///       to this method will indicate the actual number of read bytes.
    std::error_code read(std::size_t address, BytesVector& bytes, std::size_t size, osal::Timeout timeout);

    /// Reads the demanded number of bytes from the current EEPROM device.
    /// @param address              Location address, from where the data should be read.
    /// @param bytes                Memory block where the read data will be placed by this method.
    /// @param size                 Number of bytes to be read from the current EEPROM device.
    /// @param timeout              Maximal time to wait for the data.
    /// @param actualReadSize       Actual number of read bytes.
    /// @return Error code of the operation.
    /// @note This method assumes, that the output memory block has the proper capacity. After call to this
    ///       method the 'actualReadSize' parameter will indicate the actual number of received bytes.
    ///       It is also assumed, that output memory block is empty.
    std::error_code read(std::size_t address,
                         std::uint8_t* bytes,
                         std::size_t size,
                         osal::Timeout timeout,
                         std::size_t& actualReadSize);

private:
    /// Driver specific implementation of storing the memory block of bytes.
    /// @param address              Location address, where the data should be stored.
    /// @param bytes                Bytes to be stored.
    /// @param size                 Size of the memory block to be stored.
    /// @param timeout              Maximal time to wait for the data.
    /// @return Error code of the operation.
    virtual std::error_code
    drvWrite(std::size_t address, const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout)
        = 0;

    /// Driver specific implementation of the method that reads demanded number of bytes.
    /// @param address              Location address, from where the data should be read.
    /// @param bytes                Memory block where the read data will be placed by this method.
    /// @param size                 Number of bytes to be read from the current EEPROM driver.
    /// @param timeout              Maximal time to wait for the data.
    /// @param actualReadSize       Actual number of received bytes.
    /// @return Error code of the operation.
    virtual std::error_code drvRead(std::size_t address,
                                    std::uint8_t* bytes,
                                    std::size_t size,
                                    osal::Timeout timeout,
                                    std::size_t& actualReadSize)
        = 0;

private:
    std::size_t m_size;
    std::size_t m_pageSize;
};

} // namespace hal::storage
