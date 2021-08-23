/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
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

#include "hal/i2c/II2c.hpp"
#include "hal/storage/IEeprom.hpp"

#include <osal/Timeout.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <system_error>

namespace hal::storage {

/// Represents the generic driver of the IEeprom interface.
class GenericEeprom : public IEeprom {
public:
    /// Constructor.
    /// @param i2c              Reference to the I2C bus that should be used.
    /// @param address          Address of the device on the I2C bus.
    /// @param addressingMode   Addressing mode used by this device.
    /// @param size             Size of the physical EEPROM storage.
    /// @param pageSize         Page size of this EEPROM device.
    /// @param writeDelay       Minimal time between write and read operations.
    GenericEeprom(std::shared_ptr<i2c::II2c> i2c,
                  std::uint16_t address,
                  i2c::AddressingMode addressingMode,
                  std::size_t size,
                  std::size_t pageSize,
                  std::chrono::milliseconds writeDelay);

    /// Copy constructor.
    /// @note This constructor is deleted, because GenericEeprom is not meant to be copy-constructed.
    GenericEeprom(const GenericEeprom&) = delete;

    /// Move constructor.
    /// @note This constructor is deleted, because GenericEeprom is not meant to be move-constructed.
    GenericEeprom(GenericEeprom&&) noexcept = delete;

    /// Destructor.
    ~GenericEeprom() override;

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because GenericEeprom is not meant to be copy-assigned.
    GenericEeprom& operator=(const GenericEeprom&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because GenericEeprom is not meant to be move-assigned.
    GenericEeprom& operator=(GenericEeprom&&) noexcept = delete;

    /// Checks if driver has been properly initialized.
    /// @return Flag indicating if driver has been properly initialized.
    /// @retval true                Driver has been initialized.
    /// @retval false               Driver has not been initialized.
    [[nodiscard]] bool isInitialized() const { return m_initialized; }

private:
    /// @see IEeprom::drvWrite().
    std::error_code
    drvWrite(std::uint32_t address, const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout) override;

    /// @see IEeprom::drvRead().
    std::error_code drvRead(std::uint32_t address,
                            std::uint8_t* bytes,
                            std::size_t size,
                            osal::Timeout timeout,
                            std::size_t& actualReadSize) override;

    /// Writes at most one page of data into EEPROM device.
    /// @param address              Location address, where the data should be stored.
    /// @param bytes                Byte to be stored.
    /// @param size                 Size of the memory block to be stored.
    /// @param timeout              Maximal time to wait for the bus.
    /// @return Error code of the operation.
    std::error_code
    writePage(std::uint32_t address, const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout);

private:
    std::shared_ptr<i2c::II2c> m_i2c;
    std::uint16_t m_address;
    osal::Timeout m_writeDelay;
    bool m_initialized;
};

} // namespace hal::storage
