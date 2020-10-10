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

#include "hal/Error.hpp"
#include "hal/gpio/IPinOutput.hpp"
#include "hal/spi/ISpi.hpp"

#include <osal/Timeout.hpp>

#include <system_error>

namespace hal::spi {

/// Represents the RAII object to acquire the SPI device in a valid manner. In constructor it automatically
/// locks the SPI bus, sets the defined parameters and enables the chip select pin. In destructor this process is done
/// in reverse.
/// @note Release() method should not be called directly, unless necessary. The power of RAII object lays in the fact,
///       that Release() method is automatically called, when ScopedSpi is destroyed. This is handy, because in case of
///       any error client can just return from the function without worrying about unlocking the SPI.
class ScopedSpi {
public:
    /// Constructor.
    /// @param spi              Reference to the SPI driver.
    /// @param params           Parameters to be set in the SPI driver.
    /// @param chipSelect       Reference to the GPIO pin output driver representing the chip select.
    /// @param timeoutMs        Maximal time to wait for the operation.
    /// @note This constructor automatically locks the SPI bus, sets the defined parameters and enables
    ///       the chip select pin.
    ScopedSpi(const std::shared_ptr<spi::ISpi>& spi,
              const SpiParams& params,
              const std::shared_ptr<hal::gpio::IPinOutput>& chipSelect,
              osal::Timeout timeout = osal::Timeout::infinity())
        : m_spi(spi)
        , m_params(params)
        , m_chipSelect(chipSelect)
    {
        acquire(timeout);
    }

    /// Copy constructor.
    /// @note This constructor is deleted, because ScopedSpi is not meant to be copy-constructed.
    ScopedSpi(const ScopedSpi&) = delete;

    /// Move constructor.
    /// @note This constructor is deleted, because ScopedSpi is not meant to be move-constructed.
    ScopedSpi(ScopedSpi&&) = delete;

    /// Assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because ScopedSpi is not meant to be move-assigned.
    ScopedSpi& operator=(const ScopedSpi&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because ScopedSpi is not meant to be move-assigned.
    ScopedSpi& operator=(ScopedSpi&&) = delete;

    /// Destructor.
    /// @note This destructor automatically disables the chip select pin and unlocks the SPI bus.
    ~ScopedSpi() { release(); }

    /// Acquires the SPI bus.
    /// @param timeout          Maximal time to wait for the operation.
    /// @return Error code of the operation.
    /// @note This method automatically locks the SPI bus, sets the defined parameters and enables
    ///       the chip select pin.
    std::error_code acquire(osal::Timeout timeout)
    {
        if (isAcquired())
            return Error::eWrongState;

        if (auto error = m_spi->lock(timeout)) {
            release();
            return error;
        }

        m_locked = true;

        if (auto error = m_spi->setParams(m_params)) {
            release();
            return error;
        }

        if (auto error = chipSelectEnable()) {
            release();
            return error;
        }

        return Error::eOk;
    }

    /// Releases the SPI bus.
    /// @return Error code of the operation.
    /// @note This method automatically disables the chip select pin and unlocks the SPI bus.
    std::error_code release()
    {
        if (!isAcquired())
            return Error::eWrongState;

        if (m_selected) {
            if (auto error = chipSelectDisable())
                return error;
        }

        if (m_locked) {
            if (auto error = m_spi->unlock())
                return error;

            m_locked = false;
        }

        return Error::eOk;
    }

    /// Returns the flag indicating if the SPI has been acquired.
    /// @return Flag indicating if the SPI has been acquired.
    /// @retval true            SPI has been acquired.
    /// @retval false           SPI has not been acquired.
    bool isAcquired() const { return m_locked; }

    /// Returns the flag indicating if the SPI chip select is enabled.
    /// @return Flag indicating if the SPI chip select has been enabled.
    /// @retval true            SPI chip select has been enabled.
    /// @retval false           SPI chip select has been disabled.
    bool isChipSelectEnabled() const { return m_selected; }

    /// Enables the chip select manually.
    /// @return Error code of the operation.
    std::error_code chipSelectEnable()
    {
        if (!isAcquired())
            return Error::eWrongState;

        if (isChipSelectEnabled())
            return Error::eWrongState;

        if (auto error = m_chipSelect->on())
            return error;

        m_selected = true;
        return Error::eOk;
    }

    /// Disables the chip select manually.
    /// @return Error code of the operation.
    std::error_code chipSelectDisable()
    {
        if (!isAcquired())
            return Error::eWrongState;

        if (!isChipSelectEnabled())
            return Error::eWrongState;

        if (auto error = m_chipSelect->off())
            return error;

        m_selected = false;
        return Error::eOk;
    }

private:
    bool m_locked{};
    bool m_selected{};
    const std::shared_ptr<ISpi>& m_spi;
    const SpiParams& m_params;
    const std::shared_ptr<gpio::IPinOutput>& m_chipSelect;
};

} // namespace hal::spi
