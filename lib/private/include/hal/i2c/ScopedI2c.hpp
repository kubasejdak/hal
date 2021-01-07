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

#include "hal/Error.hpp"
#include "hal/i2c/II2c.hpp"

#include <osal/Timeout.hpp>

#include <memory>
#include <system_error>

namespace hal::i2c {

/// Represents the RAII object to acquire the I2C device in a valid manner. In constructor it automatically
/// locks the I2C bus. In destructor it automatically unlocks the I2C bus.
/// @note release() method should not be called directly, unless necessary. The power of RAII object lays in the fact,
///       that release() method is automatically called, when ScopedI2c is destroyed. This is handy, because in case of
///       any error client can just return from the function without worrying about unlocking the I2C.
class ScopedI2c {
public:
    /// Constructor.
    /// @param i2c              Reference to the I2C driver.
    /// @param timeout          Maximal time to wait for the operation.
    /// @note This constructor automatically locks the I2C bus.
    explicit ScopedI2c(const std::shared_ptr<i2c::II2c>& i2c, osal::Timeout timeout = osal::Timeout::infinity())
        : m_i2c(i2c)
    {
        acquire(timeout);
    }

    /// Copy constructor.
    /// @note This constructor is deleted, because ScopedI2c is not meant to be copy-constructed.
    ScopedI2c(const ScopedI2c&) = delete;

    /// Move constructor.
    /// @note This constructor is deleted, because ScopedI2c is not meant to be move-constructed.
    ScopedI2c(ScopedI2c&&) = delete;

    /// Copy assignment operator.
    /// @note This operator is deleted, because ScopedI2c is not meant to be copy-assigned.
    ScopedI2c& operator=(const ScopedI2c&) = delete;

    /// Move assignment operator.
    /// @note This operator is deleted, because ScopedI2c is not meant to be move-assigned.
    ScopedI2c& operator=(ScopedI2c&&) = delete;

    /// Destructor.
    /// @note This destructor automatically disables the chip select pin and unlocks the I2C bus.
    ~ScopedI2c() { release(); }

    /// Acquires the I2C bus.
    /// @param timeout          Maximal time to wait for the operation.
    /// @return Error code of the operation.
    /// @note This method automatically locks the I2C bus.
    std::error_code acquire(osal::Timeout timeout)
    {
        if (isAcquired())
            return Error::eWrongState;

        if (auto error = m_i2c->lock(timeout)) {
            release();
            return error;
        }

        m_locked = true;
        return Error::eOk;
    }

    /// Releases the I2C bus.
    /// @return Error code of the operation.
    /// @note This method automatically unlocks the I2C bus.
    std::error_code release()
    {
        if (!isAcquired())
            return Error::eWrongState;

        if (m_locked) {
            if (auto error = m_i2c->unlock())
                return error;

            m_locked = false;
        }

        return Error::eOk;
    }

    /// Returns the flag indicating if the I2C has been acquired.
    /// @return Flag indicating if the I2C has been acquired.
    /// @retval true            I2C has been acquired.
    /// @retval false           I2C has not been acquired.
    [[nodiscard]] bool isAcquired() const { return m_locked; }

private:
    bool m_locked{};
    const std::shared_ptr<II2c>& m_i2c;
};

} // namespace hal::i2c
