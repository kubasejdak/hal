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

#include "hal/i2c/II2c.hpp"
#include "hal/time/IRtc.hpp"

#include <cstdint>
#include <memory>
#include <system_error>

namespace hal::time {

/// Represents M41T82 RTC driver with the IRtc interface.
class M41T82 : public IRtc {
public:
    /// Constructor.
    /// @param i2c              Reference to the I2C bus that should be used.
    /// @param address          Address of the device on the I2C bus.
    M41T82(std::shared_ptr<i2c::II2c> i2c, std::uint16_t address);

    /// Copy constructor.
    /// @note This constructor is deleted, because M41T82 is not meant to be copy-constructed.
    M41T82(const M41T82&) = delete;

    /// Copy constructor.
    /// @note This constructor is deleted, because M41T82 is not meant to be move-constructed.
    M41T82(M41T82&&) noexcept = delete;

    /// Destructor.
    ~M41T82() override;

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because M41T82 is not meant to be copy-assigned.
    M41T82& operator=(const M41T82&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because M41T82 is not meant to be move-assigned.
    M41T82& operator=(M41T82&&) noexcept = delete;

private:
    /// @see IRtc::drvGetTime().
    std::error_code drvGetTime(std::tm& tm) override;

    /// @see IRtc::drvSetTime().
    std::error_code drvSetTime(const std::tm& tm) override;

private:
    std::shared_ptr<i2c::II2c> m_i2c;
    const std::uint16_t m_cAddress;
};

} // namespace hal::time
