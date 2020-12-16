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

#include "M41T82.hpp"

#include "hal/Error.hpp"
#include "hal/i2c/ScopedI2c.hpp"
#include "hal/utils/logger.hpp"

#include <cassert>
#include <utility>

namespace hal::time {

M41T82::M41T82(std::shared_ptr<i2c::II2c> i2c, std::uint16_t address)
    : m_i2c(std::move(i2c))
    , m_cAddress(address)
{
    if (!i2c::verifyAddress(i2c::AddressingMode::e7bit, m_cAddress)) {
        M41T82Logger::critical("Failed to create M41T82 RTC: bad parameters");
        assert(false);
        return;
    }

    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        M41T82Logger::critical("Failed to create M41T82 RTC: timeout when locking I2C bus (timeout=default)");
        assert(false);
        return;
    }

    m_i2c->open();

    M41T82Logger::info("Created M41T82 RTC with the following parameters:");
    M41T82Logger::info("  address      : {:#04x}", m_cAddress);
}

M41T82::~M41T82()
{
    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        M41T82Logger::error("Failed to close I2C device: timeout when locking I2C bus (timeout=default)");
        return;
    }

    m_i2c->close();
}

std::error_code M41T82::drvGetTime(std::tm& tm)
{
    (void) tm;
    return Error::eOk;
}

std::error_code M41T82::drvSetTime(const std::tm& tm)
{
    (void) tm;
    return Error::eOk;
}

} // namespace hal::time
