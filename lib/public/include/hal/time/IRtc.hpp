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

#include <cstring>
#include <ctime>
#include <system_error>

namespace hal::time {

/// Represents a single RTC device. All operations will be limited to the given instance of this class.
/// @note On some platforms (e.g. Linux) this may or may not refer to the physical device.
class IRtc : public Device {
public:
    /// Default constructor.
    IRtc();

    /// Checks if the RTC clock has been properly initialized.
    /// @return Flag indicating if the RTC clock has been properly initialized.
    /// @retval true            RTC clock has been initialized.
    /// @retval false           RTC clock has not been initialized.
    [[nodiscard]] bool isInitialized() const { return m_initialized; }

    /// Returns the current local time in a form of std::tm object.
    /// @param tm               Output object where the current local time will be placed.
    /// @return Error code of the operation.
    std::error_code getTime(std::tm& tm);

    /// Returns the current local time in a form of std::time_t object.
    /// @param tm               Output object where the current local time will be placed.
    /// @return Error code of the operation.
    std::error_code getTime(std::time_t& time);

    /// Sets the current local time in a form of std::tm object.
    /// @param tm               Time to be set. Does not require tm_wday and tm_yday fields to be set.
    /// @return Error code of the operation.
    /// @note This function may internally change actual date & time by filling some of the missing fields
    ///       and adjusting invalid values to the correct ones.
    /// @see isValidTime()
    std::error_code setTime(const std::tm& tm);

    /// Sets the current local time in a form of std::time_t object.
    /// @param tm               Time to be set.
    /// @return Error code of the operation.
    /// @note This function may internally change actual date & time by filling some of the missing fields
    ///       and adjusting invalid values to the correct ones.
    /// @see isValidTime()
    std::error_code setTime(const std::time_t& time);

private:
    /// Driver specific implementation of getting the current local time in a form of std::tm object.
    /// @param tm               Output object where the current local time will be placed.
    /// @return Error code of the operation.
    virtual std::error_code drvGetTime(std::tm& tm) = 0;

    /// Driver specific implementation of setting the current local time in a form of std::tm object.
    /// @param tm               Time to be set.
    /// @return Error code of the operation.
    virtual std::error_code drvSetTime(const std::tm& tm) = 0;

private:
    bool m_initialized{};
};

} // namespace hal::time
