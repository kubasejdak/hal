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

#include "hal/time/IRtc.hpp"

#include "hal/Error.hpp"

namespace hal::time {

/// Checks if the given std::tm object is valid in terms of allowed field values.
/// @param tm               Time object to be checked.
/// @return Flag indicating if the given std::tm object is valid in terms of allowed field values.
/// @retval true            Given time object is valid.
/// @retval false           Given time object is invalid.
/// @note This function exploits the fact, that std::mktime converts even an invalid std::tm into the std::time_t.
///       This allows to check, if after a double conversion we still have the same value.
///       This function may modify input tm object, by filling tm_wday and tm_yday fields based on all other fields.
static bool isValidTime(std::tm& tm)
{
    std::tm toConvert = tm;
    auto time = std::mktime(&toConvert);
    if (time == static_cast<std::time_t>(-1))
        return false;

    std::tm converted{};
    if (gmtime_r(&time, &converted) == nullptr)
        return false;

    tm.tm_wday = toConvert.tm_wday;
    tm.tm_yday = toConvert.tm_yday;
    tm.tm_isdst = toConvert.tm_isdst;

    return (std::memcmp(&tm, &converted, sizeof(std::tm)) == 0);
}

IRtc::IRtc()
    : Device(SharingPolicy::eShared)
{}

std::error_code IRtc::getTime(std::tm& tm)
{
    if (auto error = drvGetTime(tm))
        return error;

    return isValidTime(tm) ? Error::eOk : Error::eHardwareError;
}

std::error_code IRtc::getTime(std::time_t& time)
{
    std::tm tm{};
    if (auto error = getTime(tm))
        return error;

    auto result = std::mktime(&tm);
    if (result == static_cast<std::time_t>(-1))
        return Error::eHardwareError;

    time = result;
    return Error::eOk;
}

std::error_code IRtc::setTime(const std::tm& tm)
{
    std::tm toSet = tm;
    if (!isValidTime(toSet))
        return Error::eInvalidArgument;

    auto error = drvSetTime(toSet);
    m_initialized = static_cast<bool>(!error);
    return error;
}

std::error_code IRtc::setTime(const std::time_t& time)
{
    std::tm tm{};
    if (gmtime_r(&time, &tm) == nullptr)
        return Error::eInvalidArgument;

    return setTime(tm);
}

} // namespace hal::time