/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2020-2023, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include <hal/Error.hpp>
#include <hal/Hardware.hpp>
#include <hal/factory.hpp>
#include <hal/time/IRtc.hpp>

#include <catch2/catch.hpp>

#include <ctime>

TEST_CASE("1. Set & get RTC time with std::tm", "[unit][rtc]")
{
    hal::ScopedHardware hardware;

    auto rtc = hal::getScopedDevice<hal::time::IRtc>(hal::device_id::eM41T82Rtc);
    std::tm tmSet{};

    SECTION("1.1. 13:46:05 26.12.2020")
    {
        tmSet.tm_hour = 13;  // NOLINT
        tmSet.tm_min = 46;   // NOLINT
        tmSet.tm_sec = 5;    // NOLINT
        tmSet.tm_mday = 26;  // NOLINT
        tmSet.tm_mon = 11;   // NOLINT
        tmSet.tm_year = 120; // NOLINT
    }

    SECTION("1.2. 23:59:17 01.01.2020")
    {
        tmSet.tm_hour = 23;  // NOLINT
        tmSet.tm_min = 59;   // NOLINT
        tmSet.tm_sec = 17;   // NOLINT
        tmSet.tm_mday = 1;   // NOLINT
        tmSet.tm_mon = 0;    // NOLINT
        tmSet.tm_year = 120; // NOLINT
    }

    SECTION("1.3. 20:20:00 08.08.2008")
    {
        tmSet.tm_hour = 20;  // NOLINT
        tmSet.tm_min = 20;   // NOLINT
        tmSet.tm_sec = 00;   // NOLINT
        tmSet.tm_mday = 8;   // NOLINT
        tmSet.tm_mon = 7;    // NOLINT
        tmSet.tm_year = 108; // NOLINT
    }

    auto setError = rtc->setTime(tmSet);
    REQUIRE(!setError);

    bool initialized = rtc->isInitialized();
    REQUIRE(initialized);

    auto [tmGet, getError] = rtc->getTime();
    REQUIRE(!getError);

    auto timeSet = std::mktime(&tmSet);
    auto timeGet = std::mktime(&tmGet.value());
    constexpr int cAllowedDiffSec = 3;
    REQUIRE(timeGet >= timeSet);
    REQUIRE(timeGet <= (timeSet + cAllowedDiffSec));
}

TEST_CASE("2. Invalid RTC arguments", "[unit][rtc]")
{
    hal::ScopedHardware hardware;

    auto rtc = hal::getScopedDevice<hal::time::IRtc>(hal::device_id::eM41T82Rtc);

    std::tm tm{};
    tm.tm_hour = -1; // NOLINT
    tm.tm_min = -1;  // NOLINT
    tm.tm_sec = -1;  // NOLINT
    tm.tm_mday = -1; // NOLINT
    tm.tm_mon = -1;  // NOLINT
    tm.tm_year = -1; // NOLINT

    auto error = rtc->setTime(tm);
    REQUIRE(error == hal::Error::eInvalidArgument);
}
