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

    auto error = rtc->setTime(tmSet);
    REQUIRE(!error);

    bool initialized = rtc->isInitialized();
    REQUIRE(initialized);

    std::tm tmGet{};
    error = rtc->getTime(tmGet);
    REQUIRE(!error);

    auto timeSet = std::mktime(&tmSet);
    auto timeGet = std::mktime(&tmGet);
    constexpr int cAllowedDiffSec = 3;
    REQUIRE(timeGet >= timeSet);
    REQUIRE(timeGet <= (timeSet + cAllowedDiffSec));
}

TEST_CASE("2. Set RTC with std::tm and get with std::time_t", "[unit][rtc]")
{
    hal::ScopedHardware hardware;

    auto rtc = hal::getScopedDevice<hal::time::IRtc>(hal::device_id::eM41T82Rtc);
    std::tm tmSet{};

    SECTION("2.1. 18:13:25 04.05.2013")
    {
        tmSet.tm_hour = 18;  // NOLINT
        tmSet.tm_min = 13;   // NOLINT
        tmSet.tm_sec = 25;   // NOLINT
        tmSet.tm_mday = 4;   // NOLINT
        tmSet.tm_mon = 4;    // NOLINT
        tmSet.tm_year = 113; // NOLINT
    }

    SECTION("2.2. 07:14:03 4.11.2011")
    {
        tmSet.tm_hour = 7;   // NOLINT
        tmSet.tm_min = 14;   // NOLINT
        tmSet.tm_sec = 3;    // NOLINT
        tmSet.tm_mday = 4;   // NOLINT
        tmSet.tm_mon = 10;   // NOLINT
        tmSet.tm_year = 111; // NOLINT
    }

    SECTION("2.3. 12:34:14 17.08.2001")
    {
        tmSet.tm_hour = 12;  // NOLINT
        tmSet.tm_min = 34;   // NOLINT
        tmSet.tm_sec = 14;   // NOLINT
        tmSet.tm_mday = 17;  // NOLINT
        tmSet.tm_mon = 7;    // NOLINT
        tmSet.tm_year = 101; // NOLINT
    }

    auto error = rtc->setTime(tmSet);
    REQUIRE(!error);

    bool initialized = rtc->isInitialized();
    REQUIRE(initialized);

    std::time_t timeGet{};
    error = rtc->getTime(timeGet);
    REQUIRE(!error);

    auto timeSet = std::mktime(&tmSet);
    constexpr int cAllowedDiffSec = 3;
    REQUIRE(timeGet >= timeSet);
    REQUIRE(timeGet <= (timeSet + cAllowedDiffSec));
}

TEST_CASE("3. Set RTC with std::time_t and get with std::tm", "[unit][rtc]")
{
    hal::ScopedHardware hardware;

    auto rtc = hal::getScopedDevice<hal::time::IRtc>(hal::device_id::eM41T82Rtc);
    std::tm tmSet{};

    SECTION("3.1. 17:08:11 10.02.2017")
    {
        tmSet.tm_hour = 17;  // NOLINT
        tmSet.tm_min = 8;    // NOLINT
        tmSet.tm_sec = 11;   // NOLINT
        tmSet.tm_mday = 10;  // NOLINT
        tmSet.tm_mon = 1;    // NOLINT
        tmSet.tm_year = 117; // NOLINT
    }

    SECTION("3.2. 08:24:33 18.04.2019")
    {
        tmSet.tm_hour = 8;   // NOLINT
        tmSet.tm_min = 24;   // NOLINT
        tmSet.tm_sec = 33;   // NOLINT
        tmSet.tm_mday = 18;  // NOLINT
        tmSet.tm_mon = 3;    // NOLINT
        tmSet.tm_year = 119; // NOLINT
    }

    SECTION("3.3. 03:14:01 29.05.2004")
    {
        tmSet.tm_hour = 3;   // NOLINT
        tmSet.tm_min = 14;   // NOLINT
        tmSet.tm_sec = 1;    // NOLINT
        tmSet.tm_mday = 29;  // NOLINT
        tmSet.tm_mon = 4;    // NOLINT
        tmSet.tm_year = 104; // NOLINT
    }

    auto timeSet = std::mktime(&tmSet);
    auto error = rtc->setTime(timeSet);
    REQUIRE(!error);

    bool initialized = rtc->isInitialized();
    REQUIRE(initialized);

    std::tm tmGet{};
    error = rtc->getTime(tmGet);
    REQUIRE(!error);

    auto timeGet = std::mktime(&tmGet);
    constexpr int cAllowedDiffSec = 3;
    REQUIRE(timeGet >= timeSet);
    REQUIRE(timeGet <= (timeSet + cAllowedDiffSec));
}

TEST_CASE("4. Set & get RTC with std::time_t", "[unit][rtc]")
{
    hal::ScopedHardware hardware;

    auto rtc = hal::getScopedDevice<hal::time::IRtc>(hal::device_id::eM41T82Rtc);
    std::tm tmSet{};

    SECTION("4.1. 23:59:59 28.02.2019")
    {
        tmSet.tm_hour = 23;  // NOLINT
        tmSet.tm_min = 59;   // NOLINT
        tmSet.tm_sec = 59;   // NOLINT
        tmSet.tm_mday = 28;  // NOLINT
        tmSet.tm_mon = 1;    // NOLINT
        tmSet.tm_year = 119; // NOLINT
    }

    SECTION("4.2. 23:59:59 31.12.2020")
    {
        tmSet.tm_hour = 23;  // NOLINT
        tmSet.tm_min = 59;   // NOLINT
        tmSet.tm_sec = 59;   // NOLINT
        tmSet.tm_mday = 31;  // NOLINT
        tmSet.tm_mon = 11;   // NOLINT
        tmSet.tm_year = 120; // NOLINT
    }

    SECTION("4.3. 23:59:59 30.06.2000")
    {
        tmSet.tm_hour = 23;  // NOLINT
        tmSet.tm_min = 59;   // NOLINT
        tmSet.tm_sec = 59;   // NOLINT
        tmSet.tm_mday = 30;  // NOLINT
        tmSet.tm_mon = 5;    // NOLINT
        tmSet.tm_year = 100; // NOLINT
    }

    auto timeSet = std::mktime(&tmSet);
    auto error = rtc->setTime(timeSet);
    REQUIRE(!error);

    bool initialized = rtc->isInitialized();
    REQUIRE(initialized);

    std::time_t timeGet{};
    error = rtc->getTime(timeGet);
    REQUIRE(!error);

    constexpr int cAllowedDiffSec = 3;
    REQUIRE(timeGet >= timeSet);
    REQUIRE(timeGet <= (timeSet + cAllowedDiffSec));
}

TEST_CASE("5. Invalid RTC arguments", "[unit][rtc]")
{
    hal::ScopedHardware hardware;

    auto rtc = hal::getScopedDevice<hal::time::IRtc>(hal::device_id::eM41T82Rtc);

    SECTION("5.1. Invalid std::tm value")
    {
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

    SECTION("5.2. Invalid std::time_t value")
    {
        std::time_t time = -1;

        auto error = rtc->setTime(time);
        REQUIRE(error == hal::Error::eInvalidArgument);
    }
}
