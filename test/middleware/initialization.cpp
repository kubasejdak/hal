/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2023, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include <catch2/catch.hpp>

TEST_CASE("1.1. Typical hardware init flow", "[unit][middleware]")
{
    auto error = hal::Hardware::init();
    REQUIRE(!error);

    error = hal::Hardware::attach();
    REQUIRE(!error);

    error = hal::Hardware::detach();
    REQUIRE(!error);

    error = hal::Hardware::destroy();
    REQUIRE(!error);
}

TEST_CASE("1.2. Typical hardware init flow with ScopedHardware", "[unit][middleware]")
{
    hal::ScopedHardware hardware;
    REQUIRE(hardware.initialized());
}

TEST_CASE("1.3. Wrong Hardware states", "[unit][middleware]")
{
    SECTION("1.3.1. Hardware not initialized")
    {
        auto error = hal::Hardware::attach();
        REQUIRE(error == hal::Error::eWrongState);

        error = hal::Hardware::detach();
        REQUIRE(error == hal::Error::eWrongState);

        error = hal::Hardware::destroy();
        REQUIRE(error == hal::Error::eWrongState);

        error = hal::Hardware::init();
        REQUIRE(!error);

        error = hal::Hardware::attach();
        REQUIRE(!error);

        error = hal::Hardware::detach();
        REQUIRE(!error);

        error = hal::Hardware::destroy();
        REQUIRE(!error);
    }

    SECTION("1.3.2. Hardware not attached")
    {
        auto error = hal::Hardware::init();
        REQUIRE(!error);

        error = hal::Hardware::init();
        REQUIRE(error == hal::Error::eWrongState);

        error = hal::Hardware::detach();
        REQUIRE(error == hal::Error::eWrongState);

        error = hal::Hardware::attach();
        REQUIRE(!error);

        error = hal::Hardware::detach();
        REQUIRE(!error);

        error = hal::Hardware::destroy();
        REQUIRE(!error);
    }

    SECTION("1.3.3. Hardware not detached")
    {
        auto error = hal::Hardware::init();
        REQUIRE(!error);

        error = hal::Hardware::attach();
        REQUIRE(!error);

        error = hal::Hardware::destroy();
        REQUIRE(error == hal::Error::eWrongState);

        error = hal::Hardware::detach();
        REQUIRE(!error);

        error = hal::Hardware::destroy();
        REQUIRE(!error);
    }
}
