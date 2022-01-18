/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2021-2022, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include <hal/Hardware.hpp>
#include <hal/factory.hpp>
#include <hal/test/TestDevice.hpp>

#include <catch2/catch.hpp>

TEST_CASE("3.1. Devices are automatically returned to HAL", "[unit][ScopedDevice]")
{
    hal::ScopedHardware hardware;

    {
        hal::ScopedDevice<hal::test::TestDevice> device1;

        SECTION("3.1.1. ScopedDevice created from std::shared_ptr")
        {
            device1 = hal::getDevice<hal::test::TestDevice>(hal::device_id::eTestDeviceSingle1);
        }

        SECTION("3.1.2. ScopedDevice created automatically")
        {
            device1 = hal::getScopedDevice<hal::test::TestDevice>(hal::device_id::eTestDeviceSingle1);
        }

        REQUIRE(device1);
        REQUIRE(device1.get());

        auto secondHandle = hal::getDevice<hal::test::TestDevice>(hal::device_id::eTestDeviceSingle1);
        REQUIRE(!secondHandle);
        REQUIRE(!secondHandle.get());
    }

    auto thirdHandle = hal::getDevice<hal::test::TestDevice>(hal::device_id::eTestDeviceSingle1);
    REQUIRE(thirdHandle);

    hal::returnDevice(thirdHandle);
}

TEST_CASE("3.2. ScopedDevices can be moved around", "[unit][ScopedDevice]")
{
    hal::ScopedHardware hardware;

    SECTION("3.2.1. ScopedDevice created with a move constructor.")
    {
        auto device1 = hal::getScopedDevice<hal::test::TestDevice>(hal::device_id::eTestDeviceSingle1);
        REQUIRE(device1);

        auto secondHandle = std::move(device1);
        REQUIRE(secondHandle);
        REQUIRE(!device1); // NOLINT
    }

    SECTION("3.2.2. ScopedDevice created with a move assignment operator.")
    {
        hal::ScopedDevice<hal::test::TestDevice> device1;
        device1 = hal::getScopedDevice<hal::test::TestDevice>(hal::device_id::eTestDeviceSingle1);
        REQUIRE(device1);
    }
}

TEST_CASE("3.3. ScopedDevice can call functions of the underlying objects", "[unit][ScopedDevice]")
{
    hal::ScopedHardware hardware;

    auto device1 = hal::getScopedDevice<hal::test::TestDevice>(hal::device_id::eTestDeviceSingle1);
    REQUIRE(device1);

    device1->testFunc();
}

TEST_CASE("3.4. ScopedDevice can be explicitly cleared", "[unit][ScopedDevice]")
{
    hal::ScopedHardware hardware;

    auto device1 = hal::getScopedDevice<hal::test::TestDevice>(hal::device_id::eTestDeviceSingle1);
    REQUIRE(device1);
    REQUIRE(device1.get());

    device1.reset();
    REQUIRE(!device1);
    REQUIRE(!device1.get());

    auto secondHandle = hal::getDevice<hal::test::TestDevice>(hal::device_id::eTestDeviceSingle1);
    REQUIRE(secondHandle);

    hal::returnDevice(secondHandle);
}
