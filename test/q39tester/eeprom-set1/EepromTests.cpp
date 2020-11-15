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

#include <hal/Error.hpp>
#include <hal/Hardware.hpp>
#include <hal/factory.hpp>
#include <hal/storage/IEeprom.hpp>

#include <catch2/catch.hpp>

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

static void prepareRandomData(std::vector<std::uint8_t>& data)
{
    std::random_device randomDevice;
    std::mt19937 generator(randomDevice());
    std::generate(data.begin(), data.end(), std::ref(generator));
}

TEST_CASE("Basic EEPROM operations", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    SECTION("Get size")
    {
        constexpr std::uint32_t cSize = 256 * 1024;
        auto eepromSize = eeprom->getSize();
        REQUIRE(eepromSize == cSize);
    }

    SECTION("Get page size")
    {
        constexpr std::uint32_t cSize = 256;
        auto pageSize = eeprom->getPageSize();
        REQUIRE(pageSize == cSize);
    }

    hal::returnDevice(eeprom);
}

TEST_CASE("Write whole first page of EEPROM", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    constexpr auto cTimeout = 100ms;
    const size_t cPageSize = eeprom->getPageSize();
    std::vector<std::uint8_t> writeBlock(cPageSize);
    prepareRandomData(writeBlock);

    // Write whole first page.
    std::uint16_t address = 0;
    auto error = eeprom->write(address, writeBlock, cTimeout);
    REQUIRE(!error);

    // Read and verify whole first page.
    std::vector<std::uint8_t> readBlock;
    error = eeprom->read(address, readBlock, writeBlock.size(), cTimeout);
    REQUIRE(!error);
    REQUIRE(readBlock == writeBlock);

    hal::returnDevice(eeprom);
}

TEST_CASE("Write whole last page of EEPROM", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    constexpr auto cTimeout = 100ms;
    const std::uint32_t cSize = eeprom->getSize();
    const size_t cPageSize = eeprom->getPageSize();
    std::vector<std::uint8_t> writeBlock(cPageSize);
    prepareRandomData(writeBlock);

    // Write whole last page.
    const std::uint16_t cAddress = cSize - cPageSize;
    auto error = eeprom->write(cAddress, writeBlock, cTimeout);
    REQUIRE(!error);

    // Read and verify whole last page.
    std::vector<std::uint8_t> readBlock;
    error = eeprom->read(cAddress, readBlock, writeBlock.size(), cTimeout);
    REQUIRE(!error);
    REQUIRE(readBlock == writeBlock);

    hal::returnDevice(eeprom);
}

TEST_CASE("Write page at page intersection in EEPROM", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    constexpr auto cTimeout = 100ms;
    const size_t cPageSize = eeprom->getPageSize();
    constexpr std::uint8_t cPattern1 = 1;
    constexpr std::uint8_t cPattern2 = 2;
    constexpr std::uint8_t cPattern3 = 3;
    const std::vector<std::uint8_t> cWriteBlock1(cPageSize, cPattern1);
    const std::vector<std::uint8_t> cWriteBlock2(cPageSize, cPattern2);
    const std::vector<std::uint8_t> cWriteBlock3(cPageSize, cPattern3);

    // Fill first and second page with pattern.
    std::uint16_t address = 0;
    auto error = eeprom->write(address, cWriteBlock1, cTimeout);
    REQUIRE(!error);

    error = eeprom->write(address + cPageSize, cWriteBlock2, cTimeout);
    REQUIRE(!error);

    // Fill block with size equal to the page size with pattern at the intersection of first and second page.
    error = eeprom->write(address + (cPageSize / 2), cWriteBlock3, cTimeout);
    REQUIRE(!error);

    std::vector<std::uint8_t> readBlock;
    error = eeprom->read(address, readBlock, 2 * cPageSize, cTimeout);
    REQUIRE(!error);

    // Verify first pattern.
    for (std::size_t i = 0; i < (cPageSize / 2); ++i)
        REQUIRE(readBlock[i] == cPattern1);

    // Verify third pattern.
    for (std::size_t i = cPageSize / 2; i < cPageSize + (cPageSize / 2); ++i)
        REQUIRE(readBlock[i] == cPattern3);

    // Verify second pattern.
    for (std::size_t i = cPageSize + (cPageSize / 2); i < (2 * cPageSize); ++i)
        REQUIRE(readBlock[i] == cPattern2);

    hal::returnDevice(eeprom);
}

TEST_CASE("Write every second byte separately in EEPROM", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    constexpr auto cTimeout = 100ms;
    const std::uint32_t cSize = eeprom->getSize();
    const size_t cPageSize = eeprom->getPageSize();
    constexpr std::uint8_t cPattern = 6;

    const std::vector<std::uint8_t> cWriteBlock(cPageSize, cPattern);

    // Clear page.
    std::uint16_t address = cSize / 2;
    auto error = eeprom->write(address, cWriteBlock, cTimeout);
    REQUIRE(!error);

    // Fill page with pattern (every second byte is a next integral number).
    for (std::uint16_t i = 0; i < cPageSize; i += 2) {
        auto value = static_cast<std::uint8_t>(i / 2);
        error = eeprom->write(address + i, &value, sizeof(value), cTimeout);
        REQUIRE(!error);
    }

    // Read and verify page.
    for (std::uint16_t i = 0; i < cPageSize; ++i) {
        auto expectedValue = static_cast<std::uint8_t>((i % 2) != 0 ? cPattern : (i / 2));
        std::uint8_t readValue{};
        std::size_t actualReadSize{};
        error = eeprom->read(address + i, &readValue, sizeof(readValue), cTimeout, actualReadSize);
        REQUIRE(!error);
        REQUIRE(actualReadSize == sizeof(readValue));
        REQUIRE(readValue == expectedValue);
    }

    hal::returnDevice(eeprom);
}

TEST_CASE("Read bytes when address exceeds EEPROM size", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    // Read bytes starting from one page after the end of EEPROM.
    const std::uint16_t cAddress = eeprom->getSize() + eeprom->getPageSize();
    std::vector<std::uint8_t> readBlock;
    constexpr std::size_t cReadSize = 13;
    constexpr auto cTimeout = 100ms;
    auto error = eeprom->read(cAddress, readBlock, cReadSize, cTimeout);
    REQUIRE(error == hal::Error::eInvalidArgument);
    REQUIRE(readBlock.empty());

    hal::returnDevice(eeprom);
}

TEST_CASE("Read bytes when address plus size exceeds EEPROM size", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    // Read bytes from address, which summed with the readBlock size gives one byte after the of EEPROM.
    constexpr std::size_t cReadSize = 13;
    const std::uint16_t cAddress = eeprom->getSize() - (cReadSize - 1);
    std::vector<std::uint8_t> readBlock;
    constexpr auto cTimeout = 100ms;
    auto error = eeprom->read(cAddress, readBlock, cReadSize, cTimeout);
    REQUIRE(error == hal::Error::eInvalidArgument);
    REQUIRE(readBlock.empty());

    hal::returnDevice(eeprom);
}

TEST_CASE("Write bytes when address exceeds EEPROM size", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    // Write bytes starting from one page after the end of EEPROM.
    const std::uint16_t cAddress = eeprom->getSize() + eeprom->getPageSize();
    constexpr std::size_t cWriteSize = 13;
    constexpr std::uint8_t cPattern = 0xa;
    const std::vector<std::uint8_t> cWriteBlock(cWriteSize, cPattern);
    constexpr auto cTimeout = 100ms;
    auto error = eeprom->write(cAddress, cWriteBlock, cTimeout);
    REQUIRE(error == hal::Error::eInvalidArgument);

    hal::returnDevice(eeprom);
}

TEST_CASE("Write bytes when address plus size exceeds EEPROM size", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    // Write bytes from address, which summed with the readBlock size gives one byte after the of EEPROM.
    constexpr std::size_t cWriteSize = 13;
    constexpr std::uint8_t cPattern = 0xb;
    const std::vector<std::uint8_t> cWriteBlock(cWriteSize, cPattern);
    const std::uint16_t cAddress = eeprom->getSize() - (cWriteBlock.size() - 1);
    constexpr auto cTimeout = 100ms;
    auto error = eeprom->write(cAddress, cWriteBlock, cTimeout);
    REQUIRE(error == hal::Error::eInvalidArgument);

    hal::returnDevice(eeprom);
}
