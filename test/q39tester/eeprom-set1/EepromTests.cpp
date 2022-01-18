/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2020-2022, Kuba Sejdak <kuba.sejdak@gmail.com>
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

static void getRandomData(std::vector<std::uint8_t>& data)
{
    std::random_device randomDevice;
    std::mt19937 generator(randomDevice());
    std::generate(data.begin(), data.end(), std::ref(generator));
}

TEST_CASE("1. Basic EEPROM operations", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom1 = hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);
    auto eeprom2 = hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom2);

    SECTION("1.1. Get size")
    {
        constexpr std::uint32_t cSize = 256 * 1024;
        auto eepromSize = eeprom1->getSize();
        REQUIRE(eepromSize == cSize);

        eepromSize = eeprom2->getSize();
        REQUIRE(eepromSize == cSize);
    }

    SECTION("1.2. Get page size")
    {
        constexpr std::uint32_t cSize = 256;
        auto pageSize = eeprom1->getPageSize();
        REQUIRE(pageSize == cSize);

        pageSize = eeprom2->getPageSize();
        REQUIRE(pageSize == cSize);
    }
}

TEST_CASE("2. Write whole first page of EEPROM", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeproms = {hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1),
                    hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom2)};

    constexpr auto cTimeout = 100ms;
    const std::size_t cPageSize = eeproms.begin()->get()->getPageSize();
    std::uint32_t address{};

    SECTION("2.1. First quarter") { address = 0; }

    SECTION("2.2. Second quarter") { address = std::numeric_limits<std::uint16_t>::max() + 1; }

    SECTION("2.3. Third quarter") { address = 2 * (std::numeric_limits<std::uint16_t>::max() + 1); }

    SECTION("2.4. Fourth quarter") { address = 3 * (std::numeric_limits<std::uint16_t>::max() + 1); }

    for (const auto& eeprom : eeproms) {
        // Write whole first page.
        std::vector<std::uint8_t> writeBlock(cPageSize);
        getRandomData(writeBlock);
        auto writeError = eeprom->write(address, writeBlock, cTimeout);
        REQUIRE(!writeError);

        // Read and verify whole first page.
        auto [readBlock, readError] = eeprom->read(address, writeBlock.size(), cTimeout);
        REQUIRE(!readError);
        REQUIRE(*readBlock == writeBlock);
    }
}

TEST_CASE("3. Write whole last page of EEPROM", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeproms = {hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1),
                    hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom2)};

    constexpr auto cTimeout = 100ms;
    const std::size_t cPageSize = eeproms.begin()->get()->getPageSize();
    std::uint32_t address{};

    SECTION("3.1. First quarter") { address = (std::numeric_limits<std::uint16_t>::max() + 1) - cPageSize; }

    SECTION("3.2. Second quarter") { address = 2 * (std::numeric_limits<std::uint16_t>::max() + 1) - cPageSize; }

    SECTION("3.3. Third quarter") { address = 3 * (std::numeric_limits<std::uint16_t>::max() + 1) - cPageSize; }

    SECTION("3.4. Fourth quarter") { address = 4 * (std::numeric_limits<std::uint16_t>::max() + 1) - cPageSize; }

    for (const auto& eeprom : eeproms) {
        // Write whole last page.
        std::vector<std::uint8_t> writeBlock(cPageSize);
        getRandomData(writeBlock);
        auto writeError = eeprom->write(address, writeBlock, cTimeout);
        REQUIRE(!writeError);

        // Read and verify whole last page.
        auto [readBlock, readError] = eeprom->read(address, writeBlock.size(), cTimeout);
        REQUIRE(!readError);
        REQUIRE(*readBlock == writeBlock);
    }
}

TEST_CASE("4. Write page at page intersection in EEPROM", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeproms = {hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1),
                    hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom2)};

    constexpr auto cTimeout = 100ms;
    const std::size_t cPageSize = eeproms.begin()->get()->getPageSize();
    constexpr std::uint8_t cPattern1 = 1;
    constexpr std::uint8_t cPattern2 = 2;
    constexpr std::uint8_t cPattern3 = 3;
    const std::vector<std::uint8_t> cWriteBlock1(cPageSize, cPattern1);
    const std::vector<std::uint8_t> cWriteBlock2(cPageSize, cPattern2);
    const std::vector<std::uint8_t> cWriteBlock3(cPageSize, cPattern3);
    std::uint32_t address{};

    SECTION("4.1. First quarter") { address = 0; }

    SECTION("4.2. Second quarter") { address = std::numeric_limits<std::uint16_t>::max() + 1; }

    SECTION("4.3. Third quarter") { address = 2 * (std::numeric_limits<std::uint16_t>::max() + 1); }

    SECTION("4.4. Fourth quarter") { address = 3 * (std::numeric_limits<std::uint16_t>::max() + 1); }

    for (const auto& eeprom : eeproms) {
        // Fill first and second page with pattern.
        auto writeError = eeprom->write(address, cWriteBlock1, cTimeout);
        REQUIRE(!writeError);

        writeError = eeprom->write(address + cPageSize, cWriteBlock2, cTimeout);
        REQUIRE(!writeError);

        // Fill block with size equal to the page size with pattern at the intersection of first and second page.
        writeError = eeprom->write(address + (cPageSize / 2), cWriteBlock3, cTimeout);
        REQUIRE(!writeError);

        auto [readBlock, readError] = eeprom->read(address, 2 * cPageSize, cTimeout);
        REQUIRE(!readError);

        // Verify first pattern.
        for (std::size_t i = 0; i < (cPageSize / 2); ++i)
            REQUIRE(readBlock->at(i) == cPattern1);

        // Verify third pattern.
        for (std::size_t i = cPageSize / 2; i < cPageSize + (cPageSize / 2); ++i)
            REQUIRE(readBlock->at(i) == cPattern3);

        // Verify second pattern.
        for (std::size_t i = cPageSize + (cPageSize / 2); i < (2 * cPageSize); ++i)
            REQUIRE(readBlock->at(i) == cPattern2);
    }
}

TEST_CASE("5. Write page at page intersection in EEPROM with device address change", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeproms = {hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1),
                    hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom2)};

    constexpr auto cTimeout = 100ms;
    const std::size_t cPageSize = eeproms.begin()->get()->getPageSize();
    constexpr std::uint8_t cPattern1 = 1;
    constexpr std::uint8_t cPattern2 = 2;
    constexpr std::uint8_t cPattern3 = 3;
    const std::vector<std::uint8_t> cWriteBlock1(cPageSize, cPattern1);
    const std::vector<std::uint8_t> cWriteBlock2(cPageSize, cPattern2);
    const std::vector<std::uint8_t> cWriteBlock3(cPageSize, cPattern3);
    std::uint32_t address{};

    SECTION("5.1. First-second quarter") { address = (std::numeric_limits<std::uint16_t>::max() + 1) - cPageSize; }

    SECTION("5.2. Second-third quarter") { address = 2 * (std::numeric_limits<std::uint16_t>::max() + 1) - cPageSize; }

    SECTION("5.3. Third-fourth quarter") { address = 3 * (std::numeric_limits<std::uint16_t>::max() + 1) - cPageSize; }

    for (const auto& eeprom : eeproms) {
        // Fill first and second page with pattern.
        auto writeError = eeprom->write(address, cWriteBlock1, cTimeout);
        REQUIRE(!writeError);

        writeError = eeprom->write(address + cPageSize, cWriteBlock2, cTimeout);
        REQUIRE(!writeError);

        // Fill block with size equal to the page size with pattern at the intersection of first and second page.
        writeError = eeprom->write(address + (cPageSize / 2), cWriteBlock3, cTimeout);
        REQUIRE(!writeError);

        auto [readBlock, readError] = eeprom->read(address, 2 * cPageSize, cTimeout);
        REQUIRE(!readError);

        // Verify first pattern.
        for (std::size_t i = 0; i < (cPageSize / 2); ++i)
            REQUIRE(readBlock->at(i) == cPattern1);

        // Verify third pattern.
        for (std::size_t i = cPageSize / 2; i < cPageSize + (cPageSize / 2); ++i)
            REQUIRE(readBlock->at(i) == cPattern3);

        // Verify second pattern.
        for (std::size_t i = cPageSize + (cPageSize / 2); i < (2 * cPageSize); ++i)
            REQUIRE(readBlock->at(i) == cPattern2);
    }
}

TEST_CASE("6. Write whole page of EEPROM with device address change", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeproms = {hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1),
                    hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom2)};

    constexpr auto cTimeout = 100ms;
    const std::uint32_t cSize = eeproms.begin()->get()->getSize();
    const std::size_t cPageSize = eeproms.begin()->get()->getPageSize();

    for (const auto& eeprom : eeproms) {
        // Write whole page.
        std::vector<std::uint8_t> writeBlock(cPageSize);
        getRandomData(writeBlock);
        const std::size_t cAddress = cSize - cPageSize;
        auto writeError = eeprom->write(cAddress, writeBlock, cTimeout);
        REQUIRE(!writeError);

        // Read and verify whole page.
        auto [readBlock, readError] = eeprom->read(cAddress, writeBlock.size(), cTimeout);
        REQUIRE(!readError);
        REQUIRE(*readBlock == writeBlock);
    }
}

TEST_CASE("7. Write every second byte separately in EEPROM", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeproms = {hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1),
                    hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom2)};

    constexpr auto cTimeout = 100ms;
    const std::uint32_t cSize = eeproms.begin()->get()->getSize();
    const std::size_t cPageSize = eeproms.begin()->get()->getPageSize();
    constexpr std::uint8_t cPattern = 6;

    const std::vector<std::uint8_t> cWriteBlock(cPageSize, cPattern);

    for (const auto& eeprom : eeproms) {
        // Clear page.
        std::uint32_t address = cSize / 2;
        auto writeError = eeprom->write(address, cWriteBlock, cTimeout);
        REQUIRE(!writeError);

        // Fill page with pattern (every second byte is a next integral number).
        for (std::uint16_t i = 0; i < cPageSize; i += 2) {
            auto value = static_cast<std::uint8_t>(i / 2);
            writeError = eeprom->write(address + i, &value, sizeof(value), cTimeout);
            REQUIRE(!writeError);
        }

        // Read and verify page.
        for (std::uint16_t i = 0; i < cPageSize; ++i) {
            auto expectedValue = static_cast<std::uint8_t>((i % 2) != 0 ? cPattern : (i / 2));
            std::uint8_t readValue{};
            auto [actualReadSize, error] = eeprom->read(address + i, &readValue, sizeof(readValue), cTimeout);
            REQUIRE(!error);
            REQUIRE(*actualReadSize == sizeof(readValue));
            REQUIRE(readValue == expectedValue);
        }
    }
}

TEST_CASE("8. Read bytes when address exceeds EEPROM size", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    // Read bytes starting from one page after the end of EEPROM.
    const std::size_t cAddress = eeprom->getSize() + eeprom->getPageSize();
    constexpr std::size_t cReadSize = 13;
    constexpr auto cTimeout = 100ms;
    auto [readBlock, error] = eeprom->read(cAddress, cReadSize, cTimeout);
    REQUIRE(error == hal::Error::eInvalidArgument);
    REQUIRE(!readBlock);
}

TEST_CASE("9. Read bytes when address plus size exceeds EEPROM size", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    // Read bytes from address, which summed with the readBlock size gives one byte after the of EEPROM.
    constexpr std::size_t cReadSize = 13;
    const std::size_t cAddress = eeprom->getSize() - (cReadSize - 1);
    constexpr auto cTimeout = 100ms;
    auto [readBlock, error] = eeprom->read(cAddress, cReadSize, cTimeout);
    REQUIRE(error == hal::Error::eInvalidArgument);
    REQUIRE(!readBlock);
}

TEST_CASE("10. Write bytes when address exceeds EEPROM size", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    // Write bytes starting from one page after the end of EEPROM.
    const std::size_t cAddress = eeprom->getSize() + eeprom->getPageSize();
    constexpr std::size_t cWriteSize = 13;
    constexpr std::uint8_t cPattern = 0xa;
    const std::vector<std::uint8_t> cWriteBlock(cWriteSize, cPattern);
    constexpr auto cTimeout = 100ms;
    auto error = eeprom->write(cAddress, cWriteBlock, cTimeout);
    REQUIRE(error == hal::Error::eInvalidArgument);
}

TEST_CASE("11. Write bytes when address plus size exceeds EEPROM size", "[unit][eeprom]")
{
    hal::ScopedHardware hardware;

    auto eeprom = hal::getScopedDevice<hal::storage::IEeprom>(hal::device_id::eAt24Cm02Eeprom1);

    // Write bytes from address, which summed with the readBlock size gives one byte after the of EEPROM.
    constexpr std::size_t cWriteSize = 13;
    constexpr std::uint8_t cPattern = 0xb;
    const std::vector<std::uint8_t> cWriteBlock(cWriteSize, cPattern);
    const std::size_t cAddress = eeprom->getSize() - (cWriteBlock.size() - 1);
    constexpr auto cTimeout = 100ms;
    auto error = eeprom->write(cAddress, cWriteBlock, cTimeout);
    REQUIRE(error == hal::Error::eInvalidArgument);
}
