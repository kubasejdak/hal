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
#include <hal/uart/IUart.hpp>
#include <hash/sha256.h>
#include <nlohmann/json.hpp>
#include <osal/Thread.hpp>
#include <osal/sleep.hpp>

#include <catch2/catch.hpp>

#include <algorithm>
#include <chrono>
#include <random>
#include <tuple>

class UartEndpoint {
public:
    UartEndpoint()
        : m_uart(hal::getScopedDevice<hal::uart::IUart>(hal::device_id::eUart0))
    {
        configurePort(hal::uart::Baudrate::e115200);
    }

    UartEndpoint(const UartEndpoint&) = delete;

    UartEndpoint(UartEndpoint&&) = delete;

    virtual ~UartEndpoint() { m_uart->close(); }

    UartEndpoint& operator=(const UartEndpoint&) = delete;

    UartEndpoint& operator=(UartEndpoint&&) = delete;

    void configurePort(hal::uart::Baudrate baudrate)
    {
        if (m_uart->isOpened())
            m_uart->close();

        auto error = m_uart->setMode(hal::uart::Mode::e8n1);
        REQUIRE(!error);
        error = m_uart->setBaudrate(baudrate);
        REQUIRE(!error);
        error = m_uart->open();
        REQUIRE(!error);
    }

    nlohmann::json sendRequest(const nlohmann::json& request)
    {
        writeline(request.dump());
        return receiveRequest();
    }

    nlohmann::json receiveRequest()
    {
        std::string msgIn = readline();
        return nlohmann::json::parse(msgIn);
    }

private:
    std::string readline()
    {
        std::string line;
        std::optional<hal::BytesVector> byte;
        std::error_code error;

        do {
            std::tie(byte, error) = m_uart->read(1, m_cTimeout).toTuple();
            REQUIRE(!error);

            line += byte->at(0);
        }
        while (byte->at(0) != '\n');

        return line;
    }

    void writeline(const std::string& line)
    {
        hal::BytesVector bytes(line.begin(), line.end());
        bytes.push_back('\n');

        auto error = m_uart->write(bytes);
        REQUIRE(!error);
    }

protected:
    static constexpr auto m_cTimeout{5s};
    hal::ScopedDevice<hal::uart::IUart> m_uart; // NOLINT
};

class IUartClient : public UartEndpoint {
public:
    void run()
    {
        sendPing();

        auto baudrates = {hal::uart::Baudrate::e115200,
                          hal::uart::Baudrate::e57600,
                          hal::uart::Baudrate::e38400,
                          hal::uart::Baudrate::e19200,
                          hal::uart::Baudrate::e9600,
                          hal::uart::Baudrate::e4800,
                          hal::uart::Baudrate::e2400,
                          hal::uart::Baudrate::e1200};
        for (const auto& baudrate : baudrates) {
            sendSettings(baudrate);
            sendPing();
            testCase();
        }

        sendSettings(hal::uart::Baudrate::e115200);
    }

    void sendClose()
    {
        nlohmann::json request = {{"type", "close"}};
        auto response = sendRequest(request);
        REQUIRE(response["type"] == "ack");
    }

    virtual void testCase() = 0;

private:
    void sendPing()
    {
        nlohmann::json request = {{"type", "ping"}};
        auto response = sendRequest(request);
        REQUIRE(response["type"] == "pong");
    }

    void sendSettings(hal::uart::Baudrate baudrate)
    {
        nlohmann::json request = {{"type", "settings"}, {"baudrate", int(baudrate)}};
        auto response = sendRequest(request);
        REQUIRE(response["type"] == "ack");

        configurePort(baudrate);
        constexpr auto cDelay = 2s;
        osal::sleep(cDelay);
    }
};

template <typename T = std::size_t>
T generateRandomNumber(T min = std::numeric_limits<T>::min(), T max = std::numeric_limits<T>::max())
{
    std::random_device randomDevice;
    std::mt19937 generator(randomDevice());
    std::uniform_int_distribution<T> distribution(min, max);

    return distribution(generator);
}

void generateRandomData(std::size_t size, hal::BytesVector& bytes)
{
    bytes.resize(size);
    std::generate(bytes.begin(), bytes.end(), [] { return generateRandomNumber<std::uint8_t>(); });
}

class SynchronousClient : public IUartClient {
private:
    void testCase() override
    {
        constexpr std::size_t cMinDataSize = 1;
        constexpr std::size_t cMaxDataSize = 16 * 1024;
        auto size = generateRandomNumber<>(cMinDataSize, cMaxDataSize);

        nlohmann::json request = {{"type", "data"}, {"size", size}};
        auto response = sendRequest(request);
        REQUIRE(response["type"] == "ack");

        while (size != 0) {
            constexpr std::size_t cMinPacketSize = 1;
            constexpr std::size_t cMaxPacketSize = 32;
            auto packetSize = std::min(generateRandomNumber<>(cMinPacketSize, cMaxPacketSize), size);

            hal::BytesVector sendData;
            generateRandomData(packetSize, sendData);
            auto writeError = m_uart->write(sendData);
            REQUIRE(!writeError);

            auto [receiveData, readError] = m_uart->read(packetSize, m_cTimeout);
            REQUIRE(!readError);
            REQUIRE(receiveData->size() == packetSize);
            REQUIRE(sendData == receiveData);

            size -= packetSize;
        }
    }
};

class AsynchronousClient : public IUartClient {
private:
    void threadSender(std::size_t size)
    {
        SHA256 sha256;
        auto toSend = size;
        while (toSend != 0) {
            constexpr std::size_t cMinPacketSize = 1;
            constexpr std::size_t cMaxPacketSize = 32;
            auto packetSize = std::min(generateRandomNumber<>(cMinPacketSize, cMaxPacketSize), toSend);

            hal::BytesVector sendData;
            generateRandomData(packetSize, sendData);
            auto error = m_uart->write(sendData);
            if (error != hal::Error::eOk) {
                REQUIRE(!error);
            }

            sha256.add(sendData.data(), sendData.size());
            toSend -= packetSize;
        }

        m_checksumSender = sha256.getHash();
    }

    void threadReceiver(std::size_t size)
    {
        SHA256 sha256;
        auto toReceive = size;
        while (toReceive != 0) {
            auto [receiveData, error] = m_uart->read(1, m_cTimeout);
            if (error != hal::Error::eOk) {
                REQUIRE(!error);
            }

            sha256.add(receiveData->data(), receiveData->size());
            toReceive -= receiveData->size();
        }

        m_checksumReceiver = sha256.getHash();
    }

    void testCase() override
    {
        constexpr std::size_t cMinDataSize = 1;
        constexpr std::size_t cMaxDataSize = 16 * 1024;
        auto size = generateRandomNumber<>(cMinDataSize, cMaxDataSize);

        nlohmann::json request = {{"type", "data"}, {"size", size}};
        auto response = sendRequest(request);
        REQUIRE(response["type"] == "ack");

        m_checksumSender.clear();
        m_checksumReceiver.clear();

        {
            osal::Thread sender(&AsynchronousClient::threadSender, this, size);
            osal::Thread receiver(&AsynchronousClient::threadReceiver, this, size);
        }

        REQUIRE(m_checksumSender == m_checksumReceiver);
    }

private:
    std::string m_checksumSender;
    std::string m_checksumReceiver;
};

TEST_CASE("1. Transfers with daemon server", "[unit][hal][uart]")
{
    hal::ScopedHardware hardware;

    SECTION("1.1. Synchronous transmission")
    {
        SynchronousClient client;
        client.run();
    }

    SECTION("1.2. Asynchronous transmission")
    {
        AsynchronousClient client;
        client.run();
        client.sendClose();
    }
}
