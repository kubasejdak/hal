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

#include <digestpp/digestpp.hpp>
#include <hal/Error.hpp>
#include <hal/Hardware.hpp>
#include <hal/factory.hpp>
#include <hal/uart/IUart.hpp>
#include <osal/Thread.hpp>
#include <osal/sleep.hpp>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <catch2/catch.hpp>

#include <algorithm>
#include <chrono>
#include <random>

class UartEndpoint {
public:
    UartEndpoint()
        : m_uart(hal::getDevice<hal::uart::IUart>(hal::device_id::eUart0))
    {
        configurePort(hal::uart::Baudrate::e115200);
    }

    UartEndpoint(const UartEndpoint&) = delete;

    UartEndpoint(UartEndpoint&&) = delete;

    virtual ~UartEndpoint()
    {
        m_uart->close();
        hal::returnDevice(m_uart);
    }

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

    rapidjson::Document sendRequest(const rapidjson::Document& request)
    {
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        request.Accept(writer);

        writeline(buffer.GetString());
        return receiveRequest();
    }

    rapidjson::Document receiveRequest()
    {
        std::string msgIn = readline();
        rapidjson::Document json;
        json.Parse(msgIn.data());
        return json;
    }

private:
    std::string readline()
    {
        std::string line;
        hal::BytesVector byte;

        do {
            auto error = m_uart->read(byte, 1, m_cTimeout);
            REQUIRE(!error);

            line += byte[0];
        } while (byte[0] != '\n');

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
    std::shared_ptr<hal::uart::IUart> m_uart; // NOLINT
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
        rapidjson::Document request;
        request.SetObject();
        request.AddMember("type", "close", request.GetAllocator());
        auto response = sendRequest(request);
        REQUIRE(response["type"] == "ack");
    }

    virtual void testCase() = 0;

private:
    void sendPing()
    {
        rapidjson::Document request;
        request.SetObject();
        request.AddMember("type", "ping", request.GetAllocator());
        auto response = sendRequest(request);
        REQUIRE(response["type"] == "pong");
    }

    void sendSettings(hal::uart::Baudrate baudrate)
    {
        rapidjson::Document request;
        request.SetObject();
        request.AddMember("type", "settings", request.GetAllocator());
        request.AddMember("baudrate", static_cast<int>(baudrate), request.GetAllocator());
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

        rapidjson::Document request;
        request.SetObject();
        request.AddMember("type", "data", request.GetAllocator());
        request.AddMember("size", size, request.GetAllocator());
        auto response = sendRequest(request);
        REQUIRE(response["type"] == "ack");

        while (size != 0) {
            constexpr std::size_t cMinPacketSize = 1;
            constexpr std::size_t cMaxPacketSize = 32;
            auto packetSize = std::min(generateRandomNumber<>(cMinPacketSize, cMaxPacketSize), size);

            hal::BytesVector sendData;
            generateRandomData(packetSize, sendData);
            auto error = m_uart->write(sendData);
            REQUIRE(!error);

            hal::BytesVector receiveData;
            error = m_uart->read(receiveData, packetSize, m_cTimeout);
            REQUIRE(!error);
            REQUIRE(receiveData.size() == packetSize);
            REQUIRE(sendData == receiveData);

            size -= packetSize;
        }
    }
};

class AsynchronousClient : public IUartClient {
private:
    void threadSender(int size)
    {
        digestpp::sha256 checksum;
        auto toSend = size;
        while (toSend != 0) {
            constexpr int cMinPacketSize = 1;
            constexpr int cMaxPacketSize = 32;
            auto packetSize = std::min(generateRandomNumber<>(cMinPacketSize, cMaxPacketSize), toSend);

            hal::BytesVector sendData;
            generateRandomData(packetSize, sendData);
            auto error = m_uart->write(sendData);
            if (error != hal::Error::eOk) {
                REQUIRE(!error);
            }

            checksum.absorb(sendData.begin(), sendData.end());
            toSend -= packetSize;
        }

        m_checksumSender = checksum.hexdigest();
    }

    void threadReceiver(int size)
    {
        digestpp::sha256 checksum;
        auto toReceive = size;
        while (toReceive != 0) {
            hal::BytesVector receiveData;
            auto error = m_uart->read(receiveData, 1, m_cTimeout);
            if (error != hal::Error::eOk) {
                REQUIRE(!error);
            }

            checksum.absorb(receiveData.begin(), receiveData.end());
            toReceive -= receiveData.size();
        }

        m_checksumReceiver = checksum.hexdigest();
    }

    void testCase() override
    {
        constexpr int cMinDataSize = 1;
        constexpr int cMaxDataSize = 16 * 1024;
        auto size = generateRandomNumber<>(cMinDataSize, cMaxDataSize);

        rapidjson::Document request;
        request.SetObject();
        request.AddMember("type", "data", request.GetAllocator());
        request.AddMember("size", size, request.GetAllocator());
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

TEST_CASE("Transfers with daemon server", "[unit][hal][uart]")
{
    hal::ScopedHardware hardware;

    SECTION("Synchronous transmission")
    {
        SynchronousClient client;
        client.run();
    }

    SECTION("Asynchronous transmission")
    {
        AsynchronousClient client;
        client.run();
        client.sendClose();
    }
}
