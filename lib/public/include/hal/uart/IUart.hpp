/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Grzegorz Heldt
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2021, Kuba Sejdak <kuba.sejdak@gmail.com>
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
#include "hal/types.hpp"

#include <osal/Timeout.hpp>
#include <utils/GlobalRegistry.hpp>

#include <system_error>

namespace hal::uart {

/// Represents the baud rate (speed) used in the UART transmission.
enum class Baudrate {
    e1200 = 1200,     // NOLINT
    e2400 = 2400,     // NOLINT
    e4800 = 4800,     // NOLINT
    e9600 = 9600,     // NOLINT
    e19200 = 19200,   // NOLINT
    e38400 = 38400,   // NOLINT
    e57600 = 57600,   // NOLINT
    e115200 = 115200, // NOLINT
    e230400 = 230400, // NOLINT
    e460800 = 460800, // NOLINT
    e921600 = 921600  // NOLINT
};

// clang-format off
/// Represents possible UART configurations in traditional form (<data bits><parity><stop bits>).
enum class Mode {
    e8n1 // NOLINT
};

/// Represents the flow control selected to be used in the UART transmission.
enum class FlowControl {
    eNone,
    eRtsCts,
    eXonXoff
};
// clang-format on

/// Represents a single UART device. All operations will be limited to the given instance of this class.
class IUart : public Device {
public:
    /// Default constructor.
    IUart()
        : Device(SharingPolicy::eSingle)
    {}

    /// Copy constructor.
    /// @note This constructor is deleted, because IUart is not meant to be copy-constructed.
    IUart(const IUart&) = delete;

    /// Move constructor.
    IUart(IUart&&) = default;

    /// Destructor.
    ~IUart() override { close(); }

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because IUart is not meant to be copy-assigned.
    IUart& operator=(const IUart&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because IUart is not meant to be move-assigned.
    IUart& operator=(IUart&&) = delete;

    /// Opens the transmission channel. If the configuration of this device is valid, then after a successful
    /// call to this method the device will be able to transmit the data according to the settings.
    /// @return Error code of the operation.
    std::error_code open();

    /// Closes the transmission channel. After a successful call to this method the device will not be able.
    /// to transmit any data.
    /// @return Error code of the operation.
    std::error_code close();

    /// Checks if the device is currently opened.
    /// @see IUart::open().
    /// @return Flag indicating if the device is opened.
    /// @retval true                Device is currently opened.
    /// @retval false               Device is currently closed.
    [[nodiscard]] bool isOpened() const { return m_opened; }

    /// Sets the given baudrate to be used in the UART transmission.
    /// @param baudrate             Baudrate to be used.
    /// @return Error code of the operation.
    std::error_code setBaudrate(Baudrate baudrate);

    /// Sets the mode (data bits, parity, stop bits) to be used in the UART transmission.
    /// @param mode                 Mode to be used.
    /// @return Error code of the operation.
    std::error_code setMode(Mode mode);

    /// Sets the given flow control to be used in the UART transmission.
    /// @param flowControl          Flow control to be used.
    /// @return Error code of the operation.
    std::error_code setFlowControl(FlowControl flowControl);

    /// Transmits the given vector of bytes using the current UART instance.
    /// @param bytes                Vector of raw bytes to be transmitted.
    /// @return Error code of the operation.
    /// @note This method will block until all data has been transferred to the driver.
    ///       It is up to the driver to decide if the data will be buffered (queued) or transmitted immediately.
    std::error_code write(const BytesVector& bytes);

    /// Transmits the given memory block of bytes using the current UART instance.
    /// @param bytes                Memory block of raw bytes to be transmitted.
    /// @param size                 Size of the memory block to be transmitted.
    /// @return Error code of the operation.
    /// @note This method will block until all data has been transferred to the driver.
    ///       It is up to the driver to decide if the data will be buffered (queued) or transmitted immediately.
    std::error_code write(const std::uint8_t* bytes, std::size_t size);

    /// Receives the demanded number of bytes from the current UART instance.
    /// @param bytes                Vector where the received data will be placed by this method.
    /// @param size                 Number of bytes to be received from the current UART instance.
    /// @param timeout              Maximal time to wait for the data.
    /// @return Error code of the operation.
    /// @note This method does not assume, that the output vector has the proper capacity. It will be
    ///       automatically expanded, if needed, by the container itself. Size of the vector after call
    ///       to this method will indicate the actual number of received bytes.
    std::error_code read(BytesVector& bytes, std::size_t size, osal::Timeout timeout);

    /// Receives the demanded number of bytes from the current UART instance.
    /// @param bytes                Memory block where the received data will be placed by this method.
    /// @param size                 Number of bytes to be received from the current UART instance.
    /// @param timeout              Maximal time to wait for the data.
    /// @param actualReadSize       Actual number of received bytes.
    /// @return Error code of the operation.
    /// @note This method assumes, that the output memory block has the proper capacity. After call to this
    ///       method the 'actualReadSize' parameter will indicate the actual number of received bytes.
    ///       It is also assumed, that output memory block is empty.
    std::error_code read(std::uint8_t* bytes, std::size_t size, osal::Timeout timeout, std::size_t& actualReadSize);

private:
    /// Device specific implementation of the opening transmission channel.
    /// @return Error code of the operation.
    virtual std::error_code drvOpen() = 0;

    /// Device specific implementation of the transmission channel closing method.
    /// @return Error code of the operation.
    virtual std::error_code drvClose() = 0;

    /// Device specific implementation of setting the transmission baudrate.
    /// @param baudrate             Baudrate to be used.
    /// @return Error code of the operation.
    virtual std::error_code drvSetBaudrate(Baudrate baudrate) = 0;

    /// Device specific implementation of setting the transmission mode (data bits, parity, stop bits).
    /// @param mode                 Mode to be used.
    /// @return Error code of the operation.
    virtual std::error_code drvSetMode(Mode mode) = 0;

    /// Device specific implementation of setting the flow control.
    /// @param flowControl          Flow control to be used.
    /// @return Error code of the operation.
    virtual std::error_code drvSetFlowControl(FlowControl flowControl) = 0;

    /// Device specific implementation of transmitting the memory block of bytes.
    /// @param bytes                Byte to be transmitted.
    /// @param size                 Size of the memory block to be transmitted.
    /// @return Error code of the operation.
    virtual std::error_code drvWrite(const std::uint8_t* bytes, std::size_t size) = 0;

    /// Device specific implementation of the method that reads demanded number of bytes.
    /// @param bytes                Memory block where the received data will be placed by this method.
    /// @param size                 Number of bytes to be received from the current UART instance.
    /// @param timeout              Maximal time to wait for the data.
    /// @param actualReadSize       Actual number of received bytes.
    /// @return Error code of the operation.
    virtual std::error_code
    drvRead(std::uint8_t* bytes, std::size_t size, osal::Timeout timeout, std::size_t& actualReadSize)
        = 0;

private:
    bool m_opened{};
};

/// Represents GlobalRegistry of IUart instances.
using Registry = utils::GlobalRegistry<IUart>;

} // namespace hal::uart
