/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2020, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include "LinuxUart.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <utility>

namespace hal::uart {

LinuxUart::LinuxUart(std::string device)
    : m_device(std::move(device))
{}

std::error_code LinuxUart::drvOpen()
{
    m_fd = ::open(m_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); // NOLINT
    if (m_fd < 0)
        return Error::eFilesystemError;

    if (tcgetattr(m_fd, &m_ttyPrev) != 0)
        return Error::eHardwareError;

    if (tcflush(m_fd, TCIFLUSH) != 0)
        return Error::eHardwareError;

    if (tcflush(m_fd, TCOFLUSH) != 0)
        return Error::eHardwareError;

    if (tcsetattr(m_fd, TCSANOW, &m_tty) != 0)
        return Error::eHardwareError;

    return Error::eOk;
}

std::error_code LinuxUart::drvClose()
{
    if (tcsetattr(m_fd, TCSANOW, &m_ttyPrev) != 0)
        return Error::eHardwareError;

    if (::close(m_fd) < 0)
        return Error::eFilesystemError;

    return Error::eOk;
}

std::error_code LinuxUart::drvSetBaudrate(Baudrate baudrate)
{
    auto termiosBaudrate = [](Baudrate baudrate) {
        switch (baudrate) {
            case Baudrate::e1200: return B1200;
            case Baudrate::e2400: return B2400;
            case Baudrate::e4800: return B4800;
            case Baudrate::e9600: return B9600;
            case Baudrate::e19200: return B19200;
            case Baudrate::e38400: return B38400;
            case Baudrate::e57600: return B57600;
            case Baudrate::e115200: return B115200;
            case Baudrate::e230400: return B230400;
            case Baudrate::e460800: return B460800;
            case Baudrate::e921600: return B921600;
            default: break;
        }

        return -1;
    };

    if (cfsetospeed(&m_tty, termiosBaudrate(baudrate)) != 0)
        return Error::eHardwareError;

    if (cfsetispeed(&m_tty, termiosBaudrate(baudrate)) != 0)
        return Error::eHardwareError;

    return Error::eOk;
}

std::error_code LinuxUart::drvSetMode(Mode mode)
{
    m_tty.c_cflag = 0;
    m_tty.c_iflag = 0;
    m_tty.c_oflag = 0;
    m_tty.c_lflag = 0;

    switch (mode) {
        case Mode::e8n1:
            m_tty.c_cflag |= tcflag_t(CS8);    // 8 data bits.
            m_tty.c_cflag |= tcflag_t(CREAD);  // Enable receiver.
            m_tty.c_cflag |= tcflag_t(CLOCAL); // Ignore modem control lines.
            m_tty.c_iflag |= tcflag_t(IGNPAR); // Ignore framing errors and parity errors.
            m_tty.c_iflag |= tcflag_t(INPCK);  // Enable input parity checking.
            break;
        default: return Error::eNotSupported;
    }

    return Error::eOk;
}

std::error_code LinuxUart::drvWrite(const std::uint8_t* bytes, std::size_t size)
{
    std::size_t writeSize = 0;
    while (writeSize != size) {
        auto result = ::write(m_fd, bytes + writeSize, size - writeSize);
        if (result == -1) {
            if (errno == EAGAIN)
                continue;

            return Error::eHardwareError;
        }

        writeSize += result;
    }

    return Error::eOk;
}

std::error_code
LinuxUart::drvRead(std::uint8_t* bytes, std::size_t size, osal::Timeout timeout, std::size_t& actualReadSize)
{
    auto toRead = size;

    auto toTimeval = [](osal::Timeout timeout) {
        auto timeLeft = timeout.timeLeft();
        auto timeLeftSec = std::chrono::duration_cast<std::chrono::seconds>(timeLeft);
        auto timeLeftUSec = std::chrono::duration_cast<std::chrono::microseconds>(timeLeft - timeLeftSec);
        timeval tv{};
        tv.tv_sec = timeLeftSec.count();
        tv.tv_usec = timeLeftUSec.count();

        return tv;
    };

    do {
        fd_set readFds{};
        FD_ZERO(&readFds);      // NOLINT(hicpp-no-assembler,readability-isolate-declaration)
        FD_SET(m_fd, &readFds); // NOLINT(hicpp-no-assembler,hicpp-signed-bitwise)

        auto tv = toTimeval(timeout);
        auto result = select(m_fd + 1, &readFds, nullptr, nullptr, &tv);
        if (result > 0) {
            auto status = ::read(m_fd, bytes + (size - toRead), toRead);
            if (status != -1) {
                toRead -= status;
            }
        }
    } while (toRead != 0 && !timeout.isExpired());

    actualReadSize = size - toRead;
    if (timeout.isExpired() && toRead != 0)
        return Error::eTimeout;

    return Error::eOk;
}

} // namespace hal::uart
