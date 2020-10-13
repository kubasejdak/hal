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

#include "LinuxI2c.hpp"

#include "hal/Error.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <utility>

namespace hal::i2c {

LinuxI2c::LinuxI2c(std::string device)
    : m_device(std::move(device))
{}

std::error_code LinuxI2c::drvOpen()
{
    m_fd = ::open(m_device.c_str(), O_RDWR | O_CLOEXEC); // NOLINT
    if (m_fd < 0)
        return Error::eFilesystemError;

    return Error::eOk;
}

std::error_code LinuxI2c::drvClose()
{
    if (::close(m_fd) < 0)
        return Error::eFilesystemError;

    return Error::eOk;
}

std::error_code LinuxI2c::setSlaveAddress(std::uint16_t address) const
{
    if (ioctl(m_fd, I2C_SLAVE, address) < 0) // NOLINT
        return Error::eHardwareError;

    return Error::eOk;
}

std::error_code LinuxI2c::drvWrite(std::uint16_t address,
                                   const std::uint8_t* bytes,
                                   std::size_t size,
                                   bool stop,
                                   osal::Timeout /*unused*/)
{
    if (!stop) {
        std::vector<std::uint8_t> data(bytes, bytes + size);
        m_writeData.emplace_back(std::move(data));
        return Error::eOk;
    }

    if (auto error = setSlaveAddress(address))
        return error;

    if (::write(m_fd, bytes, size) != static_cast<int>(size))
        return Error::eHardwareError;

    return Error::eOk;
}

std::error_code LinuxI2c::drvRead(std::uint16_t address,
                                  std::uint8_t* bytes,
                                  std::size_t size,
                                  osal::Timeout /*unused*/,
                                  std::size_t& actualReadSize)
{
    if (!m_writeData.empty()) {
        std::vector<i2c_msg> msgs;
        for (auto& transferBytes : m_writeData) {
            i2c_msg sendMsg{};
            sendMsg.addr = address;
            sendMsg.flags = 0;
            sendMsg.len = transferBytes.size();
            sendMsg.buf = transferBytes.data();
            msgs.emplace_back(sendMsg);
        }

        i2c_msg receiveMsg{};
        receiveMsg.addr = address;
        receiveMsg.flags = I2C_M_RD;
        receiveMsg.len = size;
        receiveMsg.buf = bytes;
        msgs.emplace_back(receiveMsg);

        std::array<i2c_rdwr_ioctl_data, 1> msgSet{};
        msgSet[0].msgs = msgs.data();
        msgSet[0].nmsgs = msgs.size();
        if (ioctl(m_fd, I2C_RDWR, msgSet.data()) < 0) // NOLINT
            return Error::eHardwareError;

        m_writeData.clear();
    }
    else {
        if (auto error = setSlaveAddress(address))
            return error;

        if (::read(m_fd, bytes, size) != int(size))
            return Error::eHardwareError;
    }

    actualReadSize = size;
    return Error::eOk;
}

} // namespace hal::i2c
