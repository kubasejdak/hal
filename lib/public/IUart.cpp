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

#include "hal/uart/IUart.hpp"

#include "hal/Error.hpp"

namespace hal::uart {

std::error_code IUart::open()
{
    if (isOpened())
        return Error::eDeviceOpened;

    auto status = drvOpen();
    m_opened = (status == Error::eOk);
    return status;
}

std::error_code IUart::close()
{
    if (!isOpened())
        return Error::eDeviceNotOpened;

    auto status = drvClose();
    m_opened = !(status == Error::eOk);
    return status;
}

std::error_code IUart::setBaudrate(Baudrate baudrate)
{
    if (isOpened())
        return Error::eDeviceOpened;

    return drvSetBaudrate(baudrate);
}

std::error_code IUart::setMode(Mode mode)
{
    if (isOpened())
        return Error::eDeviceOpened;

    return drvSetMode(mode);
}

std::error_code IUart::setFlowControl(FlowControl flowControl)
{
    if (isOpened())
        return Error::eDeviceOpened;

    return drvSetFlowControl(flowControl);
}

std::error_code IUart::write(const BytesVector& bytes)
{
    return write(bytes.data(), bytes.size());
}

std::error_code IUart::write(const std::uint8_t* bytes, std::size_t size)
{
    if (bytes == nullptr)
        return Error::eInvalidArgument;

    if (!isOpened())
        return Error::eDeviceNotOpened;

    return drvWrite(bytes, size);
}

std::error_code IUart::read(BytesVector& bytes, std::size_t size, osal::Timeout timeout)
{
    bytes.resize(size);
    if (bytes.size() != size)
        return Error::eNoMemory;

    std::size_t actualReadSize{};
    if (auto status = read(bytes.data(), size, timeout, actualReadSize))
        return status;

    bytes.resize(actualReadSize);
    return Error::eOk;
}

std::error_code IUart::read(std::uint8_t* bytes, std::size_t size, osal::Timeout timeout, std::size_t& actualReadSize)
{
    if (bytes == nullptr)
        return Error::eInvalidArgument;

    if (!isOpened())
        return Error::eDeviceNotOpened;

    actualReadSize = 0;
    return drvRead(bytes, size, timeout, actualReadSize);
}

} // namespace hal::uart
