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

#pragma once

#include "TtyUart.hpp"

#include "hal/Error.hpp"

#include <utility>

namespace hal::uart {

TtyUart::TtyUart(std::string& ttyDevicePath)
    : m_ttyDevicePath(std::move(ttyDevicePath))
{}

std::error_code TtyUart::drvOpen()
{
    /// @todo: Implement.
    return Error::eOk;
}

std::error_code TtyUart::drvClose()
{
    /// @todo: Implement.
    return Error::eOk;
}

std::error_code TtyUart::drvSetBaudrate(Baudrate baudrate)
{
    /// @todo: Implement.
    (void) baudrate;
    return Error::eOk;
}

std::error_code TtyUart::drvSetMode(Mode mode)
{
    /// @todo: Implement.
    (void) mode;
    return Error::eOk;
}

std::error_code TtyUart::drvSetFlowControl(FlowControl flowControl)
{
    /// @todo: Implement.
    (void) flowControl;
    return Error::eOk;
}

std::error_code TtyUart::drvWrite(const std::uint8_t* bytes, std::size_t size)
{
    /// @todo: Implement.
    (void) bytes;
    (void) size;
    return Error::eOk;
}

std::error_code
TtyUart::drvRead(std::uint8_t* bytes, std::size_t size, std::uint32_t timeoutMs, std::size_t& actualReadSize)
{
    /// @todo: Implement.
    (void) bytes;
    (void) size;
    (void) timeoutMs;
    (void) actualReadSize;
    return Error::eOk;
}

} // namespace hal::uart
