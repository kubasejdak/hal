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

#include "hal/storage/IEeprom.hpp"

#include "hal/Error.hpp"

namespace hal::storage {

IEeprom::IEeprom(std::size_t size, std::size_t pageSize)
    : Device(SharingPolicy::eSingle)
    , m_size(size)
    , m_pageSize(pageSize)
{}

std::error_code IEeprom::write(std::uint32_t address, const BytesVector& bytes, osal::Timeout timeout)
{
    return write(address, bytes.data(), bytes.size(), timeout);
}

std::error_code
IEeprom::write(std::uint32_t address, const std::uint8_t* bytes, std::size_t size, osal::Timeout timeout)
{
    if ((address + size) > getSize())
        return Error::eInvalidArgument;

    if (bytes == nullptr)
        return Error::eInvalidArgument;

    if (size == 0)
        return Error::eOk;

    return drvWrite(address, bytes, size, timeout);
}

std::error_code IEeprom::read(std::uint32_t address, BytesVector& bytes, std::size_t size, osal::Timeout timeout)
{
    bytes.resize(size);
    if (bytes.size() != size)
        return Error::eNoMemory;

    std::size_t actualReadSize{};
    auto error = read(address, bytes.data(), size, timeout, actualReadSize);
    bytes.resize(actualReadSize);
    return error;
}

std::error_code IEeprom::read(std::uint32_t address,
                              std::uint8_t* bytes,
                              std::size_t size,
                              osal::Timeout timeout,
                              std::size_t& actualReadSize)
{
    if ((address + size) > getSize())
        return Error::eInvalidArgument;

    if (bytes == nullptr)
        return Error::eInvalidArgument;

    actualReadSize = 0;
    return drvRead(address, bytes, size, timeout, actualReadSize);
}

} // namespace hal::storage
