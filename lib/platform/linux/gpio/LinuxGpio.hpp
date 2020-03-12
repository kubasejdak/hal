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

#pragma once

#include "hal/Error.hpp"
#include "hal/gpio/IGpioPort.hpp"

#include <utils/GlobalRegistry.hpp>

#include <climits>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

struct gpiod_chip;
struct gpiod_line;

namespace hal::gpio {

class LinuxGpio
    : public IGpioPort<std::uint32_t>
    , public utils::Registrable<std::string_view> {
public:
    LinuxGpio(std::string_view name,
              std::string_view gpiochipName,
              const std::vector<int>& offsets,
              const std::vector<int>& directions);
    LinuxGpio(const LinuxGpio&) = delete;
    LinuxGpio(LinuxGpio&& other) noexcept;
    ~LinuxGpio() override;

    LinuxGpio& operator=(const LinuxGpio&) = delete;
    LinuxGpio& operator=(LinuxGpio&&) = delete;

    /// @see IGpioPort::drvSetDirection
    std::error_code drvSetDirection(std::uint32_t /*direction*/, std::uint32_t /*mask*/) override { return Error::eOk; }

    /// @see IGpioPort::drvRead
    std::error_code drvRead(std::uint32_t& value, std::uint32_t mask) override;

    /// @see IGpioPort::drvWrite
    std::error_code drvWrite(std::uint32_t value, std::uint32_t mask) override;

private:
    static constexpr std::size_t m_cPortBits{sizeof(std::uint32_t) * CHAR_BIT};
    static constexpr int m_cGpioInput{1};
    static constexpr int m_cGpioOutput{0};

    gpiod_chip* m_chip;
    std::map<int, gpiod_line*> m_lines;
    std::map<int, int> m_directions;
};

using LinuxGpioDriver = utils::GlobalRegistry<LinuxGpio>;

} // namespace hal::gpio
