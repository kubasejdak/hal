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

#pragma once

#include "hal/gpio/IGpioPort.hpp"

#include <climits>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

struct gpiod_chip;
struct gpiod_line;

namespace hal::gpio {

/// Represents Linux driver for the IGpioPort interface.
/// @note This class uses libgpiod as the underlying GPIO driver.
class LinuxGpio final : public IGpioPort<std::uint32_t> {
public:
    /// Constructor.
    /// @param name                 Name of the GPIO instance handled by this object.
    /// @param gpiochipName         Name of the GPIO chip (e.g. gpiochip0) to be used by this object.
    /// @param offsets              List of numerical identifiers of the GPIO lines handled by this object.
    /// @param directions           List of directions for each GPIO line handled by this object.
    LinuxGpio(std::string_view name,
              std::string_view gpiochipName,
              const std::vector<int>& offsets,
              const std::vector<int>& directions);

    /// Copy constructor.
    /// @note This constructor is deleted, because LinuxGpio is not meant to be copy-constructed.
    LinuxGpio(const LinuxGpio&) = delete;

    /// Move constructor.
    /// @param other                Object to be moved from.
    LinuxGpio(LinuxGpio&& other) noexcept;

    /// Destructor.
    ~LinuxGpio() override;

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because LinuxGpio is not meant to be copy-assigned.
    LinuxGpio& operator=(const LinuxGpio&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because LinuxGpio is not meant to be move-assigned.
    LinuxGpio& operator=(LinuxGpio&&) = delete;

    /// @see IGpioPort::get().
    std::error_code get(std::uint32_t& value, std::uint32_t mask) override;

    /// @see IGpioPort::set().
    std::error_code set(std::uint32_t value, std::uint32_t mask) override;

private:
    static constexpr std::size_t m_cPortBits{sizeof(std::uint32_t) * CHAR_BIT};
    static constexpr int m_cGpioInput{1};
    static constexpr int m_cGpioOutput{0};

    gpiod_chip* m_chip;
    std::map<unsigned int, gpiod_line*> m_lines;
    std::map<unsigned int, unsigned int> m_directions;
};

} // namespace hal::gpio
