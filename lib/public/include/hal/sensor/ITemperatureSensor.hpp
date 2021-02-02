/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2021-2021, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include <system_error>

namespace hal::sensor {

/// Represents a single temperature sensor device. All operations will be limited to the given instance of this class.
class ITemperatureSensor : public Device {
public:
    /// Default constructor.
    ITemperatureSensor();

    /// Reads the current temperature value from the sensor.
    /// @param temperature          Output parameter with the temperature value in Celsius degrees.
    /// @return Error code of the operation.
    std::error_code read(float& temperature);

    /// Returns the minimal value of the temperature that can be returned from this device in Celsius degrees.
    /// @return Minimal value of the temperature that can be returned from this device in Celsius degrees.
    [[nodiscard]] virtual float minValue() const = 0;

    /// Returns the maximal value of the temperature that can be returned from this device in Celsius degrees.
    /// @return Maximal value of the temperature that can be returned from this device in Celsius degrees.
    [[nodiscard]] virtual float maxValue() const = 0;

private:
    /// Device specific implementation of the method that reads the current temperature value in Celsius degrees.
    /// @param temperature          Output parameter with the temperature value in Celsius degrees.
    /// @return Error code of the operation.
    virtual std::error_code drvRead(float& temperature) = 0;
};

} // namespace hal::sensor
