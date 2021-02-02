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

/// Represents a single humidity sensor device. All operations will be limited to the given instance of this class.
class IHumiditySensor : public Device {
public:
    /// Default constructor.
    IHumiditySensor();

    /// Reads the current relative humidity value from the sensor.
    /// @param relativeHumidity         Output parameter with the humidity percentage in range 0-100 %.
    /// @return Error code of the operation.
    std::error_code read(float& relativeHumidity);

    /// Returns the minimal value of the humidity that can be returned from this device.
    /// @return Minimal value of the humidity that can be returned from this device.
    /// @note Default implementation returns the value of 0 %.
    [[nodiscard]] virtual float minValue() const { return 0.0F; }

    /// Returns the maximal value of the humidity that can be returned from this device.
    /// @return Maximal value of the humidity that can be returned from this device.
    /// @note Default implementation returns the value of 100 %.
    [[nodiscard]] virtual float maxValue() const { return 100.0F; }

private:
    /// Device specific implementation of the method that reads the current relative humidity value.
    /// @param relativeHumidity         Output parameter with the relative humidity percentage in range 0-100 %.
    /// @return Error code of the operation.
    virtual std::error_code drvRead(float& relativeHumidity) = 0;
};

} // namespace hal::sensor
