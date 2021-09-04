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

#include "hal/i2c/II2c.hpp"
#include "hal/sensor/IHumiditySensor.hpp"

#include <osal/Mutex.hpp>
#include <osal/timestamp.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <system_error>

namespace hal::sensor {

/// Converted measurements from the SHT3x humidity sensor.
struct Sht3xMeasurement {
    float temperature;
    float relativeHumidity;
};

/**
 * Represents the SHT3x-DIS driver.
 */
class Sht3xDisSensor {
public:
    /// Constructor.
    /// @param i2c                  I2C driver associated with this SHT3x device.
    /// @param address              Address of this SHT3x device in the given addressing mode.
    /// @param refreshThresholdMs   Maximal time from the last read operation, that doesn't require refreshing
    ///                             the measurements.
    Sht3xDisSensor(std::shared_ptr<i2c::II2c> i2c,
                   std::uint16_t address,
                   std::chrono::milliseconds refreshThreshold = 100ms);

    /// Copy constructor.
    /// @note This constructor is deleted, because Sht3xDisSensor is not meant to be copy-constructed.
    Sht3xDisSensor(const Sht3xDisSensor&) = delete;

    /// Copy constructor.
    /// @note This constructor is deleted, because Sht3xDisSensor is not meant to be move-constructed.
    Sht3xDisSensor(Sht3xDisSensor&&) noexcept = delete;

    /// Destructor.
    ~Sht3xDisSensor();

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because Sht3xDisSensor is not meant to be copy-assigned.
    Sht3xDisSensor& operator=(const Sht3xDisSensor&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because Sht3xDisSensor is not meant to be move-assigned.
    Sht3xDisSensor& operator=(Sht3xDisSensor&&) noexcept = delete;

    /// Returns the converted measurements of this instance of the SHT3x sensor.
    /// @param measurement          Output variable where the measurement will be stored.
    /// @return Error code of the operation.
    std::error_code getMeasurement(Sht3xMeasurement& measurement);

private:
    std::shared_ptr<i2c::II2c> m_i2c;
    std::uint16_t m_address;
    Sht3xMeasurement m_measurement{};
    osal::Mutex m_mutex;
    osal::Timeout m_cacheTimeout;
};

} // namespace hal::sensor
