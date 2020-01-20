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

#include "hal/gpio/IGpioPort.hpp"
#include "hal/gpio/types.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace hal::gpio {
namespace detail {

/// Represents the filesystem path to the GPIO submodule in SysFs.
static const std::filesystem::path cSysFsGpioPath = "/sys/class/gpio"; // NOLINT(cert-err58-cpp)

} // namespace detail

template <typename WidthType>
class SysFsGpio : public IGpioPort<WidthType, Access::eReadWrite> {
public:
    /// Constructor.
    /// @param chipInstance         GPIO chip instance name.
    explicit SysFsGpio(const std::string& chipInstance)
    {
        auto gpioChipPath = detail::cSysFsGpioPath / chipInstance;
        if (!std::filesystem::exists(gpioChipPath)) {
            assert(false);
            return;
        }

        int maxPin{};
        if (maxHandledPin(chipInstance, maxPin)) {
            assert(false);
            return;
        }

        m_validPins = (1U << maxPin) - 1;
    }

private:
    /// @see IGpioPort::drvInitPin().
    std::error_code drvInitPin(Pin pin) override
    {
        int pinId = toInt(pin);
        if (!m_validPins[pinId])
            return Error::eInvalidArgument;

        if (m_usedPins[pinId])
            return Error::eDeviceTaken;

        auto pinPath = detail::cSysFsGpioPath / ("gpio" + std::to_string(pinId));
        if (std::filesystem::exists(pinPath))
            return Error::ePathExists;

        auto exportPath = detail::cSysFsGpioPath / "export";
        if (!std::filesystem::exists(exportPath))
            return Error::ePathDoesNotExist;

        std::ofstream exportFile(exportPath);
        if (!exportFile.is_open())
            return Error::eFilesystemError;

        exportFile << pinId;

        m_usedPins.set(pinId, true);
        return Error::eOk;
    }

    /// @see IGpioPort::drvDeinitPin().
    std::error_code drvDeinitPin(Pin pin) override
    {
        int pinId = toInt(pin);
        if (!m_validPins[pinId])
            return Error::eInvalidArgument;

        if (!m_usedPins[pinId])
            return Error::eDeviceNotTaken;

        auto pinPath = detail::cSysFsGpioPath / ("gpio" + std::to_string(pinId));
        if (!std::filesystem::exists(pinPath))
            return Error::ePathDoesNotExist;

        auto unexportPath = detail::cSysFsGpioPath / "unexport";
        if (!std::filesystem::exists(unexportPath))
            return Error::ePathDoesNotExist;

        std::ofstream unexportFile(unexportPath);
        if (!unexportFile.is_open())
            return Error::eFilesystemError;

        unexportFile << pinId;

        m_usedPins.set(pinId, false);
        return Error::eOk;
    }

    /// @see IGpioPort::drvSetPinMode().
    std::error_code drvSetPinMode(Pin /*unused*/, unsigned int /*unused*/) override { return Error::eNotSupported; }

    /// @see IGpioPort::drvSetDirection().
    std::error_code drvSetDirection(WidthType direction, WidthType mask) override
    {
        PinMask<WidthType> directionMask = direction;
        PinMask<WidthType> affectedMask = mask;

        for (std::size_t i = 0; i < affectedMask.size(); ++i) {
            if (!affectedMask[i])
                continue;

            if (!m_usedPins[i])
                return Error::eDeviceNotTaken;

            auto directionPath = detail::cSysFsGpioPath / ("gpio" + std::to_string(i)) / "direction";
            if (!std::filesystem::exists(directionPath))
                return Error::ePathDoesNotExist;

            std::ofstream directionFile(directionPath);
            if (!directionFile.is_open())
                return Error::eFilesystemError;

            directionFile << (directionMask[i] ? "in" : "out");
        }

        return Error::eOk;
    }

    /// @see IGpioPort::drvRead().
    std::error_code drvRead(WidthType& value, WidthType mask) override
    {
        PinMask<WidthType> valueMask;
        PinMask<WidthType> affectedMask = mask;

        for (std::size_t i = 0; i < affectedMask.size(); ++i) {
            if (!affectedMask[i])
                continue;

            if (!m_usedPins[i])
                return Error::eDeviceNotTaken;

            auto valuePath = detail::cSysFsGpioPath / ("gpio" + std::to_string(i)) / "value";
            if (!std::filesystem::exists(valuePath))
                return Error::ePathDoesNotExist;

            std::ofstream valueFile(valuePath);
            if (!valueFile.is_open())
                return Error::eFilesystemError;

            std::stringstream buffer;
            buffer << valueFile.rdbuf();
            int pinValue{};
            buffer >> pinValue;
            valueMask[i] = pinValue;
        }

        value = static_cast<WidthType>(valueMask.to_ullong());
        return Error::eOk;
    }

    /// @see IGpioPort::drvWrite().
    std::error_code drvWrite(WidthType value, WidthType mask) override
    {
        PinMask<WidthType> valueMask = value;
        PinMask<WidthType> affectedMask = mask;

        for (std::size_t i = 0; i < affectedMask.size(); ++i) {
            if (!affectedMask[i])
                continue;

            if (!m_usedPins[i])
                return Error::eDeviceNotTaken;

            auto valuePath = detail::cSysFsGpioPath / ("gpio" + std::to_string(i)) / "value";
            if (!std::filesystem::exists(valuePath))
                return Error::ePathDoesNotExist;

            std::ofstream valueFile(valuePath);
            if (!valueFile.is_open())
                return Error::eFilesystemError;

            valueFile << valueMask[i];
        }

        return Error::eOk;
    }

    /// Reads the maximal pin number handled by this GPIO chip.
    /// @param chipInstance         GPIO chip instance name.
    /// @param maxPin               Output argument where the value will be stored.
    /// @return Error code of the operation.
    std::error_code maxHandledPin(const std::string& chipInstance, int& maxPin)
    {
        auto ngpioPath = detail::cSysFsGpioPath / chipInstance / "ngpio";
        if (!std::filesystem::exists(ngpioPath))
            return Error::ePathDoesNotExist;

        std::ifstream ngpioFile(ngpioPath);
        if (!ngpioFile.is_open())
            return Error::eFilesystemError;

        std::stringstream buffer;
        buffer << ngpioFile.rdbuf();
        buffer >> maxPin;

        return Error::eOk;
    }

private:
    PinMask<WidthType> m_validPins;
    PinMask<WidthType> m_usedPins;
};

} // namespace hal::gpio
