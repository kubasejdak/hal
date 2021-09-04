/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2021, Kuba Sejdak <kuba.sejdak@gmail.com>
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
#include "hal/Error.hpp"
#include "hal/IBoard.hpp"

#include <map>
#include <memory>
#include <system_error>
#include <type_traits>

namespace hal {

/// Helper class for accessing currently used version/revision of the given board.
/// @tparam IdType          Type of the device identifier (usually related to the concrete board implementation).
/// @note Board version exists to represent/support idea of small changes applied to the same board. For example,
///       v1.0 is the initial board layout, while v1.3 has additional EEPROM added. However all other components are
///       the same. In such a case, we can reuse all device ids and only add missing components.
/// @note HAL assumes, that multiple versions of the same board can be compiled into the binary, but only one of them is
///       initialized nas used. This selection can be done on the Hardware::init() level.
template <typename IdType>
class CurrentBoardVersion {
public:
    /// Sets given board as currently used version.
    /// @param board        Board to be set as currently used.
    static void set(IBoard* board) { instance().m_currentVersion = board; }

    /// Returns currently used instance of the board.
    /// @return Currently used instance of the board.
    static IBoard* get() { return instance().m_currentVersion; }

private:
    /// Default constructor.
    CurrentBoardVersion() = default;

    /// Returns only instance of the CurrentBoardVersion<IdType>.
    /// @return Only instance of the CurrentBoardVersion<IdType>.
    static CurrentBoardVersion<IdType>& instance()
    {
        static CurrentBoardVersion<IdType> object;
        return object;
    }

private:
    IBoard* m_currentVersion{};
};

/// Board template which holds all its drivers in a map (id -> handle).
/// @tparam IdType          Type of the device identifier (usually related to the concrete board implementation).
/// @tparam cVersion        Version of the board in a form of integer sequence (e.g. 1, 3, 2 meaning v1.3.2).
/// @note This map is parametrized with the device id type, which is specific to the concrete board implementation.
///       Such boards have to only specialize Board::initImpl() and optionally Board::deinitImpl().
template <typename IdType, std::size_t... cVersion>
class Board : public IBoard {
    static_assert(std::is_enum_v<IdType>, "Board can be instantiated only with enum types");

public:
    /// Returns only instance of the Board singleton.
    /// @return Only instance of the Board singleton.
    static Board<IdType>& instance()
    {
        static Board<IdType> object;
        return object;
    }

private:
    /// @see IBoard::init().
    std::error_code init() override
    {
        if (auto error = initImpl())
            return error;

        for (auto& [id, device] : m_devices)
            setBoard(device);

        CurrentBoardVersion<IdType>::set(this);
        return Error::eOk;
    }

    /// @see IBoard::deinit().
    std::error_code deinit() override
    {
        for (const auto& [id, device] : m_devices) {
            if (device->ownersCount() != 0)
                return Error::eDeviceTaken;
        }

        m_devices.clear();
        auto error = deinitImpl();
        CurrentBoardVersion<IdType>::set(nullptr);
        return error;
    }

    /// @see IBoard::getDeviceImpl().
    std::shared_ptr<Device> getDeviceImpl(int id) override
    {
        auto deviceId = static_cast<IdType>(id);
        if (m_devices.find(deviceId) == std::end(m_devices))
            return nullptr;

        return m_devices[deviceId];
    }

    /// @see IBoard::returnDeviceImpl().
    std::error_code returnDeviceImpl(std::shared_ptr<Device>& /*unused*/) override { return Error::eOk; }

    /// Fills the device map with handles to the initialized device drivers.
    /// @return Error code of the operation.
    std::error_code initImpl();

    /// Deinitializes board after the device map is already cleared.
    /// @return Error code of the operation.
    /// @note This method will usually destroy all global drivers, which are held in GlobalRegistry.
    std::error_code deinitImpl() { return Error::eOk; }

private:
    std::map<IdType, std::shared_ptr<Device>> m_devices;
};

} // namespace hal
