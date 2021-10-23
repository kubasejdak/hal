/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Grzegorz Heldt
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

#include <map>
#include <system_error>

namespace hal {

class IBoard;

/// Provides the managing capabilities to control HAL creation and destruction for the client application.
class Hardware {
public:
    /// Initializes all registered base boards.
    /// @return Error code of the operation.
    static std::error_code init();

    /// Initializes all registered removable boards.
    /// @return Error code of the operation.
    static std::error_code attach();

    /// Deinitializes all registered removable boards.
    /// @return Error code of the operation.
    static std::error_code detach();

    /// Deinitializes all registered base boards.
    /// @return Error code of the operation.
    static std::error_code destroy();

private:
    /// Default constructor.
    Hardware() = default;

    /// Creates and registers concrete boards in the c
    static void createBoards();

private:
    /// Represents the internal state of the Hardware.
    enum class State
    {
        eUninitialized,
        eAttached,
        eDetached
    };

    /// Represents type of the board managed by Hardware. Base boards have to always be present and initialized.
    /// Removable boards can be destroyed and re-initialized at any time in runtime.
    /// @note Base boards are usually the ones containing the CPU.
    enum class Type
    {
        eBase,
        eRemovable
    };

    static inline State m_state = State::eUninitialized; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    static inline std::multimap<Type, IBoard&> m_boards; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
};

/// Helper RAII type for managing board initialization via Hardware.
class ScopedHardware {
public:
    /// Constructor. Automatically initializes all base and removable boards registered in Hardware.
    ScopedHardware()
    {
        if (Hardware::init())
            return;

        if (Hardware::attach()) {
            Hardware::destroy();
            return;
        }

        m_initialized = true;
    }

    /// Copy constructor.
    /// @note This constructor is deleted, because ScopedHardware is not meant to be copy-constructed.
    ScopedHardware(const ScopedHardware&) = delete;

    /// Move constructor.
    /// @note This constructor is deleted, because ScopedHardware is not meant to be move-constructed.
    ScopedHardware(ScopedHardware&&) noexcept = delete;

    /// Destructor. Automatically deinitializes all base and removable boards registered in Hardware.
    ~ScopedHardware()
    {
        Hardware::detach();
        Hardware::destroy();
    }

    /// Copy assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because ScopedHardware is not meant to be copy-assigned.
    ScopedHardware& operator=(const ScopedHardware&) = delete;

    /// Move assignment operator.
    /// @return Reference to self.
    /// @note This operator is deleted, because ScopedHardware is not meant to be move-assigned.
    ScopedHardware& operator=(ScopedHardware&&) noexcept = delete;

    /// Returns flag indicating if Hardware has been initialized.
    /// @return Flag indicating if Hardware has been initialized.
    /// @retval true            Hardware has been initialized.
    /// @retval false           Hardware has not been initialized.
    [[nodiscard]] bool initialized() const { return m_initialized; }

private:
    bool m_initialized{};
};

} // namespace hal
