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

#include "hal/Device.hpp"

#include <memory>

namespace hal {
namespace device_id {

/// Represents identifiers of the devices provided by the q39tester-set1 board.
enum Q39TesterSet1Id {
    ////////////////////////////////////////////////////////////////////
    // GPIO.
    ////////////////////////////////////////////////////////////////////

    // MCP23S17 Port A.
    eMcp23s17PinInPA0,
    eMcp23s17PinInPA1,
    eMcp23s17PinInPA2,
    eMcp23s17PinInPA3,
    eMcp23s17PinOutPA4,
    eMcp23s17PinOutPA5,
    eMcp23s17PinOutPA6,
    eMcp23s17PinOutPA7,

    eMcp23s17PortOut4BPA0u3,
    eMcp23s17PortIn4BPA4u7,

    eMcp23s17PingPongOutPA0,
    eMcp23s17PingPongInPA1,
    eMcp23s17TriggerOutPA2,
    eMcp23s17CounterInPA3u7,

    // MCP23S17 Port B.
    eMcp23s17PinOutPB0,
    eMcp23s17PinOutPB1,
    eMcp23s17PinOutPB2,
    eMcp23s17PinOutPB3,
    eMcp23s17PinInPB4,
    eMcp23s17PinInPB5,
    eMcp23s17PinInPB6,
    eMcp23s17PinInPB7,

    eMcp23s17PortIn4BPB0u3,
    eMcp23s17PortOut4BPB4u7,

    eMcp23s17PingPongInPB0,
    eMcp23s17PingPongOutPB1,
    eMcp23s17TriggerInPB2,
    eMcp23s17CounterOutPB3u7,

    // MCP23017 Port A.
    eMcp23017PinOutPA0,
    eMcp23017PinOutPA1,
    eMcp23017PinOutPA2,
    eMcp23017PinOutPA3,
    eMcp23017PinInPA4,
    eMcp23017PinInPA5,
    eMcp23017PinInPA6,
    eMcp23017PinInPA7,

    eMcp23017PortIn4BPA0u3,
    eMcp23017PortOut4BPA4u7,

    eMcp23017PingPongInPA0,
    eMcp23017PingPongOutPA1,
    eMcp23017TriggerInPA2,
    eMcp23017CounterOutPA3u7,

    // MCP23017 Port B.
    eMcp23017PinInPB0,
    eMcp23017PinInPB1,
    eMcp23017PinInPB2,
    eMcp23017PinInPB3,
    eMcp23017PinOutPB4,
    eMcp23017PinOutPB5,
    eMcp23017PinOutPB6,
    eMcp23017PinOutPB7,

    eMcp23017PortOut4BPB0u3,
    eMcp23017PortIn4BPB4u7,

    eMcp23017PingPongOutPB0,
    eMcp23017PingPongInPB1,
    eMcp23017TriggerOutPB2,
    eMcp23017CounterInPB3u7,

    ////////////////////////////////////////////////////////////////////
    // Storage.
    ////////////////////////////////////////////////////////////////////

    eAt24Cm02Eeprom1,
    eAt24Cm02Eeprom2,

    ////////////////////////////////////////////////////////////////////
    // RTC.
    ////////////////////////////////////////////////////////////////////

    eM41T82Rtc
};

} // namespace device_id

/// Returns device handle associated with the given device id from q39tester-set1 board.
/// @tparam T           Type of the device to be returned.
/// @param id           Identifier of the device to be returned.
/// @return Device handle casted to the given type T and associated with the given device id.
/// @note If there is no such id registered in the board or handle has been retrieved maximal times, then nullptr
///       will be returned.
/// @note Caller has to use the correct T type in order to get the valid handle.
template <typename T>
std::shared_ptr<T> getDevice(device_id::Q39TesterSet1Id id)
{
    return std::dynamic_pointer_cast<T>(detail::getDeviceImpl<decltype(id)>(id));
}

} // namespace hal
