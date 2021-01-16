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

#include "platformInit.hpp"

#include <stm32f4xx.h>

#include <cstdint>
#include <type_traits>

// NOLINTNEXTLINE(fuchsia-statically-constructed-objects,cppcoreguidelines-avoid-non-const-global-variables)
UART_HandleTypeDef uart{};

extern "C" {

void HAL_MspInit()
{
    __HAL_RCC_SYSCFG_CLK_ENABLE(); // NOLINT
    __HAL_RCC_PWR_CLK_ENABLE();    // NOLINT
}

void HAL_UART_MspInit(UART_HandleTypeDef* /*unused*/)
{
    __HAL_RCC_GPIOC_CLK_ENABLE(); // NOLINT

    GPIO_InitTypeDef config{};
    config.Pin = GPIO_PIN_10;
    config.Mode = GPIO_MODE_AF_PP;
    config.Pull = GPIO_PULLUP;
    config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    config.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &config); // NOLINT
}

} // extern "C"

int consolePrint(const char* message, std::size_t size)
{
    constexpr std::uint32_t cTimeout = 1'000;
    auto result = HAL_UART_Transmit(&uart, std::remove_const_t<std::uint8_t*>(message), size, cTimeout);
    return (result == HAL_OK) ? int(size) : 0;
}

static bool consoleInitUart()
{
    __HAL_RCC_UART4_CLK_ENABLE(); // NOLINT

    uart.Instance = UART4; // NOLINT
    constexpr std::uint32_t cConsoleBaudrate = 115200;
    uart.Init.BaudRate = cConsoleBaudrate;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    auto result = HAL_UART_Init(&uart);
    return result == HAL_OK;
}

bool platformInit()
{
    if (HAL_Init() != HAL_OK)
        return false;

    return consoleInitUart();
}
