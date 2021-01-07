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

#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>

#include <cstddef>
#include <cstdint>

int consolePrint(const char* message, std::size_t size)
{
    for (std::size_t i = 0; i < size; ++i) {
        while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET) { // NOLINT
        }
        USART_SendData(UART4, message[i]); // NOLINT
    }

    return size;
}

static void consoleInitGpio()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4); // NOLINT

    GPIO_InitTypeDef config{};
    config.GPIO_Pin = GPIO_Pin_10;
    config.GPIO_Mode = GPIO_Mode_AF;
    config.GPIO_OType = GPIO_OType_PP;
    config.GPIO_PuPd = GPIO_PuPd_UP;
    config.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &config); // NOLINT
}

static void consoleInitUart()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    USART_InitTypeDef config{};
    constexpr std::uint32_t cConsoleBaudrate = 115200;
    config.USART_BaudRate = cConsoleBaudrate;
    config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    config.USART_Mode = USART_Mode_Tx;
    config.USART_Parity = USART_Parity_No;
    config.USART_StopBits = USART_StopBits_1;
    config.USART_WordLength = USART_WordLength_8b;
    USART_Init(UART4, &config); // NOLINT
    USART_Cmd(UART4, ENABLE);   // NOLINT
}

bool platformInit()
{
    consoleInitGpio();
    consoleInitUart();

    return true;
}
