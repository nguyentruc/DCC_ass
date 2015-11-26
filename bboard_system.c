/*
 * Copyright (C) 2015 Ho Chi Minh city University of Technology (HCMUT)
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License.
 */

/**
 * @file        bboard_system.c
 * @brief       This is the entry point of Bboard library
 *
 * @author      Nhat Pham <phamhuudangnhat@gmail.com>
 */
#include <stdarg.h>
#include <string.h>
#include "bboard_system.h"

/* USART1 handle */
UART_HandleTypeDef bboard_uart1_handle;

/* USART2 handle */
UART_HandleTypeDef bboard_uart2_handle;

/*----------------------------------------------------------------------------*/
void bboard_usart1_init(uint32_t buadrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    
    /* Enable USART1, GPIOA clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    
    /* Configure PA9 (TxD), PA10 (RxD) in AF push-pull mode to drive by USART1 */
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure the UART peripheral */
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
      - Word Length = 8 Bits (8 data bit + 0 parity bit)
      - Stop Bit    = One Stop bit
      - BaudRate    = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
    bboard_uart1_handle.Instance        = USART1;

    bboard_uart1_handle.Init.BaudRate   = buadrate;
    bboard_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;
    bboard_uart1_handle.Init.StopBits   = UART_STOPBITS_1;
    bboard_uart1_handle.Init.Parity     = UART_PARITY_NONE;
    bboard_uart1_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    bboard_uart1_handle.Init.Mode       = UART_MODE_TX_RX;
    
    HAL_UART_Init(&bboard_uart1_handle);
}

/*----------------------------------------------------------------------------*/
void bboard_usart2_init(uint32_t buadrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    
    /* Enable USART1, GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    
    /* Configure PA2 (TxD), PA3 (RxD) in AF push-pull mode to drive by USART2 */
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure the UART peripheral */
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
      - Word Length = 8 Bits (8 data bit + 0 parity bit)
      - Stop Bit    = One Stop bit
      - BaudRate    = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
    bboard_uart2_handle.Instance        = USART2;

    bboard_uart2_handle.Init.BaudRate   = buadrate;
    bboard_uart2_handle.Init.WordLength = UART_WORDLENGTH_8B;
    bboard_uart2_handle.Init.StopBits   = UART_STOPBITS_1;
    bboard_uart2_handle.Init.Parity     = UART_PARITY_NONE;
    bboard_uart2_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    bboard_uart2_handle.Init.Mode       = UART_MODE_TX_RX;
    
    HAL_UART_Init(&bboard_uart2_handle);
}

/*----------------------------------------------------------------------------*/
void bboard_usart1_printf(const char *format, ...)
{
    uint8_t buffer[256];
	va_list args;

	va_start(args, format);
	vsnprintf((char*)buffer, 256, format, args);
	va_end(args);

	/* print buffer to UART */
    HAL_UART_Transmit(  &bboard_uart1_handle, buffer, 
                        strlen((char *)buffer), 0xFFFF);
}

/*----------------------------------------------------------------------------*/
int sendchar(int ch) {
    uint8_t temp = (char) ch;
    HAL_UART_Transmit(&bboard_uart2_handle, (uint8_t *)&temp, 1, HAL_MAX_DELAY);
    
    return ch;
}

/*----------------------------------------------------------------------------*/
int getkey(void) {
    uint8_t temp;
    HAL_UART_Receive(&bboard_uart2_handle, (uint8_t *)&temp, 1, HAL_MAX_DELAY);
    
    if (temp == '\r') {
        return EOF;
    }
    
    return (int) temp;
}
