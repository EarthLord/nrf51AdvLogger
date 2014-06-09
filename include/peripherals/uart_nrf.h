/* 	Copyright (c) 2014, Prithvi Raj Narendra
 *	All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without modification,
 *	are permitted provided that the following conditions are met:
 *
 *	1. Redistributions of source code must retain the above copyright notice,
 *	this list of conditions and the following disclaimer.
 *
 *	2. Redistributions in binary form must reproduce the above copyright notice,
 *	this list of conditions and the following disclaimer in the documentation
 *	and/or other materials provided with the distribution.
 *
 *	3. Neither the name of the copyright holder nor the names of its contributors
 *	may be used to endorse or promote products derived from this software without
 *	specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *	IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *	INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *	OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *	WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *	POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @addtogroup peripheral-drivers
 * @{
 *
 * @defgroup uart-driver UART Driver
 * UART Driver for the nrf51822 SoC
 * supports single character printing, stream to printf and a character stream receiver handler
 * @{
 *
 * @file
 * This file contains the UART Driver declarations for the nrf51822 SoC
 * @author
 * Prithvi
 */

#ifndef UART_NRF_H
#define UART_NRF_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/** The character that is checked by @ref rx_collect to determine if a line of characters is received */
#define LINE_END				'\n'
/** Baudrate to be used by the UART module. Used by @ref uart_init.
 * In PCA10000 the maximum baudrate is 4,60,800 bps */
#define UART_BAUDRATE 			UART_BAUDRATE_BAUDRATE_Baud460800

/**
 * Function to add rx_data to a FIFO buffer and calls the receiver handler on @ref LINE_END
 * @param rx_data Data passed from the receive interrupt routine of UART
 */
void rx_collect(uint8_t rx_data);

/**
 * Function to set the handler to be called by @ref rx_collect when a line is received by UART
 * @param handler Pointer to the function to be called when a line is received by UART
 */
void set_rx_handler (void (*handler) (uint8_t * ptr) );

/**
 * Function to initialize the parameters of UART based on the configurations in @ref board.h
 */
void uart_init(void);

/**
 * @brief Send a single character through UART
 * This function is used by @ref _write so that printf can be used
 * @param cr The character to be sent
 */
void uart_putchar(uint8_t cr);

#endif
/**
 * @}
 * @}
 */
