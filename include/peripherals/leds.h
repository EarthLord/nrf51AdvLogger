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
 * @defgroup led-driver LED Driver
 * LED Driver for the PCA1000 board to initialize, switch on, switch off and toggle LEDs
 * @{
 *
 * @file
 * This file contains the LED Driver for the PCA1000 board
 * @author
 * Prithvi
 */

#ifndef LEDS_H_
#define LEDS_H_

#include "board.h"
#include "nrf_gpio.h"

/**@brief Function to initialize the LEDS in the PCA10000 board
 */
static __INLINE void leds_init(void){
  nrf_gpio_cfg_output(LED_RGB_RED);
  nrf_gpio_pin_set(LED_RGB_RED);
  nrf_gpio_cfg_output(LED_RGB_GREEN);
  nrf_gpio_pin_set(LED_RGB_GREEN);
  nrf_gpio_cfg_output(LED_RGB_BLUE);
  nrf_gpio_pin_set(LED_RGB_BLUE);
}

/**
 * Turn a specific LED on
 * @param led The pin number of the LED to turn on
 */
static __INLINE void leds_on(uint32_t led){
	  nrf_gpio_pin_clear(led);
}

/**
 * Turn a specific LED of
 * @param led The pin number of the LED to turn off
 */
static __INLINE void leds_off(uint32_t led){
	  nrf_gpio_pin_set(led);
}

/**
 * Toggle a specific LED
 * @param led The pin number of the LED to toggle
 */
static __INLINE void leds_toggle(uint32_t led){
	  nrf_gpio_pin_toggle(led);
}

#endif /* LEDS_H_ */
/**
 * @}
 * @}
 */
