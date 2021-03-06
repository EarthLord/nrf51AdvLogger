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
 * @defgroup irq-priority Interrupt Priority
 * Interrupt priority definitions for all the peripherals.
 * Having all the definitions at one place provides a single glance understanding
 * of the priority levels used by various peripherals
 * @{
 *
 * @file
 * This file contains the Interrupt priority definitions for all the peripherals
 *
 * @author
 * Prithvi
 */

#ifndef IRQ_PRIORITY_H_
#define IRQ_PRIORITY_H_

/** @anchor irq-priority-defs
 * @name Definitions of the interrupt priority of all the peripherals
 * If a peripheral does not use interrupts, its priority is defined without a value.
 * Used in @ref uart-driver, @ref rtc-timer and @ref hf-timer module
 * @{*/
#define PRIORITY_POWER_CLOCK_IRQn
#define PRIORITY_RADIO_IRQn      		3
#define PRIORITY_UART0_IRQn				4
#define PRIORITY_SPI0_TWI0_IRQn
#define PRIORITY_SPI1_TWI1_IRQn
#define PRIORITY_GPIOTE_IRQn
#define PRIORITY_ADC_IRQn
#define PRIORITY_TIMER0_IRQn
#define PRIORITY_TIMER1_IRQn
#define PRIORITY_TIMER2_IRQn
#define PRIORITY_RTC0_IRQn
#define PRIORITY_TEMP_IRQn
#define PRIORITY_RNG_IRQn
#define PRIORITY_ECB_IRQn
#define PRIORITY_CCM_AAR_IRQn
#define PRIORITY_WDT_IRQn
#define PRIORITY_RTC1_IRQn				2
#define PRIORITY_QDEC_IRQn
#define PRIORITY_LPCOMP_COMP_IRQn
#define PRIORITY_SWI0_IRQn
#define PRIORITY_SWI1_IRQn
#define PRIORITY_SWI2_IRQn
#define PRIORITY_SWI3_IRQn
#define PRIORITY_SWI4_IRQn
#define PRIORITY_SWI5_IRQn
/** @} */

#endif /* IRQ_PRIORITY_H_ */
/**
 * @}
 * @}
 */
