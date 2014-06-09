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
 *
 * @file
 * Main file where the code execution for the observer starts
 * @author
 * Prithvi
 */

/**
 * @dot
 * digraph main_flow {
 *	a [shape = oval, label = "On restart"]
 *	b [shape = record, label = <Initiation of the peripherals used<br></br><I>HF and LF clock<br></br>Profile Timer<br></br>Milli-second Timer<br></br>UART peripheral<br></br>Radio for continuously observing</I>>]
 *	c [shape = parallelogram, label = "Print the welcome message"]
 *	d [shape = box, label = "Set the UART receive handler"]
 *	e [shape = oval, label = "Sleep and wait for interrupt"]
 *
 *	a -> b -> c -> d -> e;
 *  }
 * @enddot
 */

#include "uart_nrf.h"
#include "leds.h"
#include "nrf.h"
#include "board.h"
#include "clock-nrf.h"
#include "clock-init.h"
#include "string.h"
#include "observer.h"
#include "timer.h"

/**
 * Main function where the peripherals' initialization takes place and the observation routine is started
 */
int main(void){
	hfclk_xtal_init();
	timer_init();
	lfclk_init();
	leds_init();
	ms_timer_init();
	obsvr_radio_init();
	uart_init();

	/* Enable the low latency mode */
	NRF_POWER->TASKS_CONSTLAT = 1;

    printf("Observer welcomes you. Send 'START' to initiate the observation and 'STOP' to terminate it.\n");

    obsvr_begin();

	while(true) {
	  __WFI();
	}
}
