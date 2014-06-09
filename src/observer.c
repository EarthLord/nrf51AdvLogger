/* 	Copyright (c) 2014, Prithvi Raj Narendra
 *  Copyright (c) 2013 Paulo Sérgio Borges de Oliveira Filho
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
 * @addtogroup adv-observer
 * @{
 *
 * @file
 * File containing the implementation of the BLE advertisement packets observer
 *
 * @author
 * Prithvi
 */

#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf51_deprecated.h"
#include "irq_priority.h"
#include "string.h"
#include "clock-nrf.h"
#include "observer.h"
#include "timer.h"

/** @anchor adv-seg-type
 *  @name Advertisement segment types
 *  @{
 */
#define TYPE_FLAGS                      0x1
#define TYPE_UUID16_INC                 0x2
#define TYPE_UUID16                     0x3
#define TYPE_UUID32_INC                 0x4
#define TYPE_UUID32                     0x5
#define TYPE_UUID128_INC                0x6
#define TYPE_UUID128                    0x7
#define TYPE_NAME_SHORT                 0x8
#define TYPE_NAME                       0x9
#define TYPE_TRANSMITPOWER              0xA
#define TYPE_CONNINTERVAL               0x12
#define TYPE_SERVICE_SOLICITATION16     0x14
#define TYPE_SERVICE_SOLICITATION128    0x15
#define TYPE_SERVICEDATA                0x16
#define TYPE_APPEARANCE                 0x19
#define TYPE_MANUFACTURER_SPECIFIC      0xFF
/** @} */

/** @anchor adv-packet-type
 *  @name Advertisement packet types
 *  @{
 */
#define TYPE_ADV_IND            0
#define TYPE_ADV_DIRECT_IND     1
#define TYPE_ADV_NONCONN_IND    2
#define TYPE_SCAN_REQ           3
#define TYPE_SCAN_RSP           4
#define TYPE_CONNECT_REQ        5
#define TYPE_ADV_SCAN_IND       6
/** @} */

/** Return 2^n, used for setting nth bit as 1*/
#define SET_BIT(n)      (1UL << n)

/** Maximum PDU size of Adv packets is 39 octets, defined as 40 to make it word alligned  */
#define MAX_PDU_SIZE    40

/** Maximum number of packets logged in a scan interval */
#define LOGGER_NUMBER		20

/** Period (ms) in which the radio is on and scanning on a particular channel during @ref SCAN_INTERVAL */
#define SCAN_WINDOW  	750
/** Total time in a cycle where the radio is on and off */
#define SCAN_INTERVAL  	1000

/* Check that there is enough time to send the info about the packets of last scan window */
#if (SCAN_INTERVAL - 50) < SCAN_WINDOW
#error Scan window must be atleast 50 ms greater than scan interval.
#endif

/** Packet data unit, the array where the radio dumps the data it receives */
static uint8_t pdu[MAX_PDU_SIZE];

/** Two dimensional array of packets received in one @ref SCAN_WINDOW.
 * The number of packets that can be stored is @ref LOGGER_NUMBER.
 * \todo Use dynamic memory with check if still within heap */
static uint8_t packets[LOGGER_NUMBER][MAX_PDU_SIZE+8];

/** Variable to store the signal strength of the received packet in the Interrupt routine */
static uint32_t rssi = 0;

/** Number of packets received in the last @ref SCAN_WINDOW */
static uint32_t packet_count = 0;

/** Handler of the repetitive timer with an interval of @ref SCAN_INTERVAL.
 *  This function configures the radio to scan in one of the advertising channel
 *  and starts a single shot timer with interval of @ref SCAN_WINDOW
 */
static void obsvr_interval(void);

/** Handler of the single shot timer started in @ref obsvr_interval with interval of @ref SCAN_WINDOW.
 *	This function disables the radio and prints the information about the collected packets.
 */
static void obsvr_window(void);

/** Collect the data from pdu array at the end of every received packet.
 * Called from the interrupt to copy the received data from @ref pdu.
 */
static void collect_packet(void);

/**
 * Print the packets collected during the @ref SCAN_INTERVAL
 * @param packet_to_print
 */
static void print_collected(uint8_t * packet_to_print);

/** Handler which is called when a string is sent through UART.
 * Used for initiating and terminating the BLE advertisement packet observation.
 * @param ptr The unsigned character pointer to the string received through UART
 */
static void obsvr_req(uint8_t * ptr);

/** Prints the information about the various segments of an advertisement packet
 * @param ptr	pointer to the payload of an advertisement packet
 * @param len	length of the payload of an advertisement packet
 */
static void adv_seg_print(uint8_t * ptr, uint32_t len);

/** @brief Function for handling the Radio interrupts.
 *
 * @dot
 * digraph radio_isr{
 *  start [shape=oval, label="Start of the Radio's interrupt routine"]
 *  is_end [shape=diamond, label="Is it a packet end event?"]
 *  is_rssiend [shape=diamond, label="Is it a RSSI measure end event?"]
 *  check_ptr [shape=diamond, label="Is there still space in the buffer?"]
 *  collect [shape = box, label="1. Copy the contents of the received packet, RSSI value,
Channel number, CRC status and time-stamp to the buffer
2. Increment the received packet pointer"]
 *  rssi [shape = box, label="Store the measured RSSI value"]
 *  return [shape = oval, label="Return from the interrupt routine"]
 *  start -> is_end;
 *  is_end -> check_ptr [label ="true"];
 *  check_ptr -> collect [label ="true"]
 *  check_ptr -> is_rssiend [label ="false"];
 *  is_end -> is_rssiend [label ="false"];
 *  is_rssiend -> rssi [label ="true"];
 *  is_rssiend -> return [label="false"];
 *  collect -> is_rssiend;
 *  rssi -> return;
 * }
 * @enddot
 */
void RADIO_IRQHandler(){
    if((NRF_RADIO->EVENTS_END == 1) && (NRF_RADIO->INTENSET & RADIO_INTENSET_END_Msk)){
		NRF_RADIO->EVENTS_END = 0;
		collect_packet();
	}

 	if((NRF_RADIO->EVENTS_RSSIEND == 1) && (NRF_RADIO->INTENSET & RADIO_INTENSET_RSSIEND_Msk)){
		NRF_RADIO->EVENTS_RSSIEND = 0;
		NRF_RADIO->TASKS_RSSISTOP = 1;
		rssi = (NRF_RADIO->RSSISAMPLE);
	}
}


static void collect_packet(void){
	/* Collect a max of LOGGER_NUMBER packets, otherwise reduce observer window */
	if(packet_count < LOGGER_NUMBER){
		uint32_t time = read_time_us();
		memcpy(*(packets + packet_count), pdu, pdu[1]+2);
		memcpy(*(packets + packet_count) + MAX_PDU_SIZE, &time, 4);	//sizeof(time)=4
		packets[packet_count][MAX_PDU_SIZE+4] = NRF_RADIO->DATAWHITEIV - 64;  //6th bit is always 1
		if(NRF_RADIO->CRCSTATUS == 1){
			packets[packet_count][MAX_PDU_SIZE+4] |= 0x80;
		}
		packets[packet_count][MAX_PDU_SIZE+5] = rssi;
		packet_count++;
	}
}

static void adv_seg_print(uint8_t * ptr, uint32_t len){
	uint32_t index = 0;
	while (index < len) {
		uint32_t seg_len = ptr[index];
		index++;

		switch (ptr[index]) {
			case 0: printf("Type Zero is undefined, Error.");
				break;
			case TYPE_FLAGS:
				printf("Flags:");
				break;
			case TYPE_NAME_SHORT:
			case TYPE_NAME:
				printf("Name:");
				break;
			case TYPE_UUID128:
			case TYPE_UUID128_INC:
			case TYPE_UUID32:
			case TYPE_UUID32_INC:
			case TYPE_UUID16:
			case TYPE_UUID16_INC:
				printf("UUIDs:");
				break;
			case TYPE_TRANSMITPOWER:
				printf("Transmit Power:");
				break;
			case TYPE_CONNINTERVAL:
				printf("Connect Interval:");
				break;
			case TYPE_SERVICE_SOLICITATION16:
			case TYPE_SERVICE_SOLICITATION128:
				printf("Service Solicited:");
				break;
			case TYPE_SERVICEDATA:
				printf("Service Data:");
				break;
			case TYPE_MANUFACTURER_SPECIFIC:
				printf("Manufacturer Specific Data:");
				break;
			case TYPE_APPEARANCE:
				printf("Appearance:");
				break;
			default:
				printf("Unknown type:%02X",ptr[index]);
		}

		 switch (ptr[index]) {
		 	case 0: break;
			case TYPE_TRANSMITPOWER:
				printf("%d|",(int)ptr[index+1]);
				break;
			case TYPE_NAME_SHORT:
			case TYPE_NAME:
				printf("%.*s|",(int) seg_len-1,ptr+index+1);
				break;
			case TYPE_SERVICEDATA:
				printf("UUID:0x%02X%02X|",ptr[index+2],ptr[index+1]);
				printf("Data:");
				for(uint32_t i = 3; i<seg_len; i++){
					printf("%02X ",ptr[index+i]);
				}
				printf("|");
				break;
			case TYPE_UUID128:
			case TYPE_UUID128_INC:
			case TYPE_UUID32:
			case TYPE_UUID32_INC:
			case TYPE_UUID16:
			case TYPE_UUID16_INC:
				printf("UUID:");
				for(uint32_t i = seg_len-1; i>0; i--){
					printf("%02X",ptr[index+i]);
				}
				printf("|");
				break;
			case TYPE_MANUFACTURER_SPECIFIC:
			case TYPE_CONNINTERVAL:
		    /** \todo Display flag information based on the following bits\n
			 *  0 LE Limited Discoverable Mode\n
			 *	1 LE General Discoverable Mode\n
			 *	2 BR/EDR Not Supported\n
			 *	3 Simultaneous LE and BR/EDR to Same Device is Capable (Controller)\n
			 *	4 Simultaneous LE and BR/EDR to Same Device is Capable (Host)\n
			 *	5..7 Reserved
			 */
			case TYPE_FLAGS:
			case TYPE_SERVICE_SOLICITATION16:
			case TYPE_SERVICE_SOLICITATION128:
			case TYPE_APPEARANCE:
			default:
				for(uint32_t i = 1; i<seg_len; i++){
					printf("%02X",ptr[index+i]);
				}
				printf("|");
		 }

		index += seg_len;
	}
}

static void print_collected(uint8_t * packet_to_print){
	uint32_t packet_time;
	memcpy(&packet_time,packet_to_print + MAX_PDU_SIZE,4);
	printf("Time:");
	printfcomma(packet_time);
	printf("us|");
	printf("Channel:%d|", (int) packet_to_print[MAX_PDU_SIZE+4]&0x7F);
	printf("RSSI:%d|", (int) packet_to_print[MAX_PDU_SIZE+5]);
	printf("CRC:%s|", (packet_to_print[MAX_PDU_SIZE+4]&0x80) ? "Y" : "N");

	/*  0000 		ADV_IND
		0001 		ADV_DIRECT_IND
		0010 		ADV_NONCONN_IND
		0011 		SCAN_REQ
		0100 		SCAN_RSP
		0101 		CONNECT_REQ
		0110 		ADV_SCAN_IND
		0111-1111 	Reserved 	*/
//	printf("Type:P%d ",(int) (packet_to_print[0]) & 0xF);
	printf("Type:");
	switch((packet_to_print[0]) & 0xF){
		case TYPE_ADV_IND: printf("ADV_IND|");
				break;
		case TYPE_ADV_DIRECT_IND: printf("ADV_DIRECT_IND|");
				break;
		case TYPE_ADV_NONCONN_IND: printf("ADV_NONCONN_IND|");
				break;
		case TYPE_SCAN_REQ: printf("SCAN_REQ|");
				break;
		case TYPE_SCAN_RSP: printf("SCAN_RSP|");
				break;
		case TYPE_CONNECT_REQ: printf("CONNECT_REQ|");
				break;
		case TYPE_ADV_SCAN_IND: printf("ADV_SCAN_IND|");
				break;
		default:printf("Reserved|");
	}

	/* 1: Random Tx address, 0: Public Tx address */
	printf("TxAdrs:%s|",(int) (packet_to_print[0] & 0x40)?"Random":"Public");
	/* 1: Random Rx address, 0: Public Rx address */
	printf("RxAdrs:%s|",(int) (packet_to_print[0] & 0x80)?"Random":"Public");
	/*	Length of the data packet */
	printf("Len:%d|",(int) packet_to_print[1] & 0x3F);

	uint8_t i;
	uint8_t f = packet_to_print[1] + 2;
	printf("Adrs:");
	for(i = 7; i>2; i--){
		printf("%02X:", packet_to_print[i]);
	}printf("%02X", packet_to_print[2]);
	printf("\n");

	if(packet_to_print[MAX_PDU_SIZE+4]&0x80){
		printf("Packet:");
		for (i = 0; i < f; i++) {
			printf("%02X ", packet_to_print[i]);
		}
		printf("\n");
		uint32_t type_temp = (packet_to_print[0] & 0x7);
		switch(type_temp){
			case TYPE_ADV_IND:
			case TYPE_ADV_NONCONN_IND:
			case TYPE_SCAN_RSP:
			case TYPE_ADV_SCAN_IND:
				adv_seg_print(packet_to_print+8,f-8);
				printf("\n");
				break;
			default: break;
		}
	}else{
		printf("Data corrupted. CRC check failed.\n");
	}
}

void obsvr_radio_init(){

	/* Refresh all the registers in RADIO */
	NRF_RADIO->POWER = 0;
	NRF_RADIO->POWER = 1;

    /* Set RADIO mode to Bluetooth Low Energy. */
    NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos;

    /* Copy the BLE override registers from FICR */
    NRF_RADIO->OVERRIDE0 = 	NRF_FICR->BLE_1MBIT[0];
    NRF_RADIO->OVERRIDE1 = 	NRF_FICR->BLE_1MBIT[1];
    NRF_RADIO->OVERRIDE2 = 	NRF_FICR->BLE_1MBIT[2];
    NRF_RADIO->OVERRIDE3 = 	NRF_FICR->BLE_1MBIT[3];
    NRF_RADIO->OVERRIDE4 = 	NRF_FICR->BLE_1MBIT[4];


    /* Set access address to 0x8E89BED6. This is the access address to be used
     * when send packets in obsvrertise channels.
     *
     * Since the access address is 4 bytes long and the prefix is 1 byte long,
     * we first set the base address length to be 3 bytes long.
     *
     * Then we split the full access address in:
     * 1. Prefix0:  0x0000008E (LSB -> Logic address 0)
     * 2. Base0:    0x89BED600 (3 MSB)
     *
     * At last, we enable reception for this address.
     */
    NRF_RADIO->PCNF1        |= 3UL << RADIO_PCNF1_BALEN_Pos;
    NRF_RADIO->BASE0        = 0x89BED600;
    NRF_RADIO->PREFIX0      = 0x0000008E;
    NRF_RADIO->RXADDRESSES  = 0x00000001;

    /* Enable data whitening. */
    NRF_RADIO->PCNF1 |= RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos;

    /* Set maximum PAYLOAD size. */
    NRF_RADIO->PCNF1 |= MAX_PDU_SIZE << RADIO_PCNF1_MAXLEN_Pos;

    /* Configure CRC.
     *
     * First, we set the length of CRC field to 3 bytes long and ignore the
     * access address in the CRC calculation.
     *
     * Then we set CRC initial value to 0x555555.
     *
     * The last step is to set the CRC polynomial to
     * x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1.
     */
    NRF_RADIO->CRCCNF =     RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos |
                            RADIO_CRCCNF_SKIP_ADDR_Skip
                                            << RADIO_CRCCNF_SKIP_ADDR_Pos;
    NRF_RADIO->CRCINIT =    0x555555UL;
    NRF_RADIO->CRCPOLY =    SET_BIT(24) | SET_BIT(10) | SET_BIT(9) |
                            SET_BIT(6) | SET_BIT(4) | SET_BIT(3) |
                            SET_BIT(1) | SET_BIT(0);

    /* Configure header size.
     *
     * The Advertise has the following format:
     * RxAdd(1b) | TxAdd(1b) | RFU(2b) | PDU Type(4b) | RFU(2b) | Length(6b)
     *
     * And the nRF51822 RADIO packet has the following format
     * (directly editable fields):
     * S0 (0/1 bytes) | LENGTH ([0, 8] bits) | S1 ([0, 8] bits)
     *
     * We can match those fields with the Link Layer fields:
     * S0 (1 byte)      --> PDU Type(4bits)|RFU(2bits)|TxAdd(1bit)|RxAdd(1bit)
     * LENGTH (6 bits)  --> Length(6bits)
     * S1 (0 bits)      --> S1(0bits)
     */
    NRF_RADIO->PCNF0 |= (1 << RADIO_PCNF0_S0LEN_Pos) |  /* 1 byte */
                        (8 << RADIO_PCNF0_LFLEN_Pos) |  /* 6 bits */
                        (0 << RADIO_PCNF0_S1LEN_Pos);   /* 2 bits */

    /* Set the pointer to write the incoming packet. */
    NRF_RADIO->PACKETPTR = (uint32_t) pdu;

    /* Set the initial freq to obsvr as channel 37 */
	NRF_RADIO->FREQUENCY = 80;//ADV_CHANNEL[3];

    /* Configure the shorts for observing
     * READY event and START task
     * ADDRESS event and RSSISTART task
     * END event and START task
     * */
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
    					RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
    					RADIO_SHORTS_END_START_Msk;

    NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk |
    					  RADIO_INTENSET_RSSIEND_Msk |
    					  RADIO_INTENSET_END_Msk;

    // Enable Interrupt for RADIO in the core.
    NVIC_SetPriority(RADIO_IRQn, PRIORITY_RADIO_IRQn);
	NVIC_EnableIRQ(RADIO_IRQn);
}

/**
 * @dot
 * digraph scan_window{
 *  start [shape = oval,label ="Function called at the \nend of a scan window"]
 * 	led_off [shape = box,label ="1. Disable the radio
2. Print the information of all the
packets received based on the counter
3. Switch off the red LED"]
 * 	stop [shape = oval, label ="Return"]
 * 	start->led_off->stop;
 * }
 * @enddot
 */
void obsvr_window(void){
	NRF_RADIO->EVENTS_DISABLED = 0UL;
	NRF_RADIO->TASKS_DISABLE = 1UL;
	while (NRF_RADIO->EVENTS_DISABLED == 0UL);
	if(packet_count){
		printf("Scan window's packet(s)=\n");
		for(uint32_t i = 0; i< packet_count; i++){
			print_collected(*(packets + i));
		}
		printf("\n");
	}else{
		printf("No packets in this scan interval\n\n");
	}
	leds_off(LED_RGB_RED);
}

/**
 * @dot
 * digraph scan_interval{
 *  start [shape = oval,label ="Function called at the \nbeginning of a scan interval"]
 * 	led_on [shape = box,label ="1. Switch on the red LED
2. Reset the received packet counter
3. Cyclically switch the radio to the next advertisement channel
4. Switch on the radio reception
5. Start the one shot timer with the length of the scan window"]
 * 	stop [shape = oval, label ="Return"]
 * 	start->led_on->stop;
 * }
 * @enddot
 */

void obsvr_interval(void){
	leds_on(LED_RGB_RED);
	packet_count = 0;

	uint32_t temp = NRF_RADIO->FREQUENCY;
	temp = (temp == 2)?26 : ((temp == 26)?80:2);
	NRF_RADIO->FREQUENCY = temp;
	NRF_RADIO->DATAWHITEIV = (temp == 2)?37 : ((temp == 26)?38:39);

	NRF_RADIO->EVENTS_READY = 0UL;
	NRF_RADIO->TASKS_RXEN = 1UL;

	start_ms_timer(MS_TIMER1, SINGLE_CALL, RTC_TICKS(SCAN_WINDOW), &obsvr_window);
}

/**
 * @dot
 * digraph uart_handler_flow {
 *	a [shape = oval, label = "Called with unsigned char pointer as argument"]
 *	b [shape = diamond, label ="Is the received string 'START?"]
 *	c [shape = diamond, label ="Is the received string 'STOP?"]
 *	d [shape = diamond, label ="Is the Scan-Window Timer running?"]
 *	g [shape = box, label ="Start the repetitive timer with duration of scan window "]
 *	e [shape = box, label ="1. Disable the Radio\n2. Stop the Scan-Window Timer\n3. Stop the Scan-Interval Timer"]
 *	f [shape = oval, label ="Return"]
 *	a -> b;
 *	b -> c [label ="false"];
 *	b -> d [label ="true"];
 *	c -> e [label ="true"];
 *	c -> f [label ="false"];
 *	d -> g [label ="false"];
 *	d -> f [label ="true"];
 *	e -> f;
 *	g -> f;
 * }
 * @enddot
 */
static void obsvr_req(uint8_t * ptr){

	/* Check if the string starts with 'ST' */
	if(strncmp((const char *)ptr,"ST",2)==0){
		/* if 'START' is received, start the scan_window repetitive timer */
		if(strcmp((const char *)ptr+2,"ART")==0){
			if(is_ms_timer_on(MS_TIMER0) == false){
				start_ms_timer(MS_TIMER0, REPEATED_CALL, RTC_TICKS(SCAN_INTERVAL), &obsvr_interval);
			}
		}
		/* if 'STOP' is received, disable the Radio and stop both the timers*/
		else if(strcmp((const char *)ptr+2,"OP")==0){
			NRF_RADIO->EVENTS_DISABLED = 0UL;
			NRF_RADIO->TASKS_DISABLE = 1UL;
			while (NRF_RADIO->EVENTS_DISABLED == 0UL);

			stop_ms_timer(MS_TIMER0);
			stop_ms_timer(MS_TIMER1);
		}
	}
}

void obsvr_begin(){
	set_rx_handler(&obsvr_req);
}

/**
 * @}
 */
