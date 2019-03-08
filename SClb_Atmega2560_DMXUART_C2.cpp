/*
 Name:		SClb_Atmega2560_DMXUART_C2.cpp
 Created:	08-Mar-19 16:19:33
 Author:	statuscue
 Editor:	http://hes.od.ua
*/

/*
	Copyright 2019-2019 by Yevhen Mykhailov
	Art-Net(TM) Designed by and Copyright Artistic Licence Holdings Ltd.
*/

#include "SClb_Atmega2560_DMXUART_C2.h"

#include "pins_arduino.h"
#include <inttypes.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>

SClb_Atmega2560_DMXUART_C2 SCDMXUSART2;

#define SCUCSRA_U2 UCSR2A							// USART register A
#define SCTXC_U2   TXC2								// tx buffer empty
#define SCUDRE_U2  UDRE2							// data ready
#define SCFE_U2    FE2								// frame error
#define SCU2X_U2   U2X2								// double speed

#define SCUCSRB_U2 UCSR2B							// USART register B
#define SCRXCIE_U2 RXCIE2							// rx interrupt enable bit
#define SCTXCIE_U2 TXCIE2							// tx interrupt enable bit
#define SCRXEN_U2  RXEN2							// rx enable bit
#define SCTXEN_U2  TXEN2							// tx enable bit

#define SCUCSRC_U2 UCSR2C							// USART register C
#define SCUSBS0_U2  USBS2							// stop bits
#define SCUCSZ0_U2 UCSZ20							// length
#define SCUPM0_U2  UPM20							// parity
#define SCUCSRRH_U2 UBRR2H							// USART baud rate register msb
#define SCUCSRRL_U2 UBRR2L							// USARTbaud rate register msb
#define SCUDR_U2   UDR2								// USART data register tx/rx

#define SCUSART_RX_vect_U2  USART2_RX_vect			// RX ISR
#define SCUSART_TX_vect_U2  USART2_TX_vect			// TX ISR

#define BIT_FRAME_ERROR_U2 (1<<SCFE_U2)
#define BIT_2X_SPEED_U2 (1<<SCU2X_U2)
#define FORMAT_8N2_U2 (3<<SCUCSZ0_U2) | (1<<SCUSBS0_U2)
#define FORMAT_8E1_U2 (3<<SCUCSZ0_U2) | (2<<SCUPM0_U2)
#define BIT_TX_ENABLE_U2  (1<<SCTXEN_U2)
#define BIT_TX_ISR_ENABLE_U2 (1<<SCTXCIE_U2)
#define BIT_RX_ENABLE_U2  (1<<SCRXEN_U2)
#define BIT_RX_ISR_ENABLE_U2 (1<<SCRXCIE_U2)

//***** baud rate defines
#define F_CLK_U2 				16000000UL
#define DMX_DATA_BAUD_U2		250000
#define DMX_BREAK_BAUD_U2 	 	99900
//99900

//***** states indicate current position in DMX stream
#define DMX_STATE_BREAK_U2 0
#define DMX_STATE_START_U2 1
#define DMX_STATE_DATA_U2 2
#define DMX_STATE_IDLE_U2 3

//***** status is if interrupts are enabled and IO is active
#define ISR_DISABLED_U2 			0
#define ISR_OUTPUT_ENABLED_U2 	1
#define ISR_INPUT_ENABLED_U2 	2

// **************************** global data (can be accessed in ISR)  ***************

uint8_t*  _shared_dmx_data_U2;
uint8_t   _shared_dmx_state_U2;
uint16_t  _shared_dmx_slot_U2;
uint16_t  _shared_max_slots_U2 = DMX_MIN_SLOTS_U2;
SCReceiveCallback _shared_receive_callback_U2 = NULL;


//************************************************************************************
// ************************  SClb_Atmega2560_DMXUART_C2 Output member functions  ********************

SClb_Atmega2560_DMXUART_C2::SClb_Atmega2560_DMXUART_C2(void) {
	_direction_pin = DIRECTION_PIN_NOT_USED_U2;	//optional
	_shared_max_slots_U2 = DMX_MAX_SLOTS_U2;
	_interrupt_status = ISR_DISABLED_U2;

	//zero buffer including _dmxData[0] which is start code
	for (int n = 0; n < DMX_MAX_SLOTS_U2 + 1; n++) {
		_dmxData[n] = 0;
	}
}


SClb_Atmega2560_DMXUART_C2::~SClb_Atmega2560_DMXUART_C2(void) {
	stop();
	_shared_dmx_data_U2 = NULL;
	_shared_receive_callback_U2 = NULL;
}

//  ***** start *****
//  sets up baud rate, bits and parity
//  sets globals accessed in ISR
//  enables transmission and tx interrupt

void SClb_Atmega2560_DMXUART_C2::startOutput(void) {
	if (_direction_pin != DIRECTION_PIN_NOT_USED_U2) {
		digitalWrite(_direction_pin, HIGH);
	}
	if (_interrupt_status == ISR_INPUT_ENABLED_U2) {
		stop();
	}
	if (_interrupt_status == ISR_DISABLED_U2) {	//prevent messing up sequence if already started...
		SCUCSRRH_U2 = (unsigned char)(((F_CLK_U2 + DMX_DATA_BAUD_U2 * 8L) / (DMX_DATA_BAUD_U2 * 16L) - 1) >> 8);
		SCUCSRRL_U2 = (unsigned char)((F_CLK_U2 + DMX_DATA_BAUD_U2 * 8L) / (DMX_DATA_BAUD_U2 * 16L) - 1);
		SCUCSRA_U2 &= ~BIT_2X_SPEED_U2;

		SCUDR_U2 = 0x0;     			//USART send register  
		_shared_dmx_data_U2 = dmxData();
		_shared_dmx_state_U2 = DMX_STATE_BREAK_U2;

		SCUCSRC_U2 = FORMAT_8N2_U2; 					//set length && stopbits (no parity)
		SCUCSRB_U2 |= BIT_TX_ENABLE_U2 | BIT_TX_ISR_ENABLE_U2;  //enable tx and tx interrupt
		_interrupt_status = ISR_OUTPUT_ENABLED_U2;
	}
}

//  ***** start *****
//  sets up baud rate, bits and parity
//  sets globals accessed in ISR
//  enables transmission and tx interrupt

void SClb_Atmega2560_DMXUART_C2::startInput(void) {
	if (_direction_pin != DIRECTION_PIN_NOT_USED_U2) {
		digitalWrite(_direction_pin, LOW);
	}
	if (_interrupt_status == ISR_OUTPUT_ENABLED_U2) {
		stop();
	}
	if (_interrupt_status == ISR_DISABLED_U2) {	//prevent messing up sequence if already started...
		SCUCSRRH_U2 = (unsigned char)(((F_CLK_U2 + DMX_DATA_BAUD_U2 * 8L) / (DMX_DATA_BAUD_U2 * 16L) - 1) >> 8);
		SCUCSRRL_U2 = (unsigned char)((F_CLK_U2 + DMX_DATA_BAUD_U2 * 8L) / (DMX_DATA_BAUD_U2 * 16L) - 1);
		SCUCSRA_U2 &= ~BIT_2X_SPEED_U2;

		_shared_dmx_data_U2 = dmxData();
		_shared_dmx_state_U2 = DMX_STATE_IDLE_U2;
		_shared_dmx_slot_U2 = 0;

		SCUCSRC_U2 = FORMAT_8N2_U2; 					//set length && stopbits (no parity)
		SCUCSRB_U2 |= BIT_RX_ENABLE_U2 | BIT_RX_ISR_ENABLE_U2;  //enable tx and tx interrupt
		_interrupt_status = ISR_INPUT_ENABLED_U2;
	}
}

//  ***** stop *****
//  disables interrupts

void SClb_Atmega2560_DMXUART_C2::stop(void) {
	if (_interrupt_status == ISR_OUTPUT_ENABLED_U2) {
		SCUCSRB_U2 &= ~BIT_TX_ISR_ENABLE_U2;  							//disable tx interrupt
		SCUCSRB_U2 &= ~BIT_TX_ENABLE_U2;     							//disable tx enable
	}
	else if (_interrupt_status == ISR_INPUT_ENABLED_U2) {
		SCUCSRB_U2 &= ~BIT_RX_ISR_ENABLE_U2;  							//disable rx interrupt
		SCUCSRB_U2 &= ~BIT_RX_ENABLE_U2;     							//disable rx enable	
	}
	_interrupt_status = ISR_DISABLED_U2;
}

void SClb_Atmega2560_DMXUART_C2::setDirectionPin(uint8_t pin) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

//  ***** setMaxSlots *****
//  sets the number of slots sent per DMX frame
//  defaults to 512 or DMX_MAX_SLOTS_U2
//  should be no less DMX_MIN_SLOTS_U2 slots
//  the DMX standard specifies min break to break time no less than 1024 usecs
//  at 44 usecs per slot ~= 24

void SClb_Atmega2560_DMXUART_C2::setMaxSlots(int slots) {
	_shared_max_slots_U2 = max(slots, DMX_MIN_SLOTS_U2);
}

//  ***** getSlot *****
//  reads the value of a slot
//  see buffering note for ISR below 

uint8_t SClb_Atmega2560_DMXUART_C2::getSlot(int slot) {
	return _dmxData[slot];
}

//  ***** setSlot *****
//  sets the output value of a slot

void SClb_Atmega2560_DMXUART_C2::setSlot(int slot, uint8_t value) {
	_dmxData[slot] = value;
}

//  ***** dmxData *****
//  pointer to data buffer

uint8_t* SClb_Atmega2560_DMXUART_C2::dmxData(void) {
	return &_dmxData[0];
}

//  ***** setDataReceivedCallback *****
//  sets pointer to function that is called
//  on the break after a frame has been received
//  whatever happens in this function should be quick
//  ie set a flag so that processing of the received data happens
//  outside of the ISR.


void SClb_Atmega2560_DMXUART_C2::setDataReceivedCallback(SCReceiveCallback callback) {
	_shared_receive_callback_U2 = callback;
}


//************************************************************************************
// ************************ TX ISR (transmit interrupt service routine)  *************
//
// this routine is called when USART transmission is complete
// what this does is to send the next byte
// when that byte is done being sent, the ISR is called again
// and the cycle repeats...
// until _shared_max_slots_U2 worth of bytes have been sent on succesive triggers of the ISR
// and then on the next ISR...
// the break/mark after break is sent at a different speed
// and then on the next ISR...
// the start code is sent
// and then on the next ISR...
// the next data byte is sent
// and the cycle repeats...


ISR(SCUSART_TX_vect_U2) {
	switch (_shared_dmx_state_U2) {

	case DMX_STATE_BREAK_U2:
		// set the slower baud rate and send the break
		SCUCSRRH_U2 = (unsigned char)(((F_CLK_U2 + DMX_BREAK_BAUD_U2 * 8L) / (DMX_BREAK_BAUD_U2 * 16L) - 1) >> 8);
		SCUCSRRL_U2 = (unsigned char)((F_CLK_U2 + DMX_BREAK_BAUD_U2 * 8L) / (DMX_BREAK_BAUD_U2 * 16L) - 1);
		SCUCSRA_U2 &= ~BIT_2X_SPEED_U2;
		SCUCSRC_U2 = FORMAT_8E1_U2;
		_shared_dmx_state_U2 = DMX_STATE_START_U2;
		SCUDR_U2 = 0x0;
		break;		// <- DMX_STATE_BREAK_U2

	case DMX_STATE_START_U2:
		// set the baud to full speed and send the start code
		SCUCSRRH_U2 = (unsigned char)(((F_CLK_U2 + DMX_DATA_BAUD_U2 * 8L) / (DMX_DATA_BAUD_U2 * 16L) - 1) >> 8);
		SCUCSRRL_U2 = (unsigned char)((F_CLK_U2 + DMX_DATA_BAUD_U2 * 8L) / (DMX_DATA_BAUD_U2 * 16L) - 1);
		SCUCSRA_U2 &= ~BIT_2X_SPEED_U2;
		SCUCSRC_U2 = FORMAT_8N2_U2;
		_shared_dmx_slot_U2 = 0;
		SCUDR_U2 = _shared_dmx_data_U2[_shared_dmx_slot_U2++];	//send next slot (start code)
		_shared_dmx_state_U2 = DMX_STATE_DATA_U2;
		break;		// <- DMX_STATE_START_U2

	case DMX_STATE_DATA_U2:
		// send the next data byte until the end is reached
		SCUDR_U2 = _shared_dmx_data_U2[_shared_dmx_slot_U2++];	//send next slot
		if (_shared_dmx_slot_U2 > _shared_max_slots_U2) {
			_shared_dmx_state_U2 = DMX_STATE_BREAK_U2;
		}
		break;		// <- DMX_STATE_DATA_U2
	}
}

//***********************************************************************************
// ************************ RX ISR (receive interrupt service routine)  *************
//
// this routine is called when USART receives data
// wait for break:  if have previously read data call callback function
// then on next receive:  check start code
// then on next receive:  read data until done (in which case idle)
//
//  NOTE: data is not double buffered
//  so a complete single frame is not guaranteed
//  the ISR will continue to read the next frame into the buffer

ISR(SCUSART_RX_vect_U2) {
	uint8_t status_register = SCUCSRA_U2;
	uint8_t incoming_byte = SCUDR_U2;

	if (status_register & BIT_FRAME_ERROR_U2) {
		_shared_dmx_state_U2 = DMX_STATE_BREAK_U2;
		if (_shared_dmx_slot_U2 > 0) {
			if (_shared_receive_callback_U2 != NULL) {
				_shared_receive_callback_U2(_shared_dmx_slot_U2);
			}
		}
		_shared_dmx_slot_U2 = 0;
		return;
	}

	switch (_shared_dmx_state_U2) {

	case DMX_STATE_BREAK_U2:
		if (incoming_byte == 0) {						//start code == zero (DMX)
			_shared_dmx_state_U2 = DMX_STATE_DATA_U2;
			_shared_dmx_slot_U2 = 1;
		}
		else {
			_shared_dmx_state_U2 = DMX_STATE_IDLE_U2;
		}
		break;

	case DMX_STATE_DATA_U2:
		_shared_dmx_data_U2[_shared_dmx_slot_U2++] = incoming_byte;
		if (_shared_dmx_slot_U2 > DMX_MAX_SLOTS_U2) {
			_shared_dmx_state_U2 = DMX_STATE_IDLE_U2;			// go to idle, wait for next break
		}
		break;
	}
}
