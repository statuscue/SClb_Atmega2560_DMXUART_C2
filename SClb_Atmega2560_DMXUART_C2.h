/*
 Name:		SClb_Atmega2560_DMXUART_C2.h
 Created:	08-Mar-19 16:19:33
 Author:	statuscue
 Editor:	http://hes.od.ua
*/

/*
	Copyright 2019-2019 by Yevhen Mykhailov
	Art-Net(TM) Designed by and Copyright Artistic Licence Holdings Ltd.
*/

#ifndef _SClb_Atmega2560_DMXUART_C2_h
#define _SClb_Atmega2560_DMXUART_C2_h


#include <Arduino.h>
#include <inttypes.h>

#define DMX_MIN_SLOTS_U2 24
#define DMX_MAX_SLOTS_U2 512

#define DIRECTION_PIN_NOT_USED_U2 255

typedef void(*SCReceiveCallback)(int);

class SClb_Atmega2560_DMXUART_C2 {

public:
	SClb_Atmega2560_DMXUART_C2(void);
	~SClb_Atmega2560_DMXUART_C2(void);

	/*!
	* @brief starts interrupt that continuously sends DMX data
	* @discussion sets up baud rate, bits and parity,
	*             sets globals accessed in ISR,
	*             enables transmission and tx interrupt
   */
	void startOutput(void);

	/*!
	 * @brief starts interrupt that continuously reads DMX data
	 * @discussion sets up baud rate, bits and parity,
	 *             sets globals accessed in ISR,
	 *             enables receive and rx interrupt
	*/
	void startInput(void);

	/*!
	 * @brief disables USART
	*/

	void stop(void);

	/*!
	 * @brief optional utility sets the pin used to control driver chip's
	 *        DE (data enable) line, HIGH for output, LOW for input.
	* @param pin to be automatically set for input/output direction
	*/
	void setDirectionPin(uint8_t pin);

	/*!
	  * @brief Sets the number of slots (aka addresses or channels) sent per DMX frame.
	  * @discussion defaults to 512 or DMX_MAX_SLOTS and should be no less DMX_MIN_SLOTS slots.
	  *             The DMX standard specifies min break to break time no less than 1024 usecs.
	  *             At 44 usecs per slot ~= 24
	  * @param slot the highest slot number (~24 to 512)
	 */
	void setMaxSlots(int slot);

	/*!
	* @brief reads the value of a slot/address/channel
	* @discussion NOTE: Data is not double buffered.
	*                   So a complete single frame is not guaranteed.
	*                   The ISR continuously reads the next frame into the buffer
	* @return level (0-255)
   */
	uint8_t getSlot(int slot);

	/*!
	 * @brief Sets the output value of a slot
	 * @param slot number of the slot/address/channel (1-512)
	 * @param value level (0-255)
	*/
	void setSlot(int slot, uint8_t value);

	/*!
	 * @brief provides direct access to data array
	 * @return pointer to dmx array
	*/
	uint8_t* dmxData(void);

	/*!
	 * @brief Function called when DMX frame has been read
	 * @discussion Sets a pointer to a function that is called
	 *             on the break after a DMX frame has been received.
	 *             Whatever happens in this function should be quick!
	 *             Best used to set a flag that is polled outside of ISR for available data.
	*/
	void setDataReceivedCallback(SCReceiveCallback callback);

private:
	/*!
	  * @brief Indicates mode ISR_OUTPUT_ENABLED or ISR_INPUT_ENABLED or ISR_DISABLED
	 */
	uint8_t  _interrupt_status;

	/*!
   * @brief pin used to control direction of output driver chip
   */
	uint8_t  _direction_pin;

	/*!
	* @brief Array of dmx data including start code
   */
	uint8_t  _dmxData[DMX_MAX_SLOTS_U2 + 1];
};

extern SClb_Atmega2560_DMXUART_C2 SCDMXUSART2;

#endif // ifndef SClb_Atmega2560_DMXUART_C2_H


