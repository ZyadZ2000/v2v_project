/*
 * 
 *
 *  
 *      Author: Gp Team
 *
 * 
 */

#include "UartRingbuffer.h"
#include <string.h>
#include <stdint.h>

ring_buffer ring_rx_buffer = { { 0 }, 0, 0 };

ring_buffer *_rx_buffer;

void store_char(unsigned char c, ring_buffer *buffer);

void Ringbuf_init(void) {
	_rx_buffer = &ring_rx_buffer;
}

void store_char(unsigned char c, ring_buffer *buffer) {
	int i = (unsigned int) (buffer->head + 1) % UART_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != buffer->tail) {
		buffer->buffer[buffer->head] = c;
		buffer->head = i;
	}
}

int Uart_read(void) {
	// if the head isn't ahead of the tail, we don't have any characters
	if (_rx_buffer->head == _rx_buffer->tail) {
		return -1;
	} else {
		unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
		_rx_buffer->tail = (unsigned int) (_rx_buffer->tail + 1)
				% UART_BUFFER_SIZE;
		return c;
	}
}

/* checks if the new data is available in the incoming buffer
 */
uint16_t IsDataAvailable(void) {
	return (uint16_t) (UART_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail)
			% UART_BUFFER_SIZE;
}

int Uart_peek() {
	if (_rx_buffer->head == _rx_buffer->tail) {
		return -1;
	} else {
		return _rx_buffer->buffer[_rx_buffer->tail];
	}
}

/* copies the data from the incoming buffer into our buffer
 * Must be used if you are sure that the data is being received
 * it will copy irrespective of, if the end string is there or not
 * if the end string gets copied, it returns 1 or else 0
 * Use it either after (IsDataAvailable) or after (Wait_for) functions
 */
int Copy_upto(char *string, char *buffertocopyinto) {
	int so_far = 0;
	int len = strlen(string);
	int indx = 0;

	again: while (Uart_peek() != string[so_far]) {
		buffertocopyinto[indx] = _rx_buffer->buffer[_rx_buffer->tail];
		_rx_buffer->tail = (unsigned int) (_rx_buffer->tail + 1)
				% UART_BUFFER_SIZE;
		indx++;
		while (!IsDataAvailable())
			;

	}
	while (Uart_peek() == string[so_far]) {
		so_far++;
		buffertocopyinto[indx++] = Uart_read();
		if (so_far == len)
			return 1;
		while ((!IsDataAvailable()))
			;
	}

	if (so_far != len) {
		so_far = 0;
		goto again;
	}

	if (so_far == len)
		return 1;
	else
		return 0;
}

/* Waits for a particular string to arrive in the incoming buffer... It also increments the tail
 * returns 1, if the string is detected
 */
// added timeout feature so the function won't block the processing of the other functions
int Wait_for(char *string) {
	int so_far = 0;
	int len = strlen(string);

	again:

	while ((!IsDataAvailable()))
		;  // let's wait for the data to show up
	while (Uart_peek() != string[so_far]) // peek in the rx_buffer to see if we get the string
	{
		if (_rx_buffer->tail != _rx_buffer->head) {
			_rx_buffer->tail = (unsigned int) (_rx_buffer->tail + 1)
					% UART_BUFFER_SIZE;  // increment the tail
		}

		else {
			return 0;
		}
	}
	while (Uart_peek() == string[so_far]) // if we got the first letter of the string
	{
		// now we will peek for the other letters too
		so_far++;
		_rx_buffer->tail = (unsigned int) (_rx_buffer->tail + 1)
				% UART_BUFFER_SIZE;  // increment the tail
		if (so_far == len)
			return 1;
		while ((!IsDataAvailable()))
			;
	}

	if (so_far != len) {
		so_far = 0;
		goto again;
	}

	if (so_far == len)
		return 1;
	else
		return 0;
}

