/*
 * Digital.h
 *
 *  Created on: 09/04/2017
 *      Author: otavi
 */

#ifndef DRIVERS_DIGITAL_H_
#define DRIVERS_DIGITAL_H_

#include "MKE02Z2.h"

typedef enum {
	Digital0   = 8u,
	Digital1   = 9u,
	Digital2   = 28u,
	Digital3   = 7u,
	Digital4   = 31u,
	Digital5   = 6u,
	Digital6   = 13u,
	Digital7   = 30u,
	Digital8   = 1u,
	Digital9   = 0,
	Digital10  = 27u,
	Digital11  = 25u,
	Digital12  = 26u,
	Digital13  = 24u,
	Digital14  = 2u,
	Digital15  = 3u,
	DigitalLed = 12u
} Pin;

typedef enum {
	INPUT,
	OUTPUT,
	INPUT_PULLUP

} Mode;

typedef enum {
	LOW  = 0x00,
	HIGH = 0x01
} Value;

void Digital_pinMode(Pin pin, Mode mode);
void Digital_Write(Pin pin, Value value);
void Digital_Toggle(Pin pin);
Value Digital_Read(Pin pin);

#endif /* DRIVERS_DIGITAL_H_ */
