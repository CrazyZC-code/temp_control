/*
 * Digital.c
 *
 *  Created on: 09/04/2017
 *      Author: otavi
 */

#include "Digital.h"

void Digital_pinMode(Pin pin, Mode mode){
	switch(mode){
		case OUTPUT:
			GPIOA->PDDR |= 1 << pin;
			break;
		case INPUT:
			GPIOA->PDDR &= ~(1 << pin);	// clear PDDR and PIDR pin bit
			GPIOA->PIDR &= ~(1 << pin);
			PORT->PUEL &= ~(1 << pin);
			break;
		case INPUT_PULLUP:
			GPIOA->PDDR &= ~(1 << pin);	// clear PDDR and PIDR pin bit
			GPIOA->PIDR &= ~(1 << pin);
			PORT->PUEL |= (1 << pin);
	}
}

void Digital_Write(Pin pin, Value value){
	if((GPIOA->PDDR & (1 << pin)) == 0)	// Pin defined as input return without
		return;							// setting value

	if(value == HIGH)
		GPIOA->PSOR = (1 << pin);
	else
		GPIOA->PCOR = (1 << pin);
}

void Digital_Toggle(Pin pin){
	if((GPIOA->PDDR & (1 << pin)) == 0)	// Pin defined as input return without
		return;							// setting value

	GPIOA->PTOR = (1 << pin);
}

Value Digital_Read(Pin pin){
	if((GPIOA->PDDR & (1 << pin)) == 0)	// Input
		return (uint8_t)((GPIOA->PDIR >> pin) & 1u);	// shift the pin bit to be the first and clear remaining
	else
		return (uint8_t)((GPIOA->PDOR >> pin) & 1u);
}
