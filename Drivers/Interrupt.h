/*
 * Interrupt.h
 *
 *  Created on: 05/05/2017
 *      Author: Otavio
 */

#ifndef DRIVERS_INTERRUPT_H_
#define DRIVERS_INTERRUPT_H_

#include "MKE02Z2.h"

typedef enum {
	IntPin0  = 0x0001,
	IntPin1  = 0x0002,
	IntPin2  = 0x0004,
	IntPin3  = 0x0008,
	IntPin4  = 0x0010,
	IntPin5  = 0x0020,
	IntPin6  = 0x0040,
	IntPin7  = 0x0080,
	IntPin8  = 0x0101,
	IntPin9  = 0x0102,
	IntPin10 = 0x0104,
	IntPin11 = 0x0108,
	IntPin12 = 0x0110,
} InterruptPin;

typedef enum {
	Edges,
	EdgesAndLevels
} DetectionMode;

typedef enum {
	FallingLowLevel,
	RiseHighLevel
} EdgeSelect;

typedef void (*functionPtrInt)(uint8_t);

void Interrupt_Init(uint8_t enableIRQ, DetectionMode mode);
void Interrupt_EnablePin(InterruptPin pin, EdgeSelect edge);
void Interrupt_DisablePin(InterruptPin pin);
uint8_t Interrupt_PendingIRQ(void);
void Interrupt_SetIRQFunction(functionPtrInt function);

#endif /* DRIVERS_INTERRUPT_H_ */
