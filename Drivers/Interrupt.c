/*
 * Interrupt.c
 *
 *  Created on: 05/05/2017
 *      Author: Otavio
 */

#include "Interrupt.h"

functionPtrInt KBIfunction;

void KBI0_IRQHandler(void){
	KBIfunction(0x00);
	KBI0->SC |= KBI_SC_KBACK_MASK;
}

void KBI1_IRQHandler(void){
	KBIfunction(0x01);
	KBI0->SC |= KBI_SC_KBACK_MASK;
}

void Interrupt_Init(uint8_t enableIRQ, DetectionMode mode){
	SIM->SCGC |= (SIM_SCGC_KBI0_MASK | SIM_SCGC_KBI1_MASK);

	KBI0->SC = 0;
	KBI1->SC = 0;

	if(enableIRQ){
		KBI0->SC |= KBI_SC_KBIE_MASK;
		KBI1->SC |= KBI_SC_KBIE_MASK;

		NVIC_EnableIRQ(KBI0_IRQn);
		NVIC_EnableIRQ(KBI1_IRQn);
	}
	if(mode == EdgesAndLevels){
		KBI0->SC |= KBI_SC_KBMOD_MASK;
		KBI1->SC |= KBI_SC_KBMOD_MASK;
	}
}

void Interrupt_EnablePin(InterruptPin pin, EdgeSelect edge){
	KBI_Type *kbiPtr;
	if((0xFF00 & pin) == 0)
		kbiPtr = KBI0;
	else
		kbiPtr = KBI1;

	kbiPtr->PE |= (uint8_t)pin;
	if(edge == RiseHighLevel)
		kbiPtr->ES |= (uint8_t)pin;
	else
		kbiPtr->ES &= ~((uint8_t)pin);
}

void Interrupt_DisablePin(InterruptPin pin){
	KBI_Type *kbiPtr;
	if((0xFF00 & pin) == 0)
		kbiPtr = KBI0;
	else
		kbiPtr = KBI1;

	kbiPtr->PE &= ~((uint8_t)pin);
}

uint8_t Interrupt_PendingIRQ(void){
	uint8_t result = 0;

	if((KBI0->SC & KBI_SC_KBF_MASK))
		result |= 0x01;
	if((KBI1->SC & KBI_SC_KBF_MASK))
		result |= 0x02;

	return result;
}

void Interrupt_SetIRQFunction(functionPtrInt function){
	KBIfunction = function;
}
