/*
 * TimedInt.c
 *
 *  Created on: 06/05/2017
 *      Author: Otavio
 */

#include "TimedInt.h"

functionPtr PIT0Function;
functionPtr PIT1Function;

void PIT_CH0_IRQHandler(void){
	PIT0Function();

	PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
}

void PIT_CH1_IRQHandler(void){
	PIT1Function();

	PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
}

void TimedInt_Config(TimedChannel channel, float periodInSec){
	if((SIM->SCGC & SIM_SCGC_PIT_MASK) == 0){
		SIM->SCGC |= SIM_SCGC_PIT_MASK;
	}
	PIT->MCR = PIT_MCR_MDIS_MASK; // disable clocks for config

	PIT->CHANNEL[(uint8_t)channel].LDVAL = (uint32_t)(SystemCoreClock * periodInSec);
	PIT->CHANNEL[(uint8_t)channel].TCTRL = (PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK);

	if(channel == TimedChannel0)
		NVIC_EnableIRQ(PIT_CH0_IRQn);
	else
		NVIC_EnableIRQ(PIT_CH1_IRQn);

	PIT->MCR = PIT_MCR_FRZ_MASK; // enable clocks
}

void TimedInt_SetIRQFunction(TimedChannel channel, functionPtr function){
	if(channel == TimedChannel0)
		PIT0Function = function;
	else
		PIT1Function = function;
}

uint32_t TimedInt_ReadCount(TimedChannel channel){
	return PIT->CHANNEL[(uint8_t)channel].CVAL;
}
