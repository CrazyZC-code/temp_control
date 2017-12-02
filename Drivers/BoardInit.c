/*
 * BoardInit.c
 *
 *  Created on: 7 de abr de 2017
 *      Author: Otavio
 */

#include "system_MKE02Z2.h"
#include "BoardInit.h"

uint64_t count = 0;

void SysTick_Handler(void){
	count++;
}

void ConfigBoardDefaultMuxing(void){
	SystemCoreClockUpdate();

	SIM->SOPT &= ~(SIM_SOPT_NMIE_MASK);			// disable NMIE on PTB4 (used as output)
	SIM->PINSEL = 0;							// default pin muxing

	SysTick_Config(SystemCoreClock/1000u);	// 1ms tick
	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn,0u);
}

void Delay(unsigned short ms){
	uint64_t currentCount = count + ms;
	while(count < currentCount)
		asm("nop");
}

uint64_t Millis(void){
	return count;
}
