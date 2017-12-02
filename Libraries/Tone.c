/*
 * Tone.c
 *
 *  Created on: 07/05/2017
 *      Author: Otavio
 */

#include "Tone.h"

Pin buzzPin;
TimedChannel toneChannel;

void SwitchBuzzer(void){
	Digital_Toggle(buzzPin);
}

void Tone_Init(TimedChannel channel, Pin buzzer){
	toneChannel = channel;
	buzzPin = buzzer;
	if((SIM->SCGC & SIM_SCGC_PIT_MASK) == 0){
		SIM->SCGC |= SIM_SCGC_PIT_MASK;
	}

	TimedInt_SetIRQFunction(toneChannel,SwitchBuzzer);

	PIT->MCR = PIT_MCR_MDIS_MASK; // disable clocks for config
	PIT->CHANNEL[(uint8_t)channel].TCTRL = PIT_TCTRL_TIE_MASK;
	PIT->MCR = PIT_MCR_FRZ_MASK; // enable clocks

	Digital_pinMode(buzzPin,OUTPUT);
	Digital_Write(buzzPin,LOW);
}

void Tone_Off(void){
	PIT->MCR = PIT_MCR_MDIS_MASK; // disable clocks for config

	PIT->CHANNEL[(uint8_t)toneChannel].TCTRL &= ~(PIT_TCTRL_TEN_MASK);

	if(toneChannel == TimedChannel0)
		NVIC_DisableIRQ(PIT_CH0_IRQn);
	else
		NVIC_DisableIRQ(PIT_CH1_IRQn);

	PIT->MCR = PIT_MCR_FRZ_MASK; // enable clocks
	Digital_Write(buzzPin,LOW);
}

void Tone_Play(uint32_t frequency){
	PIT->MCR = PIT_MCR_MDIS_MASK; // disable clocks for config

	PIT->CHANNEL[(uint8_t)toneChannel].LDVAL = (SystemCoreClock / (frequency*2));
	PIT->CHANNEL[(uint8_t)toneChannel].TCTRL |= PIT_TCTRL_TEN_MASK;

	if(toneChannel == TimedChannel0)
		NVIC_EnableIRQ(PIT_CH0_IRQn);
	else
		NVIC_EnableIRQ(PIT_CH1_IRQn);

	PIT->MCR = PIT_MCR_FRZ_MASK; // enable clocks
}

uint32_t musicTime = 100u;

void Tone_Time(uint32_t time){
	musicTime = time;
}

void Tone_Note(Notes note, uint8_t size){
	Tone_Play(note);
	Delay(size*musicTime);
	Tone_Off();
	Delay(musicTime);
}
