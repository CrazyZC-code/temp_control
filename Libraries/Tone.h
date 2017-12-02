/*
 * Tone.h
 *
 *  Created on: 07/05/2017
 *      Author: Otavio
 */

#ifndef LIBRARIES_TONE_H_
#define LIBRARIES_TONE_H_

#include "MKE02Z2.h"
#include "TimedInt.h"
#include "Digital.h"

typedef enum {
	Do		= 261u,
	ReBemol	= 277u,
	Re		= 294u,
	MiBemol	= 311u,
	Mi		= 330u,
	Fa		= 349u,
	SolBemol= 370u,
	Sol		= 392u,
	LaBemol	= 415u,
	La		= 440u,
	SiBemol = 466u,
	Si		= 494u
} Notes;

void Tone_Init(TimedChannel channel, Pin buzzer);
void Tone_Off(void);
void Tone_Play(uint32_t frequency);

void Tone_Time(uint32_t time);
void Tone_Note(Notes note, uint8_t size);

#endif /* LIBRARIES_TONE_H_ */
