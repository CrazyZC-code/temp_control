/*
 * TimedInt.h
 *
 *  Created on: 06/05/2017
 *      Author: Otavio
 */

#ifndef DRIVERS_TIMEDINT_H_
#define DRIVERS_TIMEDINT_H_

#include "MKE02Z2.h"

typedef enum {
	TimedChannel0 = 0,
	TimedChannel1 = 0x01
} TimedChannel;

typedef void (*functionPtr)(void);

void TimedInt_Config(TimedChannel channel, float periodInSec);
void TimedInt_SetIRQFunction(TimedChannel channel, functionPtr function);
uint32_t TimedInt_ReadCount(TimedChannel channel);

#endif /* DRIVERS_TIMEDINT_H_ */
