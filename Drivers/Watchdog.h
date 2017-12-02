/*
 * Watchdog.h
 *
 *  Created on: 17/05/2017
 *      Author: Otavio
 */

#ifndef DRIVERS_WATCHDOG_H_
#define DRIVERS_WATCHDOG_H_

#include "MKE02Z2.h"

typedef void (*functionPtr)(void);

uint8_t Watchdog_Setup(float timeout, uint8_t enableInterrupt);
void Watchdog_Disable(void);
void Watchdog_Reset(void);
void Watchdog_SetIRQFunction(functionPtr function);

#endif /* DRIVERS_WATCHDOG_H_ */
