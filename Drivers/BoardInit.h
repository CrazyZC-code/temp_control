/*
 * BoardInit.h
 *
 *  Created on: 7 de abr de 2017
 *      Author: Otavio
 */

#ifndef DRIVERS_BOARDINIT_H_
#define DRIVERS_BOARDINIT_H_

#include "MKE02Z2.h"

typedef enum {
	FALSE = 0x00,
	TRUE  = 0x01u
} Boolean;

void ConfigBoardDefaultMuxing(void);
void Delay(unsigned short ms);
uint64_t Millis(void);

#endif /* DRIVERS_BOARDINIT_H_ */
