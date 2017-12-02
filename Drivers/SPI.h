/*
 * SPI.h
 *
 *  Created on: 10 de abr de 2017
 *      Author: Otavio
 */

#ifndef DRIVERS_SPI_H_
#define DRIVERS_SPI_H_

#include "MKE02Z2.h"

typedef enum{
	Slave 	= 0,
	Master	= SPI_C1_MSTR_MASK
} SPIMode;

typedef enum{
	ActiveHigh	= 0,
	ActiveLow	= SPI_C1_CPOL_MASK
} ClockPolarity;

typedef enum{
	Middle	= 0,
	Start	= SPI_C1_CPHA_MASK
} ClockPhase;

typedef enum{
	MSB	= 0,
	LSB	= SPI_C1_LSBFE_MASK
} BitOrder;

typedef void (*functionPtr)(void);

void SPI_Init(uint32_t frequency, SPIMode mode, ClockPolarity polarity, ClockPhase phase, BitOrder order);
void SPI_Deinit(void);
void SPI_Transfer(uint8_t *send, uint8_t *recv, uint16_t size);
void SPI_SetIRQFunction(functionPtr function);

#endif /* DRIVERS_SPI_H_ */
