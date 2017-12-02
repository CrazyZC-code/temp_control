/*
 * I2C.h
 *
 *  Created on: 10 de abr de 2017
 *      Author: Otavio
 */

#ifndef DRIVERS_I2C_H_
#define DRIVERS_I2C_H_

#include "MKE02Z2.h"

typedef void (*functionPtr)(void);

typedef enum{
	I2CMaster,
	I2CSlave
} I2CMode;

typedef enum{
	StandardMode,
	FullSpeed
} I2CClock;

void I2C_Init(I2CMode mode, I2CClock clock, uint8_t generateACK_NACK, uint8_t address10bit, uint16_t address);
void I2C_Deinit(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_MasterWrite(uint8_t data);
uint8_t I2C_MasterRead(uint8_t ack);
void I2C_MasterSendMsg(uint16_t slaveAddress, uint8_t *msg, uint16_t length);
void I2C_MasterRecvMsg(uint16_t slaveAddress, uint8_t *msg, uint16_t length);

#endif /* DRIVERS_I2C_H_ */
