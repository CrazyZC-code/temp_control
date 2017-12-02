/*
 * Serial.h
 *
 *  Created on: 7 de abr de 2017
 *      Author: Otavio
 */

#ifndef DRIVERS_SERIAL_H_
#define DRIVERS_SERIAL_H_

#include "MKE02Z2.h"

typedef enum{
	Serial0,
	Serial1,
	Serial2
}SerialNumber;

typedef enum{
	None,
	Even,
	Odd
}Parity;

typedef enum{
	One,
	Two
}StopBits;

typedef void (*functionPtr)(void);

void Serial_Setup(SerialNumber serial, unsigned short baudrate, Parity parity, StopBits stopbits);
void Serial_Deinit(SerialNumber serial);
void Serial_Write(SerialNumber serial, uint8_t *data, uint16_t length);
void Serial_Read(SerialNumber serial, uint8_t *data, uint16_t length);

void Serial_EnableReceiveIRQ(SerialNumber serial);
void Serial_DisableReceiveIRQ(SerialNumber serial);

void Serial_SetIRQFunction(SerialNumber serial, functionPtr function);

void Serial_UseAlternateMux(uint8_t alternate);

// PRINT functions
void Serial_Print(SerialNumber serial, char* string);
void Serial_PrintInteger(SerialNumber serial, int value);
void Serial_PrintDecimal(SerialNumber serial, float value);

#endif /* DRIVERS_SERIAL_H_ */
