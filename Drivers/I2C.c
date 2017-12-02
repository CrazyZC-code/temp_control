/*
 * I2C.c
 *
 *  Created on: 10 de abr de 2017
 *      Author: Otavio
 */

#define I2C_TxEnable 	(I2C0->C1 |= I2C_C1_TX_MASK)
#define I2C_RxEnable	(I2C0->C1 &= ~(I2C_C1_TX_MASK))

#define I2C_SendNACK	(I2C0->C1 |= I2C_C1_TXAK_MASK)
#define I2C_SendACK		(I2C0->C1 &= ~(I2C_C1_TXAK_MASK))

#define I2C_Read		0x01
#define I2C_Write		0x00

#define I2C_ACK			0x00
#define I2C_NACK		0x01

#include "I2C.h"

I2CMode selectedMode;

/**
* Initiates the I2C peripheral.
 * @param mode				Defines if peripheral will act as master (generator of clock signal) or slave (this mode only works with interrupts for msg reading).
 * @param clock				Clock to be used: Standard Mode - 100KHz or FullSpeed - 400Khz.
 * @param generateACK_NACK	Data received will generate ACK/NACK transmition.
 * @param address10bit		(Only used for slave mode) When different than 0 I2C address will have 10-bit instead of 6-bit.
 * @param address			(Only used for slave mode) Defines this device slave address.
 */
void I2C_Init(I2CMode mode, I2CClock clock, uint8_t generateACK_NACK, uint8_t address10bit, uint16_t address){
	selectedMode = mode;
	SIM->SCGC |= SIM_SCGC_IIC_MASK;

	if(mode == I2CMaster){
		switch(clock){
			case StandardMode:
				I2C0->F = (I2C_F_MULT(0x00) | I2C_F_ICR(0x1D));	// 100KHz clock (assuming bus at default 16MHZ)
				break;
			case FullSpeed:
				I2C0->F = (I2C_F_MULT(0x00) | I2C_F_ICR(0x0B));	// 400KHz clock (assuming bus at default 16MHZ)
				break;
		}
		I2C0->C1 = I2C_C1_IICEN_MASK;
		//if(generateACK_NACK)
		//	I2C0->C1 |= I2C_C1_TXAK_MASK;


		if(address10bit)
			I2C0->C2 = I2C_C2_ADEXT_MASK;
		else
			I2C0->C2 = 0;

	}else{
		I2C0->C2 = I2C_C2_GCAEN_MASK;
		if(address10bit){
			I2C0->C2 |= I2C_C2_ADEXT_MASK;
			I2C0->A1 = (uint8_t)(address << 1u);
			I2C0->C2 |= I2C_C2_AD((address >> 7u));
		}else{
			I2C0->A1 = (uint8_t)(address << 1u);
		}
		I2C0->C1 = (I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK);
		//if(generateACK_NACK)
		//	I2C0->C1 |= I2C_C1_TXAK_MASK;

		NVIC_EnableIRQ(I2C0_IRQn);
	}
}

void I2C_Deinit(void){
	NVIC_DisableIRQ(I2C0_IRQn);
	SIM->SCGC &= ~(SIM_SCGC_I2C_MASK);
}

void I2C_Start(void){
	I2C_TxEnable;
	I2C0->C1 |= I2C_C1_MST_MASK;

	while((I2C0->S & I2C_S_BUSY_MASK) == 0);	// wait for START to be send
}

void I2C_Stop(void){
	I2C0->C1 &= ~(I2C_C1_TX_MASK);

	while((I2C0->S & I2C_S_BUSY_MASK) == 0);	// wait for STOP to be send
}

void I2C_MasterWrite(uint8_t data){
	// wait for previous transfer to be complete
	while((I2C0->S & I2C_S_TCF_MASK) != I2C_S_TCF_MASK);

	I2C_TxEnable;
	I2C0->D = data;

	while((I2C0->S & I2C_S_IICIF_MASK) == 0);	// wait for byte to be transmitted
	I2C0->S |= I2C_S_IICIF_MASK;				// clear IRQ mask
}

uint8_t I2C_MasterRead(uint8_t ack){
	I2C_RxEnable;

	if(ack)
		I2C_ACK;
	else
		I2C_NACK;

	uint8_t recvByte = I2C0->D;

	// wait for previous transfer to be complete
	while((I2C0->S & I2C_S_TCF_MASK) == 0);

	while((I2C0->S & I2C_S_IICIF_MASK) == 0);	// wait for byte to be transmitted
	I2C0->S |= I2C_S_IICIF_MASK;				// clear IRQ mask

	return recvByte;
}

void I2C_MasterSendMsg(uint16_t slaveAddress, uint8_t *msg, uint16_t length){
	I2C_Start();

	// write address (switch between 10bit and 7bit
	if((I2C0->C2 & I2C_C2_ADEXT_MASK) != 0){
		I2C_MasterWrite((0x78 & ((uint8_t)(slaveAddress >> 8u))));	// send 2 MSB of address
		I2C_MasterWrite(((uint8_t)(slaveAddress & 0xFF)));
	}else{
		I2C_MasterWrite(((uint8_t)((slaveAddress << 1) | I2C_Write)));
	}

	for(uint16_t pos = 0; pos < length; pos++){
		I2C_MasterWrite(msg[pos]);
	}

	I2C_Stop();
}

void I2C_MasterRecvMsg(uint16_t slaveAddress, uint8_t *msg, uint16_t length){
	I2C_Start();

	// write address (switch between 10bit and 7bit
	if((I2C0->C2 & I2C_C2_ADEXT_MASK) != 0){
		I2C_MasterWrite((0x78 & ((uint8_t)(slaveAddress >> 8u))));	// send 2 MSB of address
		I2C_MasterWrite(((uint8_t)(slaveAddress & 0xFF)));
	}else{
		I2C_MasterWrite(((uint8_t)((slaveAddress << 1) | I2C_Read)));
	}

	// read one dummy byte to switch Rx
	I2C_MasterRead(0x01);

	for(uint16_t pos = 0; pos < length; pos++){
		msg[pos] = I2C_MasterRead(0x01);
	}
	I2C_MasterRead(0x00);

	I2C_Stop();
}
