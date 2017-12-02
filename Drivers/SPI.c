/*
 * SPI.c
 *
 *  Created on: 10 de abr de 2017
 *      Author: Otavio
 */

#include "SPI.h"

functionPtr functionSPI;
void SPI1_IRQHandler(void){
	functionSPI();
}

uint8_t SPI_CalcBR(uint32_t freq){
	uint8_t SPR[8];

	int32_t closestResults[8];
	for(uint8_t SPPR = 0; SPPR < 8; SPPR++){
		closestResults[SPPR] = SystemCoreClock/(SPPR+1);
		SPR[SPPR] = 0;

		while(((closestResults[SPPR] >> (SPR[SPPR]+1u)) > freq) & (SPR[SPPR] < 9)){
			SPR[SPPR]++;
		}
		//SPR[SPPR]--;
	}

	uint8_t bestSPPR = 0;
	for(uint8_t SPPR = 0; SPPR < 8; SPPR++){
		closestResults[SPPR] = freq - (closestResults[SPPR] >> (SPR[SPPR]+1u));
		if(closestResults[SPPR] < 0)
			closestResults[SPPR] = ~(closestResults[SPPR]) + 1u;

		if(closestResults[SPPR] < closestResults[bestSPPR])
			bestSPPR = SPPR;
	}

	return ((uint8_t)((bestSPPR << 4) | SPR[bestSPPR]));
}

/**
 * Initiates SPI peripheral with determined parameters.
 * @param frequency	Clock frequency for the SPI.
 * @param mode		Selects if peripheral is gonna work as Master or Slave (pooling doesn't work on slave mode).
 * @param polarity	Defines if data will be read or written when clock signal is High or Low.
 * @param phase		Defines which clock signal edge will have stable data on MOSI and MISO.
 * @param order		Defines which bit will be transfered first least (LSB) or most (MSB) significant bit.
 */
void SPI_Init(uint32_t frequency, SPIMode mode, ClockPolarity polarity, ClockPhase phase, BitOrder order){
	SIM->SCGC |= SIM_SCGC_SPI1_MASK;

	SPI1->C1 = (SPI_C1_SPE_MASK | SPI_C1_SSOE_MASK | mode | polarity | phase | order);
	SPI1->C2 = (SPI_C2_SPISWAI_MASK | SPI_C2_MODFEN_MASK);

	SPI1->BR = SPI_CalcBR(frequency);

	if(mode == Slave){
		SPI1->C1 |= SPI_C1_SPIE_MASK;
		NVIC_EnableIRQ(SPI1_IRQn);
	}
}

/**
 * Disable SPI peripheral.
 */
void SPI_Deinit(void){
	SPI1->C1 &= ~(SPI_C1_SPIE_MASK);
	NVIC_DisableIRQ(SPI1_IRQn);

	SIM->SCGC &= ~(SIM_SCGC_SPI1_MASK);
}

/**
 * Initiates a SPI transfer. Some devices will only start sending data after message is sent, in this case use dummies in send buffer.
 * @param send	The bytes to be sent.
 * @param recv	The bytes received.
 * @param size	Amount of bytes to transfer (consider send length + receive length)
 */
void SPI_Transfer(uint8_t *send, uint8_t *recv, uint16_t size){
	for(uint16_t pos = 0; pos < size; pos++){
		while((SPI1->S & SPI_S_SPTEF_MASK) == 0);	// wait until line is available to send
		SPI1->D = send[pos];
		while((SPI1->S & SPI_S_SPRF_MASK) == 0);	// wait for data to be received
		recv[pos] = SPI1->D;
	}
}

/**
 * Defines the function to be called upon SPI interrupt
 * @param function	Desired function to handle IRQ
 */
void SPI_SetIRQFunction(functionPtr function){
	functionSPI = function;
}
