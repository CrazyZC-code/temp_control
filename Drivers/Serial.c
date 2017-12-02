/*
 * Serial.c
 *
 *  Created on: 7 de abr de 2017
 *      Author: Otavio
 */

#include "system_MKE02Z2.h"
#include "Serial.h"
#include <stdio.h>

functionPtr functionUART0;
functionPtr functionUART1;
functionPtr functionUART2;

void UART0_IRQHandler(void){
	functionUART0();
}
void UART1_IRQHandler(void){
	functionUART1();
}
void UART2_IRQHandler(void){
	functionUART2();
}

UART_Type* SelectUART(SerialNumber serial){
	switch(serial){
		case Serial0:
			return UART0;
		case Serial1:
			return UART1;
		case Serial2:
			return UART2;
		default:
			return UART0;
	}
}

void Serial_Setup(SerialNumber serial, unsigned short baudrate, Parity parity, StopBits stopbits){
	UART_Type* lUART = SelectUART(serial);

	uint32_t baudRateDivider = (((SystemCoreClock)>>4) + (baudrate>>1))/baudrate;

	// activate Clock
	switch(serial){
		case Serial0:
			SIM->SCGC |= SIM_SCGC_UART0_MASK;
			break;
		case Serial1:
			SIM->SCGC |= SIM_SCGC_UART1_MASK;
			break;
		case Serial2:
			SIM->SCGC |= SIM_SCGC_UART2_MASK;
			break;
	}

	lUART->BDH = (uint8_t)(baudRateDivider >> 8);
	if(stopbits == Two)
		lUART->BDH |= UART_BDH_SBNS_MASK;

	lUART->BDL = (uint8_t)(baudRateDivider & 0x00FF);
	lUART->C1 = UART_C1_UARTSWAI_MASK;

	if(parity != None){		// register setting on both Even and Odd
		lUART->C1 |= (UART_C1_PE_MASK | UART_C1_M_MASK);
	}
	if(parity == Odd)		// change parity form even to odd, if it's the case
		lUART->C1 |= UART_C1_PT_MASK;

	lUART->C2 = (UART_C2_TE_MASK | UART_C2_RE_MASK);
	lUART->C3 = 0;
}

void Serial_Deinit(SerialNumber serial){
	// deactivate Clock
	switch(serial){
		case Serial0:
			SIM->SCGC &= ~(SIM_SCGC_UART0_MASK);
			break;
		case Serial1:
			SIM->SCGC &= ~(SIM_SCGC_UART1_MASK);
			break;
		case Serial2:
			SIM->SCGC &= ~(SIM_SCGC_UART2_MASK);
			break;
	}
}

void Serial_Write(SerialNumber serial, uint8_t *data, uint16_t length){
	UART_Type* lUART = SelectUART(serial);

	while((lUART->S1 & UART_S1_TDRE_MASK) == 0);	// wait line to be available

	for(uint16_t pos = 0; pos < length; pos++){
		lUART->D = data[pos];
		while((lUART->S1 & UART_S1_TDRE_MASK) == 0);	// wait line to be available
	}
}

void Serial_Read(SerialNumber serial, uint8_t *data, uint16_t length){
	UART_Type* lUART = SelectUART(serial);

	uint16_t pos = 0;
	while(pos < length){
		if((lUART->S1 & UART_S1_RDRF_MASK)){
			data[pos] = lUART->D;
			pos++;
		}
	}
}

void Serial_EnableReceiveIRQ(SerialNumber serial){
	UART_Type* lUART = SelectUART(serial);

	lUART->C2 |= UART_C2_RIE_MASK;
	switch(serial){
		case Serial0:
			NVIC_EnableIRQ(UART0_IRQn);
			break;
		case Serial1:
			NVIC_EnableIRQ(UART1_IRQn);
			break;
		case Serial2:
			NVIC_EnableIRQ(UART2_IRQn);
			break;
	}
}
void Serial_DisableReceiveIRQ(SerialNumber serial){
	UART_Type* lUART = SelectUART(serial);

	lUART->C2 |= UART_C2_RIE_MASK;
	switch(serial){
		case Serial0:
			NVIC_DisableIRQ(UART0_IRQn);
			break;
		case Serial1:
			NVIC_DisableIRQ(UART1_IRQn);
			break;
		case Serial2:
			NVIC_DisableIRQ(UART2_IRQn);
			break;
	}
}

void Serial_SetIRQFunction(SerialNumber serial, functionPtr function){
	switch(serial){
		case Serial0:
			functionUART0 = function;
			break;
		case Serial1:
			functionUART1 = function;
			break;
		case Serial2:
			functionUART2 = function;
			break;
	}
}

void Serial_UseAlternateMux(uint8_t alternate){
	if(alternate)
		SIM->PINSEL |= SIM_PINSEL_UART0PS_MASK;
	else
		SIM->PINSEL &= ~(SIM_PINSEL_UART0PS_MASK);
}

void Serial_Print(SerialNumber serial, char* string){
	uint8_t currentChar = *string;
	uint16_t displacment = 0;
	UART_Type* lUART = SelectUART(serial);

	while((lUART->S1 & UART_S1_TDRE_MASK) == 0);	// wait line to be available

	while(currentChar != '\0'){
		lUART->D = currentChar;
		displacment++;
		currentChar = *(string+displacment);

		while((lUART->S1 & UART_S1_TDRE_MASK) == 0);	// wait line to be available
	}
}

char numericBuffer[30];
void Serial_PrintInteger(SerialNumber serial, int value){
	sprintf(numericBuffer,"%d",value);
	Serial_Print(serial,numericBuffer);
}
void Serial_PrintDecimal(SerialNumber serial, float value){
	uint16_t integerPart = (uint16_t)value;
	uint8_t decimalPart = (uint8_t)((value - integerPart)*100);
	sprintf(numericBuffer,"%d.%d",integerPart,decimalPart);

	Serial_Print(serial,numericBuffer);
}
