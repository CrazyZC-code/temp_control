/*
 ============================================================================
152000152000 Name        : main.c
 Author      : Otavio Borges
 Version     :
 Copyright   : nada
 Description : Hello World in C
 ============================================================================
 */

#include <string.h>
#include <stdio.h>

#include "MKE02Z2.h"
#include "BoardInit.h"

#include "Digital.h"
#include "Analog.h"
#include "TimedInt.h"
#include "Serial.h"

#define TARGET_TEMP			2300
#define MAX_PELTIER_PWM		29000u
#define MAX_PELTIER_GAIN	100u
#define DELAY_FAN			100u

#define INT_GAIN			0.04f
#define PROP_GAIN			0.1f

uint16_t tempValues[16];
uint8_t tempIdx = 0;

void TempResult(void){
	tempValues[tempIdx] = Analog_ReadIRQValue(Analog0);
	tempIdx++;
	if(tempIdx == 16)
		tempIdx = 0;
}

int32_t currentTemp = 0;
int32_t currentPeltier = 0;
uint8_t delayedTurnoff = 0xFF;
int32_t error;
int32_t errorSum = 0;
uint8_t fanState = 0;
int32_t output = 0;

uint32_t averagedSample(void){
	uint64_t sumValue = 0;
	for(uint8_t idx = 0; idx < 16; idx++)
		sumValue += tempValues[idx];

	sumValue = (sumValue >> 4) * 47800; // divide by 16

	return (sumValue >> 12); // divide by 4096
}

void TimedIrq(void){
	currentTemp = (int32_t)averagedSample();
	error = currentTemp - TARGET_TEMP;

	if(delayedTurnoff != 0xFF){
		delayedTurnoff--;
		if(delayedTurnoff == 0){
			delayedTurnoff = 0xFF;
			Analog_Write(PWM6, 0); // turnoff fan
			fanState = 0;
		}
	}

	errorSum += error;
	output = (INT_GAIN * errorSum) + (PROP_GAIN * error);

	if(errorSum > 2000)
		errorSum = 2000;
	else if(errorSum < -2000)
		errorSum = -2000;

	if(output > MAX_PELTIER_PWM)
		output = MAX_PELTIER_PWM;

	if(output < 0){
		if(delayedTurnoff == 0xFF)
			delayedTurnoff = DELAY_FAN;
		Analog_Write(PWM8, 0);
	}else{
		if(fanState == 0){
			Analog_Write(PWM6, 31000u);
			fanState = 0x01;
		}

		if((output - currentPeltier) > MAX_PELTIER_GAIN){
			currentPeltier += MAX_PELTIER_GAIN;
		}else{
			currentPeltier = output;
		}

		if(currentPeltier > MAX_PELTIER_PWM)
			currentPeltier = MAX_PELTIER_PWM;

		Analog_Write(PWM8, currentPeltier);
	}
}

char msg[30];

void Uart0Recv(void){
	// do nothing
}

void SendDataOverUart(void){
	Serial_Write(Serial0, msg, 23);
}

int main(void){
	// Configure board pins and auxiliary functions
	ConfigBoardDefaultMuxing();
	memset(tempValues, 0u, sizeof(uint16_t) * 16);

	// Set board led as output
	Digital_pinMode(DigitalLed, OUTPUT);

	Analog_InitAnalog();
	Analog_SetPin(Analog0);
	Analog_SetIRQFunction(TempResult);
	Analog_EnableIRQ();
	Analog_StartReading(Analog0);

	Analog_InitPWM();
	Analog_SetPWMPin(PWM6, 0);
	Analog_SetPWMPin(PWM8, 0);

	Analog_Write(PWM6, 0);
	Analog_Write(PWM8, 0);

	TimedInt_SetIRQFunction(TimedChannel0, TimedIrq);
	TimedInt_Config(TimedChannel0, 0.05f);

	Serial_SetIRQFunction(Serial0, Uart0Recv);
	Serial_Setup(Serial0, 57600, None, One);

	TimedInt_SetIRQFunction(TimedChannel1, SendDataOverUart);
	TimedInt_Config(TimedChannel1, 10.0f);

	while(1){

		Digital_Toggle(DigitalLed);
		// wait 1s
		Analog_StartReading(Analog0);
		uint16_t seconds = (uint16_t)((Millis() * 16) >> 14);
		sprintf(msg, "X:%05d;T:%05d;O:%05d\n", seconds, currentTemp, output);
		Delay(50u);
	}

	return 0;
}
