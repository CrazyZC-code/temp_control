/*
 * Analog.h
 *
 *  Created on: 09/04/2017
 *      Author: otavi
 */

#ifndef DRIVERS_ANALOG_H_
#define DRIVERS_ANALOG_H_

#define ANALOG_MIN_VALUE	0
#define ANALOG_MAX_VALUE	0x0FFF

#include "MKE02Z2.h"

typedef enum{
	PWM6	= 5u,
	PWM8	= 1u,
	PWM9	= 0u,
	PWMLED	= 4u
} PWM;

typedef enum{
	Analog0 = 11u,
	Analog1 = 10u,
	Analog2 = 9u,
	Analog3 = 8u,
	Analog4 = 7u,
	Analog5 = 6u
} AnalogPin;

typedef void (*functionPtr)(void);

void Analog_InitPWM(void);
void Analog_InitAnalog(void);

void Analog_DeinitPWM(void);
void Analog_DeinitAnalog(void);

void Analog_SetPWMPin(PWM pwmPin, uint8_t inverted);
void Analog_UnsetPWMPin(PWM pwmPin);
uint8_t Analog_SetPWMFrequency(uint32_t freq);
uint16_t Analog_PWMMaxValue(void);
void Analog_Write(PWM pwmPin, uint16_t value);

void Analog_EnableIRQ(void);
void Analog_DisableIRQ(void);
void Analog_SetIRQFunction(functionPtr function);
void Analog_SetPin(AnalogPin pin);
void Analog_UnsetPin(AnalogPin pin);
uint16_t Analog_ReadPolling(AnalogPin pin);
uint8_t Analog_ReadComplete(void);
void Analog_StartReading(AnalogPin pin);
uint16_t Analog_ReadIRQValue(AnalogPin pin);

#endif /* DRIVERS_ANALOG_H_ */
