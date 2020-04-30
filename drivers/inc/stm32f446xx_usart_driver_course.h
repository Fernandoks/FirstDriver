/*
 * stm32f446xx_usart_driver_course.h
 *
 *  Created on: Apr 21, 2020
 *      Author: quentin
 */
#define QUENTIN

#ifdef QUENTIN

#ifndef INC_STM32F446XX_USART_DRIVER_COURSE_H_
#define INC_STM32F446XX_USART_DRIVER_COURSE_H_

#include "stm32f446xx_course.h"

/* GPIO_PIN_CONF_TypeDef */
typedef struct{
	uint8_t USART_Instance;
	uint32_t USART_BaudRate; //size might change
	uint8_t USART_WordLength;
	uint8_t USART_StopBits;//0=1 bits, 1=0.5 bit, 2=2 bits, 3=1.5 bit
	uint8_t USART_Parity;
	uint8_t USART_Mode;
	uint8_t USART_HardwareFlowCtrl;
	uint8_t USART_OverSampling;
}USART_PIN_CONF_TypeDef;

typedef struct{
	USART_RegDef_t* pUSART; /*holds register addresses*/
	USART_PIN_CONF_TypeDef USART_Pin_Config; /* holds pins settings */
}USART_Handler_TypedDef;

/* INSTANCE */
#define USART_INSTANCE_1			1
#define USART_INSTANCE_2			2
#define USART_INSTANCE_3			3
#define USART_INSTANCE_4			4
#define USART_INSTANCE_5			5
#define USART_INSTANCE_6			6

/* WORD LENGTH */
#define USART_WORD_LENGTH_8DATA		0
#define USART_WORD_LENGTH_9DATA		1

/* WAKEUP METHOD */
#define USART_WAKE_IDLE_LINE		0
#define USART_WAKE_ADDRESS_MARK		1

/* PARITY */
#define USART_PARITY_DISABLE	DISABLE
#define USART_PARITY_ENABLE		ENABLE

/* PARITY SELECTION */
#define USART_PARITY_EVEN			0
#define USART_PARITY_ODD			1

/* OVERSAMPLING */
#define USART_OVERSAMPLING_16		0
#define USART_OVERSAMPLING_8		1

/* RECEIVER WAKEUP */
#define USART_WAKE_ACTIVE_MODE		0
#define USART_WAKE_MUTE_MODE		1

/*** USART Actions ***/
void USART_Initialize(USART_Handler_TypedDef* pUSART_Handler);

void USART_WriteData(USART_Handler_TypedDef* pUSART_Handler, uint8_t* pUSART_TX_Buffer);

void USART_ReadData(USART_Handler_TypedDef* pUSART_Handler);

#endif /* INC_STM32F446XX_USART_DRIVER_COURSE_H_ */

#endif /* QUENTIN */
