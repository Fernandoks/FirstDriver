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
	uint32_t USART_BaudRate; //size might change
	uint8_t USART_WordLength;
	uint8_t USART_StopBits;//0=1 bits, 1=0.5 bit, 2=2 bits, 3=1.5 bit
	uint8_t USART_Parity;
	uint8_t USART_Mode;
	uint8_t USART_HardwareFlowCtrl;
	uint8_t USART_OverSampling;
}USART_PIN_CONF_TypeDef;

typedef struct{
	_Quentin_USART_RegDef_t* pUSART; /*holds register addresses*/
	USART_PIN_CONF_TypeDef USART_Pin_Config; /* holds pins settings */
	uint8_t* USART_TxBuffer;
	uint8_t* USART_RxBuffer;
	uint32_t TxLength;
	uint32_t RxLength;
	uint8_t USART_TxState;
	uint8_t USART_RxState;
}USART_Handler_TypedDef;

/* WORD LENGTH */
#define USART_WORD_LENGTH_8			0
#define USART_WORD_LENGTH_9			1

/* WAKEUP METHOD */
#define USART_WAKE_IDLE_LINE		0
#define USART_WAKE_ADDRESS_MARK		1

/* STOP BIT */
#define USART_STOP_BIT_1			0
#define USART_STOP_BIT_0_5			1
#define USART_STOP_BIT_2			2
#define USART_STOP_BIT_1_5			3

/* PARITY */
#define USART_PARITY_DISABLE	DISABLE
#define USART_PARITY_ENABLE		ENABLE

/* PARITY SELECTION */
#define USART_PARITY_EVEN			0
#define USART_PARITY_ODD			1

/* USART MODE */
#define USART_MODE_TX				0
#define USART_MODE_RX				1
#define USART_MODE_TXRX				2	//USART synchronous mode ?

/* OVERSAMPLING */
#define USART_OVERSAMPLING_16		0
#define USART_OVERSAMPLING_8		1

/* RECEIVER WAKEUP */
#define USART_WAKE_ACTIVE_MODE		0
#define USART_WAKE_MUTE_MODE		1

/* BAUD RATE  */
#define USART_BAUD_RATE_1200		1200
#define USART_BAUD_RATE_2400		2400
#define USART_BAUD_RATE_9600		9600
#define USART_BAUD_RATE_19200		19200
#define USART_BAUD_RATE_38400		38400
#define USART_BAUD_RATE_57600		57600
#define USART_BAUD_RATE_115200		115200
#define USART_BAUD_RATE_230400		230400
#define USART_BAUD_RATE_460800		460800
#define USART_BAUD_RATE_921600		921600
#define USART_BAUD_RATE_1792000		1792000
#define USART_BAUD_RATE_1843200		1843200
#define USART_BAUD_RATE_3584000		3584000
#define USART_BAUD_RATE_3686400		3686400

/* BIT SHIFT */
#define USART_SHIFT_CR1_OVER8		15
#define USART_SHIFT_CR1_UE			13
#define USART_SHIFT_CR1_M			12
#define USART_SHIFT_CR1_PCE			10
#define USART_SHIFT_CR1_TE			3
#define USART_SHIFT_CR1_RE			2

#define USART_SHIFT_CR2_STOP		12

#define USART_SHIFT_SR_TXE			7
#define USART_SHIFT_SR_TC			6
#define USART_SHIFT_SR_RXNE			5

#define USART_SHIFT_BRR_MANTISSA	4

/*** USART Actions ***/
void UART_PCLKControl(_Quentin_USART_RegDef_t *pUSARTx, uint8_t EnableDisable);

void USART_Initialize(USART_Handler_TypedDef* pUSART_Handler);

void USART_WriteData(USART_Handler_TypedDef* pUSART_Handler);

void USART_ReadData(USART_Handler_TypedDef* pUSART_Handler);

void USART_SetBaudRate(USART_Handler_TypedDef* pUSART_Handler);

#endif /* INC_STM32F446XX_USART_DRIVER_COURSE_H_ */

#endif /* QUENTIN */
