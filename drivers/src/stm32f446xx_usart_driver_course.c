/*
 * stm32f446xx_usart_driver_course.c
 *
 *  Created on: Apr 24, 2020
 *      Author: quentin
 */

#include "stm32f446xx_usart_driver_course.h"

/* USART Initialization */
void USART_Initialize(USART_Handler_TypedDef* pUSART_Handler)
{
	//baud rate equation : txrx baud = Fclk/(8*(2-oversampling)*USARTDIV)
	float USARTDIV, SysClock;

//	USARTDIV = f/((8*2-pUSART_Handler->USART_Pin_Config.USART_OverSampling)*pUSART_Handler->USART_Pin_Config.USART_BaudRate);

	if((pUSART_Handler->USART_Pin_Config.USART_Instance == USART_INSTANCE_4) || (pUSART_Handler->USART_Pin_Config.USART_Instance == USART_INSTANCE_5))
	{

	}
	else
	{

	}

	/** OVERSAMPLING **/
	pUSART_Handler->pUSART->CR1 &= ~(1ul << 15);
	pUSART_Handler->pUSART->CR1 |= (pUSART_Handler->USART_Pin_Config.USART_OverSampling << 15);

	/** PARITY **/
	pUSART_Handler->pUSART->CR1 &= ~(1ul << 10);
	pUSART_Handler->pUSART->CR1 |= (pUSART_Handler->USART_Pin_Config.USART_Parity << 10);
}

void USART_ReadData(USART_Handler_TypedDef* pUSART_Handler)
{
	uint32_t temp;

	/* Enable UART */
	pUSART_Handler->pUSART->CR1 |= (1ul << 13);

	/**  WORDLENGTH **/
	pUSART_Handler->pUSART->CR1 &= ~(1ul << 12);
	pUSART_Handler->pUSART->CR1 |= (pUSART_Handler->USART_Pin_Config.USART_WordLength << 12);

	/** Stop bit **/
	pUSART_Handler->pUSART->CR2 &= (3ul << 12);
	pUSART_Handler->pUSART->CR2 |= (pUSART_Handler->USART_Pin_Config.USART_StopBits << 12);

	/** Baud rate **/
	pUSART_Handler->pUSART->BRR &= ~(0xFFFF); //bits 0-3 are fraction, bits 4-15 are mantissa
	//	pUSART_Handler->pUSART->BRR |= (pUSART_Handler->USART_Pin_Config.USART_BaudRate); //fraction
	//	pUSART_Handler->pUSART->BRR |= (pUSART_Handler->USART_Pin_Config.USART_BaudRate << 4); //mantissa

	/** Receiver enable **/
	pUSART_Handler->pUSART->CR1 &= ~(1ul << 2);
	pUSART_Handler->pUSART->CR1 |= (1ul << 2); //begins search for a start bit

	/** Read data register not empty **/
	temp |= ((pUSART_Handler->pUSART->SR >> 5) & 1ul);
	while(temp != 1)	//once RXNE=1 data can be read
	{
	}

}

void USART_WriteData(USART_Handler_TypedDef* pUSART_Handler, uint8_t* pUSART_TX_Buffer)
{
	//Procedure p801

	uint32_t temp;

	/** Enable UART **/
	pUSART_Handler->pUSART->CR1 |= (1ul << 13);

	/**  WORDLENGTH **/
	pUSART_Handler->pUSART->CR1 &= ~(1ul << 12);
	pUSART_Handler->pUSART->CR1 |= (pUSART_Handler->USART_Pin_Config.USART_WordLength << 12);

	/** Stop bit **/
	pUSART_Handler->pUSART->CR2 &= (3ul << 12);
	pUSART_Handler->pUSART->CR2 |= (pUSART_Handler->USART_Pin_Config.USART_StopBits << 12);

	/** Baud rate **/
	pUSART_Handler->pUSART->BRR &= ~(0x0000FFFF); //bits 0-3 are fraction, bits 4-15 are mantissa
	//	pUSART_Handler->pUSART->BRR |= (pUSART_Handler->USART_Pin_Config.USART_BaudRate); //fraction
	//	pUSART_Handler->pUSART->BRR |= (pUSART_Handler->USART_Pin_Config.USART_BaudRate << 4); //mantissa

	/** Transmitter enable **/
	pUSART_Handler->pUSART->CR1 &= ~(1ul << 3);
	pUSART_Handler->pUSART->CR1 |= (1ul << 3);
	//Should we put a delay there ?

	/** Data to send **/
	//clear necessary?
	pUSART_Handler->pUSART->DR &= ~(0x000000FF);
	temp |= (pUSART_Handler->pUSART->CR1 >> 10) && 1ul;
	if(temp == 1)	//if parity enabled, MSB replaced by parity bit
	{
		pUSART_Handler->pUSART->DR |= (uint32_t)(pUSART_TX_Buffer); //parity enabled,TODO push one less bit in the DR
	}
	else
	{
		pUSART_Handler->pUSART->DR |= (uint32_t)(pUSART_TX_Buffer);
	}

	temp |= ((pUSART_Handler->pUSART->SR >> 6) & 1ul);
	while(temp != 1)	//wait until TC bit is 1
	{
	}
}
