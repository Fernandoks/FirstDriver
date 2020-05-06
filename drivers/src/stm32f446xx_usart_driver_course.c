/*
 * stm32f446xx_usart_driver_course.c
 *
 *  Created on: Apr 24, 2020
 *      Author: quentin
 */

#include "stm32f446xx_usart_driver_course.h"
#include "stm32f446xx_rcc.h"

void UART_PeriClockControl(USART_RegDef_t* pUSARTx, uint8_t EnableDisable)
{

	if(EnableDisable == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == USART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == USART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else if(EnableDisable == DISABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == USART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == USART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}

/* USART Initialization */
void USART_Initialize(USART_Handler_TypedDef* pUSART_Handler)
{
	//baud rate equation : txrx baud = Fclk/(8*(2-oversampling)*USARTDIV)
	float USARTDIV, SysClock;

	/* Enable UART */
	pUSART_Handler->pUSART->CR1 |= (1ul << 13);

	/**  WORDLENGTH **/
	pUSART_Handler->pUSART->CR1 &= ~(1ul << 12);
	pUSART_Handler->pUSART->CR1 |= (pUSART_Handler->USART_Pin_Config.USART_WordLength << 12);

	/** Stop bit **/
	pUSART_Handler->pUSART->CR2 &= (3ul << 12);
	pUSART_Handler->pUSART->CR2 |= (pUSART_Handler->USART_Pin_Config.USART_StopBits << 12);

	//UART4 & UART5 don't have CTS bit
	if((pUSART_Handler->pUSART == UART4) || (pUSART_Handler->pUSART == UART5))
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

void USART_SetBaudRate(USART_Handler_TypedDef* pUSART_Handler)
{
	uint16_t tempreg, mantissa, DIV_fraction, DIV_mantissa;
	float fraction;

	uint32_t usart_clock, usart_div;

	//UART1 and USART6 are on APB2 so they use PCLK2
	if((pUSART_Handler->pUSART == UART1) || (pUSART_Handler->pUSART == UART6))
	{
		usart_clock = RCC_GetPCLK2();
	}
	else
	{
		usart_clock = RCC_GetPCLK1();
	}

	usart_div = (usart_clock*100)/((8*2-pUSART_Handler->USART_Pin_Config.USART_OverSampling)*pUSART_Handler->USART_Pin_Config.USART_BaudRate);

	mantissa = usart_div/100;
	fraction = usart_div - (mantissa*100);
	fraction = fraction/100;

	if(pUSART_Handler->USART_Pin_Config.USART_OverSampling == 0)	//oversampling = 16, fraction coded on 4 bits
	{
		DIV_fraction = (fraction*16) + 0.5; //round to the nearest number
		if(DIV_fraction > 0xF)	//test to see if the fraction bits overflow
		{
			DIV_mantissa = mantissa + 1;
			DIV_fraction = 0;
		}
		else
		{
			DIV_mantissa = mantissa;
		}

	}
	else if(pUSART_Handler->USART_Pin_Config.USART_OverSampling == 1)	//oversampling = 8, fraction coded on 3 bits, fraction bit 3 must be left cleared
	{
		DIV_fraction = (fraction*8) + 0.5;	//round to the nearest number

		if(DIV_fraction > 0x7)	//test to see if the fraction bits overflow
		{
			DIV_mantissa = mantissa + 1;
			DIV_fraction = 0;
		}
		else
		{
			DIV_mantissa = mantissa;
		}
	}
	tempreg &= ~(0x0000FFFF);
	pUSART_Handler->pUSART->BRR &= tempreg; //bits 0-3 are fraction, bits 4-15 are mantissa

	tempreg |= (mantissa << 4);
	tempreg |= (fraction);
	pUSART_Handler->pUSART->BRR |= tempreg;
}

