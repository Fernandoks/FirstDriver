/*
 * stm32f446xx_usart_driver_course.c
 *
 *  Created on: Apr 24, 2020
 *      Author: quentin
 */

//UART4 & UART5 don't have CTS bit

#include "stm32f446xx_usart_driver_course.h"
#include "stm32f446xx_rcc.h"

void UART_PCLKControl(_Quentin_USART_RegDef_t* pUSARTx, uint8_t EnableDisable)
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
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
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
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
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
	uint32_t tempreg_CR1 = 0;
	uint32_t tempreg_CR2 = 0;

	/* Enable UART */
	tempreg_CR1 |= (1ul << USART_SHIFT_CR1_UE);

	/**  WORDLENGTH **/
	tempreg_CR1 &= ~(1ul << USART_SHIFT_CR1_M);
	tempreg_CR1 |= (pUSART_Handler->USART_Pin_Config.USART_WordLength << USART_SHIFT_CR1_M);

	/** Stop bit **/
	tempreg_CR2 &= ~(3ul << USART_SHIFT_CR2_STOP); //clear stop bits
	tempreg_CR2 |= (pUSART_Handler->USART_Pin_Config.USART_StopBits << USART_SHIFT_CR2_STOP);

	/** OVERSAMPLING **/
	tempreg_CR1 &= ~(1ul << USART_SHIFT_CR1_OVER8);
	tempreg_CR1 |= (pUSART_Handler->USART_Pin_Config.USART_OverSampling << USART_SHIFT_CR1_OVER8);

	/** PARITY **/
	tempreg_CR1 &= ~(1ul << USART_SHIFT_CR1_PCE);
	tempreg_CR1 |= (pUSART_Handler->USART_Pin_Config.USART_Parity << USART_SHIFT_CR1_PCE);

	/** BAUD RATE **/
	USART_SetBaudRate(pUSART_Handler);

	if(pUSART_Handler->USART_Pin_Config.USART_Mode == USART_MODE_TX)
	{
		tempreg_CR1 |= (1ul << USART_SHIFT_CR1_TE);
	}
	else if(pUSART_Handler->USART_Pin_Config.USART_Mode == USART_MODE_RX)
	{
		tempreg_CR1 |= (1ul << USART_SHIFT_CR1_RE);
	}

	pUSART_Handler->pUSART->CR1 |= tempreg_CR1;
	pUSART_Handler->pUSART->CR2 |= tempreg_CR2;

}

void USART_ReadData(USART_Handler_TypedDef* pUSART_Handler)
{

	//wait until data can be read
	for(uint32_t i=0; i < (pUSART_Handler->TxLength); i++){
		while(((pUSART_Handler->pUSART->SR >> USART_SHIFT_SR_RXNE) & 1ul) != 1)
		{
			if(pUSART_Handler->USART_Pin_Config.USART_Parity == USART_PARITY_ENABLE)	//if parity enabled, MSB replaced by parity bit
			{
				if(pUSART_Handler->USART_Pin_Config.USART_WordLength == USART_WORD_LENGTH_8)
				{
					*(pUSART_Handler->USART_RxBuffer) = (pUSART_Handler->pUSART->DR & (0x7F));
					pUSART_Handler->USART_RxBuffer++;
				}
				else if(pUSART_Handler->USART_Pin_Config.USART_WordLength == USART_WORD_LENGTH_9)
				{
					*(pUSART_Handler->USART_RxBuffer) = pUSART_Handler->pUSART->DR;
					pUSART_Handler->USART_RxBuffer++;
				}
			}
			else if(pUSART_Handler->USART_Pin_Config.USART_Parity == USART_PARITY_DISABLE)
			{
				if(pUSART_Handler->USART_Pin_Config.USART_WordLength == USART_WORD_LENGTH_8)
				{
					*(pUSART_Handler->USART_RxBuffer) = pUSART_Handler->pUSART->DR;
					pUSART_Handler->USART_RxBuffer++;
				}
				else if(pUSART_Handler->USART_Pin_Config.USART_WordLength == USART_WORD_LENGTH_9)
				{
					*((uint16_t*)(pUSART_Handler->USART_RxBuffer)) = (pUSART_Handler->pUSART->DR & (0x01FF));
					pUSART_Handler->USART_RxBuffer++;
					pUSART_Handler->USART_RxBuffer++;
				}
			}
		}
	}
}

void USART_WriteData(USART_Handler_TypedDef* pUSART_Handler)
{
	//Procedure p801

	for(uint32_t i=0; i < (pUSART_Handler->TxLength); i++){
		while(((pUSART_Handler->pUSART->SR >> USART_SHIFT_SR_TXE) & 1ul) != 1)	//TODO implement flag to get
		{
			if(pUSART_Handler->USART_Pin_Config.USART_Parity == USART_PARITY_ENABLE)	//if parity enabled, MSB replaced by parity bit
			{
				if(pUSART_Handler->USART_Pin_Config.USART_WordLength == USART_WORD_LENGTH_8)
				{
					pUSART_Handler->pUSART->DR = *(pUSART_Handler->USART_TxBuffer) & (0x7F);	//8th bit is parity so we clear it
					pUSART_Handler->USART_TxBuffer++;
				}
				else if(pUSART_Handler->USART_Pin_Config.USART_WordLength == USART_WORD_LENGTH_9)
				{
					pUSART_Handler->pUSART->DR = *(pUSART_Handler->USART_TxBuffer);
					pUSART_Handler->USART_TxBuffer++;
				}
			}
			else if(pUSART_Handler->USART_Pin_Config.USART_Parity == USART_PARITY_DISABLE)
			{
				if(pUSART_Handler->USART_Pin_Config.USART_WordLength == USART_WORD_LENGTH_8)
				{
					pUSART_Handler->pUSART->DR = *(pUSART_Handler->USART_TxBuffer);
					pUSART_Handler->USART_TxBuffer++;
				}
				else if(pUSART_Handler->USART_Pin_Config.USART_WordLength == USART_WORD_LENGTH_9)
				{
					pUSART_Handler->pUSART->DR = (*((uint16_t*)pUSART_Handler->USART_TxBuffer) & 0x01FF);	//casting the pointer to uint16_t to send two uint8_t
					pUSART_Handler->USART_TxBuffer++;
					pUSART_Handler->USART_TxBuffer++;
				}
			}
		}
	}
	while(((pUSART_Handler->pUSART->SR >> USART_SHIFT_SR_TC) & 1ul) != 1)
	{
		//wait until transmission is complete
	}
}

void USART_SetBaudRate(USART_Handler_TypedDef* pUSART_Handler)
{
	uint16_t  mantissa, fraction, DIV_fraction, DIV_mantissa;
	uint32_t usart_clock, usart_div;

	uint32_t tempreg = 0;

	//UART1 and USART6 are on APB2 so they use PCLK2
	if((pUSART_Handler->pUSART == USART1) || (pUSART_Handler->pUSART == USART6))
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

	if(pUSART_Handler->USART_Pin_Config.USART_OverSampling == 0)	//oversampling = 16, fraction coded on 4 bits
	{
		DIV_fraction = ((fraction*16) + 50)/100; //round to the nearest number
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
		DIV_fraction = ((fraction*8) + 50)/100;	//round to the nearest number

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

	tempreg |= (DIV_mantissa << USART_SHIFT_BRR_MANTISSA);
	tempreg |= (DIV_fraction);
	pUSART_Handler->pUSART->BRR |= tempreg;
}

