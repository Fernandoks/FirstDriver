/*
 * stm32f446xx_usart.c
 *
 *  Created on: Apr 29, 2020
 *      Author: fernandoks
 */

#include "stm32f446xx_usart.h"
#include "stm32f446xx_rcc.h"

/*
 * Peripheral clock
 */
Status_t UART_PeriClockControl(UART_RegDef_t *pUARTx, uint8_t EnableDisable)
{
	if (EnableDisable == ENABLE)
		{
			if (pUARTx == UART1)
			{
				UART1_PCLK_EN();
			}
			else if(pUARTx == UART2)
			{
				UART2_PCLK_EN();
			}
			else if(pUARTx == UART3)
			{
				UART3_PCLK_EN();
			}
			else if(pUARTx == UART4)
			{
				UART4_PCLK_EN();
			}
			else if(pUARTx == UART5)
			{
				UART5_PCLK_EN();
			}
			else if(pUARTx == UART6)
			{
				UART6_PCLK_EN();
			}
			else
			{
				return STATUS_ERROR;
			}
			return STATUS_OK;
		}
	else if (EnableDisable == DISABLE)
	{
		if (pUARTx == UART1)
		{
			UART1_PCLK_DI();
		}
		else if(pUARTx == UART2)
		{
			UART2_PCLK_DI();
		}
		else if(pUARTx == UART3)
		{
			UART3_PCLK_DI();
		}
		else if(pUARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUARTx == UART6)
		{
			UART6_PCLK_DI();
		}
		else
		{
			return STATUS_ERROR;
		}
		return STATUS_OK;
	}
	else return STATUS_ERROR;

}

/*
 * Init and De-Init
 */
Status_t UART_Init(UART_Handle_t *pUARTHandle)
{
	UART_PeriClockControl(pUARTHandle->pUARTx, ENABLE);

	uint32_t CR1temp = 0;
	uint32_t CR2temp = 0;
	uint32_t CR3temp = 0;

	//Mode
	if ( pUARTHandle->UARTConfig.USART_Mode == UART_MODE_TX)
	{
		CR1temp |= (1ul << UART_CR1_TE);
		CR1temp &= ~(1ul << UART_CR1_RE);
	}
	else if (pUARTHandle->UARTConfig.USART_Mode == UART_MODE_RX)
	{
		CR1temp &= ~(1ul << UART_CR1_TE);
		CR1temp |= (1ul << UART_CR1_RE);
	}
	else if (pUARTHandle->UARTConfig.USART_Mode == UART_MODE_TXRX)
	{
		CR1temp |= (1ul << UART_CR1_TE);
		CR1temp |= (1ul << UART_CR1_RE);
	}
	else return STATUS_ERROR;

	//Wordlength
	if ( pUARTHandle->UARTConfig.USART_WordLength == UART_WORD_8BITS)
	{
		CR1temp &= ~(1ul << UART_CR1_M);
	}
	else if (pUARTHandle->UARTConfig.USART_WordLength == UART_WORD_9BITS)
	{
		CR1temp |= (1ul << UART_CR1_M);
	}
	else return STATUS_ERROR;

	//Parity
	if ( pUARTHandle->UARTConfig.USART_Parity == UART_PARITY_DISABLE)
	{
		CR1temp &= ~(1ul << UART_CR1_PCE);
	}
	else if (pUARTHandle->UARTConfig.USART_Parity == UART_PARITY_EN_EVEN)
	{
		CR1temp |= (1ul << UART_CR1_PCE);
		CR1temp &= ~(1ul << UART_CR1_PS);
	}
	else if (pUARTHandle->UARTConfig.USART_Parity == UART_PARITY_EN_ODD)
	{
		CR1temp |= (1ul << UART_CR1_PCE);
		CR1temp |= (1ul << UART_CR1_PS);
	}
	else return STATUS_ERROR;

	//Stopbits - default 1
	if (pUARTHandle->UARTConfig.USART_StopBits == UART_STOPBITS_1)
	{
		CR2temp &= ~(3ul << UART_CR2_STOP);
	}
	else if (pUARTHandle->UARTConfig.USART_StopBits == UART_STOPBITS_HALF)
	{
		CR2temp |= (1ul << UART_CR2_STOP);
	}
	else if (pUARTHandle->UARTConfig.USART_StopBits == UART_STOPBITS_2)
	{
		CR2temp |= (2ul << UART_CR2_STOP);
	}
	else if (pUARTHandle->UARTConfig.USART_StopBits == UART_STOPBITS_1HALF)
	{
		CR2temp|= (3ul << UART_CR2_STOP);
	}
	else return STATUS_ERROR;



	//Flowcontrol
	if ( pUARTHandle->UARTConfig.USART_HWFlowControl == UART_FLOWCONTROL_NONE)
	{
		CR3temp &= ~(1ul << UART_CR3_CTSE);
		CR3temp &= ~(1ul << UART_CR3_RTSE);
	}
	else if (pUARTHandle->UARTConfig.USART_HWFlowControl == UART_FLOWCONTROL_CTS)
	{
		CR3temp |= (1ul << UART_CR3_CTSE);
		CR3temp &= ~(1ul << UART_CR3_RTSE);
	}
	else if (pUARTHandle->UARTConfig.USART_HWFlowControl == UART_FLOWCONTROL_RTS)
	{
		CR3temp &= ~(1ul << UART_CR3_CTSE);
		CR3temp |= (1ul << UART_CR3_RTSE);
	}
	else if (pUARTHandle->UARTConfig.USART_HWFlowControl == UART_FLOWCONTROL_CTS_RTS)
	{
		CR3temp |= (1ul << UART_CR3_CTSE);
		CR3temp |= (1ul << UART_CR3_RTSE);
	}
	else return STATUS_ERROR;


	//baudrate
	if ( UART_SetBaudRate(pUARTHandle) == STATUS_ERROR)
	{
		return STATUS_ERROR;
	}

	/*
	 * Using temporary functions to guarantee atomic functions
	 */
	pUARTHandle->pUARTx->CR1 |= CR1temp;
	pUARTHandle->pUARTx->CR2 |= CR2temp;
	pUARTHandle->pUARTx->CR3 |= CR3temp;


	UART_PeripheralControl(pUARTHandle->pUARTx, ENABLE);

	return STATUS_OK;
}

Status_t UART_DeInit(UART_RegDef_t *pUARTx)
{

	if (pUARTx == UART1)
	{
		UART1_RESET();
	}
	else if (pUARTx == UART2)
	{
		UART2_RESET();
	}
	else if (pUARTx == UART3)
	{
		UART3_RESET();
	}
	else if (pUARTx == UART4)
	{
		UART4_RESET();
	}
	else if (pUARTx == UART5)
	{
		UART5_RESET();
	}
	else if (pUARTx == UART6)
	{
		UART6_RESET();
	}
	else
	{
		return STATUS_ERROR;
	}
	return STATUS_OK;

}


Status_t UART_SetBaudRate(UART_Handle_t *pUARTHandle)
{
	//USART1 and 6 are APB2
	//USART2,3,4,5 are APB1

	/*
	 * UART_BRR
	 * Fraction - 4 bits
	 * Mantissa - 12 bits
	 * USARTDivider = fclk/ ( 8 * (2-over8) * BaudRate )
	 */
	uint32_t tempreg=0;

	// USART_BaudRate;
	uint32_t UART_Div = 0;
	uint32_t UART_Clk = 0;
	uint32_t mantissa, fraction;

	uint32_t BaudRate = pUARTHandle->UARTConfig.USART_BaudRate;

	if ( (pUARTHandle->pUARTx == UART1) || (pUARTHandle->pUARTx == UART6))
	{
		UART_Clk = RCC_GetPCLK1();
	}
	else
	{
		UART_Clk = RCC_GetPCLK2();
	}

	uint32_t OVER8 =  ((pUARTHandle->pUARTx->CR1 & (1 << UART_CR1_OVER8)) >> UART_CR1_OVER8);

	//Test OVER8
	UART_Div = (100 * UART_Clk)/(8 * (2 - OVER8) * BaudRate);

	//Mantissa
	mantissa = UART_Div/100;
	tempreg |= mantissa << 4;

	//Fraction
	fraction = (UART_Div - (mantissa* 100));

	if(pUARTHandle->pUARTx->CR1 & ( 1 << UART_CR1_OVER8))
	{
	  //OVER8 = 1 , DIV_Fraction3 bit is not considered and must be kept cleared
		fraction = ((( fraction * 8)+ 50) / 100)& ((uint8_t)0x07); //The 50 is a round up factor because we multiply by 100 (0,5*100).

	}else
	{
	   //over sampling by 16
		fraction = ((( fraction * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}
	tempreg |= fraction;

	//copy the value of tempreg in to BRR register
	pUARTHandle->pUARTx->BRR = tempreg;


	return STATUS_OK;
}


/*
 * Data send and Receive
 */
void UART_SendData(UART_Handle_t *pUARTHandle)
{
	uint8_t *pTxBuffer =  pUARTHandle->pTxBuffer;
	uint32_t Lenght = pUARTHandle->TxLen;

	for(uint32_t i = 0 ; i < Lenght; ++i)
	{
		//Wait until TXE is set
		while( UART_GetFlagStatus(pUARTHandle->pUARTx,UART_TXE_FLAG) != FLAG_SET);

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUARTHandle->UARTConfig.USART_WordLength == UART_WORD_9BITS)
		{
			//if 9BIT, DR is loaded with 2bytes masking the bits other than first 9 bits
			pUARTHandle->pUARTx->DR = ( *((uint16_t*)pTxBuffer) & ((uint16_t)0x01FF) );
			//Parity
			if(pUARTHandle->UARTConfig.USART_Parity == UART_PARITY_DISABLE)
			{
				//If no Parity we will transmitt 9bits of data
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer so the 9th bit is the parity
				pTxBuffer++;
			}
		}
		else
		{
			//8bits
			pUARTHandle->pUARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}
	//wait till TC flag is set
	while( UART_GetFlagStatus(pUARTHandle->pUARTx,UART_TC_FLAG) != FLAG_SET);
}


void UART_ReceiveDataBlock(UART_Handle_t *pUARTHandle, uint8_t *pRxBuffer, uint32_t Lenght)
{
	for(uint32_t i = 0 ; i < Lenght; ++i)
	{
		//Wait until RXNE is set
		while( UART_GetFlagStatus(pUARTHandle->pUARTx,UART_RXNE_FLAG) != FLAG_SET);

         // USART_WordLength for 9BIT or 8BIT in a frame
		if(pUARTHandle->UARTConfig.USART_WordLength == UART_WORD_9BITS)
		{
			//USART_ParityControl
			if(pUARTHandle->UARTConfig.USART_Parity == UART_PARITY_DISABLE)
			{
				//read  first 9 bits -  DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUARTHandle->pUARTx->DR  & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				 *pRxBuffer = (pUARTHandle->pUARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			//8bits
			//USART_ParityControl control
			if(pUARTHandle->UARTConfig.USART_Parity == UART_PARITY_DISABLE)
			{

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUARTHandle->pUARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used
				//read only 7 bits
				 *pRxBuffer = (uint8_t) (pUARTHandle->pUARTx->DR  & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
	pUARTHandle->pRxBuffer = (pRxBuffer - Lenght);
}



void UART_ReceiveDataString(UART_Handle_t *pUARTHandle, uint8_t *pRxBuffer)
{
	uint32_t Length = 0;
	uint8_t temp;

	do
	{
		//Wait until RXNE is set
		while( UART_GetFlagStatus(pUARTHandle->pUARTx,UART_RXNE_FLAG) != FLAG_SET);

         // USART_WordLength for 9BIT or 8BIT in a frame
		if(pUARTHandle->UARTConfig.USART_WordLength == UART_WORD_9BITS)
		{
			//USART_ParityControl
			if(pUARTHandle->UARTConfig.USART_Parity == UART_PARITY_DISABLE)
			{
				//read  first 9 bits -  DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUARTHandle->pUARTx->DR  & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				 *pRxBuffer = (pUARTHandle->pUARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			//8bits
			//USART_ParityControl control
			if(pUARTHandle->UARTConfig.USART_Parity == UART_PARITY_DISABLE)
			{

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUARTHandle->pUARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used
				//read only 7 bits
				 *pRxBuffer = (uint8_t) (pUARTHandle->pUARTx->DR  & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
		Length++;
		temp = *(pRxBuffer-1);
	}
	while (temp != '\n');


	pUARTHandle->pRxBuffer = (pRxBuffer - Length);
	pUARTHandle->RxLen = Length;
}



UART_Events_t UART_SendData_IT(UART_Handle_t *pUARTHandle)
{
	return 0;
}

UART_Events_t UART_ReceiveData_IT(UART_Handle_t *pUARTHandle)
{
	return 0;
}


/*
 * Peripheral Status
 */
FLAG_Status_t UART_GetFlagStatus(UART_RegDef_t *pUARTx, uint32_t FlagName)
{
	if ( pUARTx->SR & FlagName )
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


uint8_t UART_ClearFlag(UART_RegDef_t *pUARTx, uint32_t FlagName)
{

	return 0;
}


void UART_PeripheralControl(UART_RegDef_t *pUARTx, uint32_t EnDis)
{

	if(EnDis == ENABLE)
	{
		pUARTx->CR1 |= (1 << 13);
	}else
	{
		pUARTx->CR1 &= ~(1 << 13);
	}

}



/*
 * Callback
 */
uint8_t UART_EventCallback(UART_Handle_t *pUARTHandle, UART_Events_t Event)
{

	return 0;
}

