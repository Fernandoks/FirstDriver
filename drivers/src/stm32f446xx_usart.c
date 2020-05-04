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
	assert(IS_UART(pUARTx));

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
	assert(IS_UART(pUARTHandle->pUARTx));
	assert(IS_UART_MODE(pUARTHandle->UARTConfig.USART_Mode));
	assert(IS_UART_BAUDRATE(pUARTHandle->UARTConfig.USART_BaudRate));
	assert(IS_UART_FLOW(pUARTHandle->UARTConfig.USART_HWFlowControl));
	assert(IS_UART_PARITY(pUARTHandle->UARTConfig.USART_Parity));
	assert(IS_UART_STOPBITS(pUARTHandle->UARTConfig.USART_StopBits));
	assert(IS_UART_WORD(pUARTHandle->UARTConfig.USART_WordLength));


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
	assert(IS_UART(pUARTx));

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


static Status_t UART_SetBaudRate(UART_Handle_t *pUARTHandle)
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

/*********************************************************************
 *
 *  SEND / RECEIVE - BLOCKING
 *
 *********************************************************************/

void UART_SendData(UART_Handle_t *pUARTHandle)
{

	assert(IS_UART(pUARTHandle->pUARTx));
	assert(pUARTHandle->pTxBuffer != NULL);
	assert(pUARTHandle->TxLen >= 0);

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
	assert(IS_UART(pUARTHandle->pUARTx));
	assert(pRxBuffer != NULL);
	assert(Lenght >= 0);

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


/*********************************************************************
 *
 *  INTERRUPT
 *
 *********************************************************************/

void UART_IRQInterruptConfig(IRQn_Type IRQNumber, uint8_t EnableDisable)
{

	if (EnableDisable == ENABLE)
	{
		if (IRQNumber < 32){
			//NVIC_ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if( IRQNumber >= 32 && IRQNumber < 64){
			//NVIC_ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			//NVIC_ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else if (EnableDisable == DISABLE)
	{
		if (IRQNumber < 32){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if( IRQNumber >= 32 && IRQNumber < 64){
			//NVIC_ICER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			//NVIC_ICER2
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}

void UART_IRQPriorityConfig(IRQn_Type IRQNumber,uint32_t IRQPriority)
{

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_IRQ = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDRESS + (iprx * 4)) = (IRQPriority << shift_IRQ);

}




__inline void UART_IRQ_Control(UART_Handle_t *pUARTHandle, uint8_t CR1IRQbit, uint8_t CMD)
{
	if (CMD == ENABLE)
	{
		pUARTHandle->pUARTx->CR1 |= (1 << CR1IRQbit);
	}
	else
	{
		pUARTHandle->pUARTx->CR1 &= ~(1 << CR1IRQbit);
	}
}


UART_States_t UART_SendDataBlockIT(UART_Handle_t *pUARTHandle,uint8_t *pTxBuffer, uint32_t Lenght)
{
	UART_States_t txstate = pUARTHandle->TxState;

	if(txstate != USART_BUSY_TX)
	{
		pUARTHandle->TxLen = Lenght;
		pUARTHandle->pTxBuffer = pTxBuffer;
		pUARTHandle->TxState = USART_BUSY_TX;

		//Enable TXE
		UART_IRQ_Control(pUARTHandle,UART_CR1_TXEIE,ENABLE);

		//EnableTC
		UART_IRQ_Control(pUARTHandle,UART_CR1_TCIE,ENABLE);
	}
	return txstate;
}


UART_States_t UART_ReceiveBlockDataIT(UART_Handle_t *pUARTHandle,uint8_t *pRxBuffer, uint32_t Lenght)
{
	UART_States_t rxstate = pUARTHandle->RxState;

	if(rxstate != UART_BUSY_RX)
	{
		pUARTHandle->RxLen = Lenght;
		pUARTHandle->pRxBuffer = pRxBuffer;
		pUARTHandle->RxState = UART_BUSY_RX;

		(void)pUARTHandle->pUARTx->DR;

		//Enable RXNE
		UART_IRQ_Control(pUARTHandle,UART_CR1_RXNEIE,ENABLE);

	}

	return rxstate;
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


FLAG_Status_t UART_ClearFlag(UART_RegDef_t *pUARTx, uint32_t FlagName)
{
	pUARTx->SR &= ~(1 << FlagName);

	FLAG_Status_t flag = UART_GetFlagStatus(pUARTx, FlagName);
	return flag;
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

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void UART_IRQHandling(UART_Handle_t *pUARTHandle)
{
	uint16_t *pbuffer;

	uint32_t SRreg = pUARTHandle->pUARTx->SR;
	uint32_t CR1reg = pUARTHandle->pUARTx->CR1;
	uint32_t CR3reg = pUARTHandle->pUARTx->CR3;

	//Check the state of TC and TCEIE
	if ( (SRreg & (1ul << UART_SR_TC)) && (CR1reg & (1ul << UART_CR1_TCIE) ) )
	{
		//close transmission and call application callback if TxLen is zero
		if ( pUARTHandle->TxState == USART_BUSY_TX)
		{
			if(! pUARTHandle->TxLen )
			{
				pUARTHandle->pUARTx->SR &= ~( 1 << UART_SR_TC);

				pUARTHandle->TxState = UART_READY;
				pUARTHandle->pTxBuffer = NULL;
				pUARTHandle->TxLen = 0;

				UART_EventCallback(pUARTHandle,UART_EVENT_TC);
			}
		}
	}

	//Check the state of TXE and TXEIE
	if ( (SRreg & (1ul << UART_SR_TXE)) && (CR1reg & (1ul << UART_CR1_TXEIE) ) )
	{
		if(pUARTHandle->TxState == USART_BUSY_TX)
		{
			//Keep sending data until Txlen == 0
			if(pUARTHandle->TxLen > 0)
			{
				//9BIT or 8BIT in a frame
				if(pUARTHandle->UARTConfig.USART_WordLength == UART_WORD_9BITS)
				{
					pbuffer = (uint16_t*) pUARTHandle->pTxBuffer;
					pUARTHandle->pUARTx->DR = (*pbuffer & (uint16_t)0x01FF);

					//Parity
					if(pUARTHandle->UARTConfig.USART_Parity == UART_PARITY_DISABLE)
					{
						pUARTHandle->pTxBuffer++;
						pUARTHandle->pTxBuffer++;
						pUARTHandle->TxLen -= 2;
					}
					else
					{
						pUARTHandle->pTxBuffer++;
						pUARTHandle->TxLen-=1;
					}
				}
				else
				{
					//8bit data transfer
					pUARTHandle->pUARTx->DR = (*pUARTHandle->pTxBuffer  & (uint8_t)0xFF);

					pUARTHandle->pTxBuffer++;
					pUARTHandle->TxLen-=1;
				}
			}
			if (pUARTHandle->TxLen == 0 )
			{
				//TxLen is zero - ClearFLAG
				pUARTHandle->pUARTx->CR1 &= ~( 1 << UART_CR1_TXEIE);
			}
		}
	}


	//Check the state of RXE and RXEIE
	if ( (SRreg & (1ul << UART_SR_RXNE)) && (CR1reg & (1ul << UART_CR1_RXNEIE) ) )
	{

	}

	//CTS - Check the state of CTS, CTSE and CTSIE
	if ( (SRreg & (1ul << UART_SR_CTS)) && (CR1reg & (1ul << UART_CR3_CTSIE) ) )
	{
		pUARTHandle->pUARTx->SR &=  ~( 1 << UART_SR_CTS);
		UART_EventCallback(pUARTHandle,UART_EVENT_CTS);
	}

	//IDLE - Check the state of IDLE and IDLEIE
	if ( (SRreg & (1ul << UART_SR_TC)) && (CR1reg & (1ul << UART_CR1_TCIE) ) )
	{
		pUARTHandle->pUARTx->SR &=  ~( 1 << UART_SR_IDLE);
		UART_EventCallback(pUARTHandle,UART_EVENT_IDLE);
	}

	//OVERUN - Check the state of ORE and RXNEIE
	if ( (SRreg & (1ul << UART_SR_TC)) && (CR1reg & (1ul << UART_CR1_TCIE) ) )
	{
		pUARTHandle->pUARTx->SR &=  ~( 1 << UART_SR_ORE);
		UART_EventCallback(pUARTHandle,UART_EVENT_ORE);
	}

	//ERROR - Noise Flag, Overrun error and Framing Error

	if( CR3reg & ( 1 << UART_CR3_EIE) )
	{
		if( SRreg & ( 1 << UART_SR_FE))
		{
			UART_EventCallback(pUARTHandle,UART_EVENT_FE);
		}

		if( SRreg & ( 1 << UART_SR_NF) )
		{
			UART_EventCallback(pUARTHandle,UART_EVENT_NF);
		}

		if(SRreg & ( 1 << UART_SR_ORE) )
		{
			UART_EventCallback(pUARTHandle,UART_EVENT_ORE);
		}
	}


}


/*********************************************************************
 * @fn      		  - USART_EventCallback
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
__weak void UART_EventCallback(UART_Handle_t *pUARTHandle, UART_Events_t Event)
{
	__NOP();

}

