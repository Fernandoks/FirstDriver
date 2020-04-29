/*
 * stm32f446xx_usart.c
 *
 *  Created on: Apr 29, 2020
 *      Author: fernandoks
 */

#include "stm32f446xx_usart.h"

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
	UART_PeriClockControl(pUARTHandle->pUARTx, Enable);

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
	pUARTHandle->pUARTx->CR2 &= ~(3ul << UART_CR2_STOP);
	if (pUARTHandle->UARTConfig.USART_StopBits == UART_STOPBITS_HALF)
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
		CR3temp->pUARTx->CR3 &= ~(1ul << UART_CR3_RTSE);
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
	if ( UART_SetBaudRate(pUARTHandle->pUARTx) == STATUS_ERROR)
	{
		return STATUS_ERROR;
	}

	/*
	 * Using temporary functions to guarantee atomic functions
	 */
	pUARTHandle->pUARTx->CR1 |= CR1temp;
	pUARTHandle->pUARTx->CR2 |= CR2temp;
	pUARTHandle->pUARTx->CR3 |= CR3temp;

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


Status_t UART_SetBaudRate(UART_RegDef_t *pUARTx)
{
	// USART_BaudRate;
	#define USART_BAUDRATE_1200			1200
	#define USART_BAUDRATE_2400			2400
	#define USART_BAUDRATE_9600			9600
	#define USART_BAUDRATE_19200		19200
	#define USART_BAUDRATE_38400		38400
	#define USART_BAUDRATE_57600		57600
	#define USART_BAUDRATE_115200		115200
	#define USART_BAUDRATE_230400		230400
	#define USART_BAUDRATE_460800		460800
	#define USART_BAUDRATE_921600		921600
	#define USART_BAUDRATE_2M			2000000
	#define USART_BAUDRATE_3M			3000000


	return STATUS_OK;
}

/*
 * Data send and Receive
 */
void UART_SendData(UART_RegDef_t *pUARTx, uint8_t *pTXBuffer, uint32_t Lenght)
{

}


void UART_ReceiveData(UART_RegDef_t *pUARTx, uint8_t *pRXBuffer, uint32_t Lenght)
{

}


UART_Events_t UART_SendData_IT(UART_Handle_t *pUARTHandle, uint8_t *pTXBuffer, uint32_t Lenght)
{
	return 0;
}

UART_Events_t UART_ReceiveData_IT(UART_Handle_t *pUARTHandle, uint8_t *pRXBuffer, uint32_t Lenght)
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


uint8_t UART_PeripheralControl(UART_RegDef_t *pUARTx, uint32_t FlagName)
{

	return 0;
}

/*
 * Callback
 */
uint8_t UART_EventCallback(UART_Handle_t *pUARTHandle, UART_Events_t Event)
{

	return 0;
}

