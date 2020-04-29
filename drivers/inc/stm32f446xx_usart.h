/*
 * stm32f446xx_usart.h
 *
 *  Created on: Apr 29, 2020
 *      Author: fernandoks
 */

#ifndef INC_STM32F446XX_USART_H_
#define INC_STM32F446XX_USART_H_


#include "stm32f446xx.h"

/*
 * USART 1 and 6 connected to APB2
 * USART 2,3,4,5 connected to APB1
 */


/*********************************************************************************
 * UART states
 ********************************************************************************/
typedef enum
{
	UART_READY = 0,
	UART_BUSY_RX = 1,
	USART_BUSY_TX = 2
} UART_States_t;

/*********************************************************************************
 * Configuration Structure for UARTx
 ********************************************************************************/

typedef struct
{
	uint32_t USART_Mode;
	uint32_t USART_BaudRate;
	uint32_t USART_StopBits;
	uint32_t USART_WordLength;
	uint32_t USART_Parity;
	uint32_t USART_HWFlowControl;
} UART_Config_t;



/*********************************************************************************
 * Handle Structure for UARTx
 ********************************************************************************/

typedef struct
{
	UART_RegDef_t	*pUARTx;
	UART_Config_t	UARTConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	UART_States_t	TxState;
	UART_States_t	RxState;
} UART_Handle_t;


/*********************************************************************************
 * Configuration Structure MACROS for UARTx
 ********************************************************************************/

// USART_Mode;
#define UART_MODE_TX				0
#define UART_MODE_RX				1
#define UART_MODE_TXRX				2


// USART_BaudRate;
#define UART_BAUDRATE_1200			1200
#define UART_BAUDRATE_2400			2400
#define UART_BAUDRATE_9600			9600
#define UART_BAUDRATE_19200			19200
#define UART_BAUDRATE_38400			38400
#define UART_BAUDRATE_57600			57600
#define UART_BAUDRATE_115200		115200
#define UART_BAUDRATE_230400		230400
#define UART_BAUDRATE_460800		460800
#define UART_BAUDRATE_921600		921600
#define UART_BAUDRATE_2M			2000000
#define UART_BAUDRATE_3M			3000000

// USART_StopBits;
#define UART_STOPBITS_1			0
#define UART_STOPBITS_HALF			1
#define UART_STOPBITS_2			2
#define UART_STOPBITS_1HALF		3

// USART_WordLength;
#define UART_WORD_8BITS			0
#define UART_WORD_9BITS			1

// USART_Parity;
#define UART_PARITY_DISABLE		0
#define UART_PARITY_EN_EVEN		1
#define UART_PARITY_EN_ODD			2

// USART_HWFlowControl;
#define UART_FLOWCONTROL_NONE		0
#define UART_FLOWCONTROL_CTS		1
#define UART_FLOWCONTROL_RTS		2
#define UART_FLOWCONTROL_CTS_RTS	3


#define USART_PE_FLAG					(1 << USART_SR_PE)
#define USART_FE_FLAG					(1 << USART_SR_FE)
#define USART_NF_FLAG					(1 << USART_SR_NF)
#define USART_ORE_FLAG					(1 << USART_SR_ORE)
#define USART_IDLE_FLAG					(1 << USART_SR_IDLE)
#define USART_RXNE_FLAG					(1 << USART_SR_RXNE)
#define USART_TC_FLAG					(1 << USART_SR_TC)
#define USART_TXE_FLAG					(1 << USART_SR_TXE)
#define USART_LBD_FLAG					(1 << USART_SR_LBD)
#define USART_CTS_FLAG					(1 << USART_SR_CTS)

typedef enum
{
	FLAG_SET = 0,
	FLAG_RESET = 1,
} FLAG_Status_t;


/*********************************************************************************
 * IRQ Events
 * Event Flags
 * TODO: TC, RXE
 ********************************************************************************/
typedef enum
{
	SPI_EVENT_TC = 0,
	SPI_EVENT_RXE = 1,
} UART_Events_t;


/*
 * Peripheral clock
 */
Status_t UART_PeriClockControl(UART_RegDef_t *pUARTx, uint8_t EnableDisable); /* First parameter is to select with peripheral (port) so we use base address */

/*
 * Init and De-Init
 */
Status_t UART_Init(UART_Handle_t *pUARTHandle);
Status_t UART_DeInit(UART_RegDef_t *pUARTx);
Status_t UART_SetBaudRate(UART_RegDef_t *pUARTx);

/*
 * Data send and Receive
 */
void UART_SendData(UART_RegDef_t *pUARTx, uint8_t *pTXBuffer, uint32_t Lenght);
void UART_ReceiveData(UART_RegDef_t *pUARTx, uint8_t *pRXBuffer, uint32_t Lenght);
UART_Events_t UART_SendData_IT(UART_Handle_t *pUARTHandle, uint8_t *pTXBuffer, uint32_t Lenght);
UART_Events_t UART_ReceiveData_IT(UART_Handle_t *pUARTHandle, uint8_t *pRXBuffer, uint32_t Lenght);
/*
 * Peripheral Status
 */
uint8_t UART_GetFlagStatus(UART_RegDef_t *pUARTx, uint32_t FlagName);
uint8_t UART_ClearFlag(UART_RegDef_t *pUARTx, uint32_t FlagName);
uint8_t UART_PeripheralControl(UART_RegDef_t *pUARTx, uint32_t FlagName);

/*
 * Callback
 */
uint8_t UART_EventCallback(UART_Handle_t *pUARTHandle, UART_Events_t Event);




#endif /* INC_STM32F446XX_USART_H_ */
