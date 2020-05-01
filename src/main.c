/*
 * main.c
 *
 *  Created on: 6 de jul de 2019
 *      Author: Fernando
 */

//#define NDEBUG //Disable all asserts

/*
 * Includes
 */

#include <string.h>
#include <stdio.h>
#include <assert.h> //use NDEBUG do disable asserts

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_delay.h"
#include "stm32f446xx_usart.h"
#include "CircularUART.h"

//LED PA5
//Button PC13

#define NACK 					0xA5
#define ACK 					0xF5

//command codes
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define ARDUINO_SPI				SPI1
#define SPI1_PORT				GPIOA
#define SPI1_MOSI_PIN			GPIO_PIN_7
#define SPI1_MISO_PIN			GPIO_PIN_6
#define SPI1_SCLK_PIN			GPIO_PIN_5
#define SPI1_NSS_PIN			GPIO_PIN_4

#define USART2_PORT				GPIOA
#define USART2_TX_PIN			GPIO_PIN_2
#define USART2_RX_PIN			GPIO_PIN_3

#define DUMMY_BYTE				0xFF


/*
 * Functions declarations
 */
void GPIO_Conf(void);
void delay(void);
void SPI_Conf(void);
void UART_Conf(UART_Handle_t* _UART2Handler);


extern void initialise_monitor_handles();


uint8_t* TxData = "Hello, World!\r\n";
uint8_t RxData[100];

UART_Handle_t pUART2;

/*
 * START PROGRAM
 */
int main()
{


	initialise_monitor_handles();

	printf("Program starting\n");

	SysTickInit();
	GPIO_Conf();
	SPI_Conf();
	UART_Conf(&pUART2);


	pUART2.pTxBuffer = (uint8_t*)"Program Starting\r\n";
	pUART2.TxLen = strlen((char*)pUART2.pTxBuffer);
	UART_SendData(&pUART2);

	UART_IRQInterruptConfig(USART2_IRQn, ENABLE);
	UART_IRQPriorityConfig(USART2_IRQn,0);

	size_t size = 64;
	uint8_t* buffer = (uint8_t*)malloc((int)size*sizeof(uint8_t));

	cbuf_handle_t cbuf = CircularUART_Init(buffer, size);

	uint32_t txdatalen = strlen(TxData);
	for (int i = 0; i < txdatalen; ++i)
	{
		circular_buffer_push(cbuf, TxData[i]);
	}

	 CircularUART_Send(cbuf, &pUART2);


	while(1)
	{

		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay_ms(100);
	}

	return 0;
}


void USART2_IRQHandler(void)
{
	UART_IRQHandling(&pUART2);
}

/*
 * EXTI Handler
 */
void EXTI15_10_IRQHandler(void){
	uint8_t CMDCode = COMMAND_LED_CTRL;
	uint8_t Ack_byte = 0;
	uint8_t Dummy_Read = 0;
	uint8_t Dummy_Write = 0;
	uint8_t args[2] = {0};

	GPIO_Clear_Interrupt(GPIO_PIN_13);

	SPI_PeripheralControl(ARDUINO_SPI, ENABLE);

	//Send the command
	SPI_SendData(ARDUINO_SPI, &CMDCode, 1);
	printf("SPI SEND: %d\n",CMDCode);
	//Dummy read to clean
	SPI_ReceiveData(ARDUINO_SPI, &Dummy_Read, 1);
	printf("SPI Receive: %d\n",Dummy_Read);
	//Send the Dummy byte to shift
	SPI_SendData(ARDUINO_SPI, Dummy_Write, 1);
	printf("SPI SEND: %d\n",Dummy_Write);
	//get the ACK
	SPI_ReceiveData(ARDUINO_SPI, &Ack_byte, 1);
	if ( Ack_byte == ACK)
	{
		args[0] = 9;
		args[1] = 1;
		SPI_SendData(ARDUINO_SPI, &args, (uint32_t)strlen(args));
	}


	while(SPI_GetFlagStatus(ARDUINO_SPI, SPI_BSY_FLAG) );
	SPI_PeripheralControl(ARDUINO_SPI, DISABLE);

}

void UART_Conf(UART_Handle_t* _UART2Handler)
{

	/*
	 * Usart2
	 * TX - PA2
	 * RX - PA3
	 */
	GPIO_Handle_t _UART_Pins;
	_UART_Pins.pGPIOX = USART2_PORT;
	_UART_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	_UART_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF7;
	_UART_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	_UART_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	_UART_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	_UART_Pins.GPIO_PinConfig.GPIO_PinNumber = USART2_TX_PIN;
	GPIO_Init(&_UART_Pins);

	_UART_Pins.GPIO_PinConfig.GPIO_PinNumber = USART2_RX_PIN;
	GPIO_Init(&_UART_Pins);


	_UART2Handler->pUARTx = UART2;
	_UART2Handler->UARTConfig.USART_BaudRate = UART_BAUDRATE_9600;
	_UART2Handler->UARTConfig.USART_HWFlowControl = UART_FLOWCONTROL_NONE;
	_UART2Handler->UARTConfig.USART_Mode = UART_MODE_TXRX;
	_UART2Handler->UARTConfig.USART_Parity = UART_PARITY_DISABLE;
	_UART2Handler->UARTConfig.USART_StopBits = UART_STOPBITS_1;
	_UART2Handler->UARTConfig.USART_WordLength = UART_FLOWCONTROL_NONE;
	UART_Init(_UART2Handler);





}

void SPI_Conf(void)
{
	/*
	 * SPI has 4 wires
	 * MOSI - Master output Slave Input
	 * MISO - Master Input Slave Output
	 * SCLK - Serial Clock
	 * NSS  - Select
	 * To find the correct pins to map, you need to verify the available alternate
	 *  functions in the Datasheet
	 *  SPI1
	 *  PA6 - AF5 - SPI1_MISO
	 *  PA7 - AF5 - SPI1_MOSI
	 *  PA5 - AF5 - SPI1_SCK
	 *  PA4 - AF5 - SPI1_NSS
	 *  SPI2
	 *  PB14 or PC2 - AF5 - SPI2_MISO
	 *  PB15 or PC3 - AF5 - SPI2_MOSI
	 *  PB13 or PC7 - AF5 - SPI2_SCK
	 *  PB12 or PB9 - AF5 - SPI2_NSS
	 */

	/*
	 * TESTING WITH ARDUINO BOAD
	 * F446 master
	 *	DFF = 0
	 *	SCLK = 2MHz?
	 *	Arduino Slave
	 *	PB5 - D13 - SCK
	 *	PB4 - D12 - MISO
	 *	PB3 - D11 - MOSI
	 *	PB2 - D10 - SS
	 *
	 *	Connection Scheme
	 *	SPI	- 	F446	-	Arduino Nano
	 *	MISO-	PB14	-	D12
	 *	MOSI-	PB15	-	D11
	 *	SCLK-	PB13	-	D13
	 *	SS	-	PB12	-	D10
	 *
	 */
//#define SPI
#ifdef SPI




	GPIO_Handle_t _SPIO1_PINS;
	_SPIO1_PINS.pGPIOX = SPI1_PORT;
	_SPIO1_PINS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	_SPIO1_PINS.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF5;
	_SPIO1_PINS.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	_SPIO1_PINS.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	_SPIO1_PINS.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//MISO
	_SPIO1_PINS.GPIO_PinConfig.GPIO_PinNumber = SPI1_MISO_PIN;
	GPIO_Init(&_SPIO1_PINS);
	//SCLK
	_SPIO1_PINS.GPIO_PinConfig.GPIO_PinNumber = SPI1_SCLK_PIN;
	GPIO_Init(&_SPIO1_PINS);
	//MOSI
	_SPIO1_PINS.GPIO_PinConfig.GPIO_PinNumber = SPI1_MOSI_PIN;
	GPIO_Init(&_SPIO1_PINS);
	//NSS
	_SPIO1_PINS.GPIO_PinConfig.GPIO_PinNumber = SPI1_NSS_PIN;
	GPIO_Init(&_SPIO1_PINS);

	SPI_Handle_t SPIO1Handle;
	SPIO1Handle.pSPIx = SPI1;
	SPIO1Handle.SPIConfig.SPI_BusConfig = SPI_BUSCONFIG_FD;
	SPIO1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIO1Handle.SPIConfig.SPI_SPI_SclkSpeed = SPI_SCLKSPEED_DIV8;
	SPIO1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPIO1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIO1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIO1Handle.SPIConfig.SPI_SSM = SPI_SSM_DISABLE; //Software slave management
	SPI_Init(&SPIO1Handle);



	//Put SSI to 1 - used only if you use the Software Slave
	//SPI_Config_SSI(SPIO1Handle.pSPIx, ENABLE);
	//Enable the Peripheral

	/*
	 *When you enable the SSOE in Master Mode the hardware will control the NSS Pin
	 *The NSS pin will automatically go to 0 when you enable the SPI
	 */
	SPI_Config_SSOE(SPI1,ENABLE);

#endif

}



void GPIO_Conf(void)
{
	GPIO_Handle_t _GPIOA, _GPIOC;

#ifndef SPI
	/*
	 * Configure LED
	 * **If SPI is ON PA5 isn't available
	 */

	_GPIOA.pGPIOX = GPIOA;
	_GPIOA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	_GPIOA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	_GPIOA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	_GPIOA.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	_GPIOA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&_GPIOA);
#endif
	/*
	 * Configure Button
	 */
	_GPIOC.pGPIOX = GPIOC;
	_GPIOC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	_GPIOC.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	_GPIOC.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	_GPIOC.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	_GPIOC.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&_GPIOC);

	//Button interrupt
	GPIO_IRQInterruptConfig(EXTI15_10_IRQn,ENABLE);
	GPIO_IRQPriorityConfig(EXTI15_10_IRQn,1);




}

