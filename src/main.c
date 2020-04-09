/*
 * main.c
 *
 *  Created on: 6 de jul de 2019
 *      Author: Fernando
 */


/*
 * Includes
 */

#include <string.h>

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_delay.h"

//LED PA5
//Button PC13

/*
 * Functions declarations
 */
void GPIO_Conf(void);
void delay(void);
void SPI_Conf(void);



/*
 * START PROGRAM
 */
int main()
{
	uint8_t testdata[] = "Hello, World!";

	SysTickInit();
	GPIO_Conf();

	SPI_Conf();

	SPI_SendData(SPI2, testdata, (uint32_t)strlen(testdata));

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay_ms(1000);
	}

	return 0;
}


/*
 * EXTI Handler
 */
void EXTI15_10_IRQHandler(void){
	GPIO_Clear_Interrupt(GPIO_PIN_13);

	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);

	delay_ms(100);


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
	 *  PB14 or PC2 - AF5 - SPI2_MISO
	 *  PB15 or PC3 - AF5 - SPI2_MOSI
	 *  PB13 or PC7 - AF5 - SPI2_SCK
	 *  PB12 or PB9 - AF5 - SPI2_NSS
	 */
#define SPI_TEST

	GPIO_Handle_t _SPIO2_PINS;
	_SPIO2_PINS.pGPIOX = GPIOB;
	_SPIO2_PINS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	_SPIO2_PINS.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF5;
	_SPIO2_PINS.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	_SPIO2_PINS.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	_SPIO2_PINS.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//MISO
	_SPIO2_PINS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&_SPIO2_PINS);
	//SCLK
	_SPIO2_PINS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&_SPIO2_PINS);

#ifndef SPI_TEST
	//MOSI
	_SPIO2_PINS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&_SPIO2_PINS);
	//NSS
	_SPIO2_PINS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&_SPIO2_PINS);
#endif

	SPI_Handle_t SPIO2Handle;

	SPIO2Handle.pSPIx = SPI2;
	SPIO2Handle.SPIConfig.SPI_BusConfig = SPI_BUSCONFIG_FD;
	SPIO2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIO2Handle.SPIConfig.SPI_SPI_SclkSpeed = SPI_SCLKSPEED_DIV2; //Max speed
	SPIO2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPIO2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIO2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIO2Handle.SPIConfig.SPI_SSM = SPI_SSM_ENABLE; //Software slave management
	SPI_Init(&SPIO2Handle);

	//Put SSI to 1
	SPI_Config_SSI(SPIO2Handle.pSPIx, ENABLE);
	//Enable the Peripheral
	SPI_PeripheralControl(SPIO2Handle.pSPIx, ENABLE);



}



void GPIO_Conf(void)
{
	GPIO_Handle_t _GPIOA, _GPIOC;

	/*
	 * Configure LED
	 */

	_GPIOA.pGPIOX = GPIOA;
	_GPIOA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	_GPIOA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	_GPIOA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	_GPIOA.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	_GPIOA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&_GPIOA);

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

