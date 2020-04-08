/*
 * main.c
 *
 *  Created on: 6 de jul de 2019
 *      Author: Fernando
 */


/*
 * Includes
 */
#include "stm32f446xx_gpio_driver.h"

//LED PA5
//Button PC13

/*
 * Functions declarations
 */
void GPIO_Conf(void);
void delay(void);




/*
 * START PROGRAM
 */
int main(){


	GPIO_Conf();

	while(1)
	{

		delay();
	}

	return 0;
}


/*
 * EXTI Handler
 */
void EXTI15_10_IRQHandler(void){
	GPIO_Clear_Interrupt(GPIO_PIN_13);

	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
	delay();

}

void GPIO_Conf(void){


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
	GPIO_PeriClockControl(GPIOA, ENABLE);
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
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&_GPIOC);

	//Button interrupt
	GPIO_IRQInterruptConfig(EXTI15_10_IRQn,ENABLE);
	GPIO_IRQPriorityConfig(EXTI15_10_IRQn,1);




}


void delay(void){

	for (volatile uint32_t i = 0; i < 500000; i++);
}
