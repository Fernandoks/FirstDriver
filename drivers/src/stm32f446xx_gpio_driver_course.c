/*
 * stm32f446xx_gpio_driver_course.c
 *
 *  Created on: Apr 8, 2020
 *      Author: quentin
 */

#include "stm32f446xx_gpio_driver_course.h"


/* Peripheral clock control */
void GPIO_PCLK(GPIO_RegDef_t* pGPIO, uint8_t status)
{
	if(status == ENABLE)
	{
		if(pGPIO == GPIOA)//can't do switch on a pointer
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIO == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIO == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIO == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIO == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIO == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIO == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIO == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else if(status == DISABLE)
	{
		if(pGPIO == GPIOA)//can't do switch on a pointer
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIO == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIO == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIO == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIO == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIO == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIO == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIO == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/* GPIO Initialization */
void GPIO_Initialize(GPIO_Handler_TypedDef* pGPIO_Handler)
{
	uint32_t temp = 0;//
	//Configure the mode
	//First, check if interrupt mode selected, if not do this
	if(pGPIO_Handler->GPIO_Pin_Config.GPIO_Mode <= GPIO_MODE_ANALOG)
	{
		pGPIO_Handler->pGPIO->MODER |= ((pGPIO_Handler->GPIO_Pin_Config.GPIO_Mode) << (2*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));
	}
	else //Configure everything for each interrupt mode
	{
	}
	//Configure Type
	temp = (pGPIO_Handler->GPIO_Pin_Config.GPIO_Type << pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number);
	pGPIO_Handler->pGPIO->OTYPER &= ~(1ul << pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number); //clear the requested port bit
	pGPIO_Handler->pGPIO->OTYPER |= temp;
	temp = 0;
	//Configure Speed
	temp = (pGPIO_Handler->GPIO_Pin_Config.GPIO_Speed << (2*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));
	pGPIO_Handler->pGPIO->OSPEEDER &= ~(3ul << (2*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));//clear the requested port bits
	pGPIO_Handler->pGPIO->OSPEEDER |= temp;
	temp = 0;
	//Select pull-down/pull-up
	temp = (pGPIO_Handler->GPIO_Pin_Config.GPIO_PUPD << (2*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));
	pGPIO_Handler->pGPIO->PUPDR &= ~(3ul << (2*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));//clear the requested port bits
	pGPIO_Handler->pGPIO->PUPDR |= temp;
	temp = 0;

	//Alternate function AFRL & AFRH
	if(pGPIO_Handler->GPIO_Pin_Config.GPIO_Mode == GPIO_MODE_ALTER)
	{
		if(pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number <= 7)
		{
			temp = (pGPIO_Handler->GPIO_Pin_Config.GPIO_AlternateMode << (4*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));
			pGPIO_Handler->pGPIO->AFRL &= ~(15ul << (4*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));//clear the requested port bits
			pGPIO_Handler->pGPIO->AFRL |= temp;
			temp = 0;
		}
		else if(pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number > 7)
		{
			temp = (pGPIO_Handler->GPIO_Pin_Config.GPIO_AlternateMode << (4*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));
			pGPIO_Handler->pGPIO->AFRH &= ~(15ul << (4*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));//clear the requested port bits
			pGPIO_Handler->pGPIO->AFRH |= temp;
			temp = 0;
		}
	}
}

/* GPIO Deinitialization */
void GPIO_DeInitialize(GPIO_RegDef_t* pGPIO)
{
	if(pGPIO == GPIOA){
		GPIOA_RESET();
	}
	else if(pGPIO == GPIOB)
	{
		GPIOB_RESET();
	}
	else if(pGPIO == GPIOC)
	{
		GPIOC_RESET();
	}
	else if(pGPIO == GPIOD)
	{
		GPIOD_RESET();
	}
	else if(pGPIO == GPIOE)
	{
		GPIOE_RESET();
	}
	else if(pGPIO == GPIOF)
	{
		GPIOF_RESET();
	}
	else if(pGPIO == GPIOG)
	{
		GPIOG_RESET();
	}
	else if(pGPIO == GPIOH)
	{
		GPIOH_RESET();
	}
}

/* GPIO Read Input PIN */
uint8_t GPIO_ReadInputPIN(GPIO_RegDef_t* pGPIO, uint8_t PinNumber)
{
	return (uint8_t)((pGPIO->IDR >> PinNumber) & 0x00000001);
}

/* GPIO Read Input Port */
uint16_t GPIO_ReadInputPORT(GPIO_RegDef_t* pGPIO)
{
	return (uint16_t)(pGPIO->IDR);
}

/* GPIO Write Output PIN */
void GPIO_WriteOutputPIN(GPIO_RegDef_t* pGPIO, uint8_t PinNumber, uint8_t value)
{
	if(value == PIN_SET_ENABLE)
	{
		pGPIO->BSRR |= (1 << PinNumber);
	}
	else if(value == PIN_SET_DISABLE)
	{
		pGPIO->BSRR |= (1 << (PinNumber + 16));
	}
}

/* GPIO Write Output Port */
void GPIO_WriteOutputPORT(GPIO_RegDef_t* pGPIO, uint16_t value)
{
	pGPIO->BSRR |= value; //set the requested BS bits to 1
}

/* GPIO Toogle Output PIN */
void GPIO_ToggleOutputPIN(GPIO_RegDef_t* pGPIO, uint8_t PinNumber)
{
	uint32_t temp;
	temp = (pGPIO->ODR >> PinNumber) & 0x00000001;
	if(temp == 0x00000001)
	{
		pGPIO->BSRR |= (0x00000001 << (PinNumber + 16));
	}
	else if(temp == 0x00000000)
	{
		pGPIO->BSRR |= (0x00000001 << PinNumber);
	}
}
