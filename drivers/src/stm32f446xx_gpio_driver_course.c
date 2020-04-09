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
		switch(pGPIO)
		{
		case GPIOA:
			GPIOA_PCLK_EN();
			break;
		case GPIOB:
			GPIOB_PCLK_EN();
			break;
		case GPIOC:
			GPIOC_PCLK_EN();
			break;
		case GPIOD:
			GPIOD_PCLK_EN();
			break;
		case GPIOE:
			GPIOE_PCLK_EN();
			break;
		case GPIOF:
			GPIOF_PCLK_EN();
			break;
		case GPIOG:
			GPIOG_PCLK_EN();
			break;
		case GPIOH:
			GPIOH_PCLK_EN();
			break;
		default:
			break;
		}
	}
	else if(status == DISABLE)
	{
		switch(pGPIO)
		{
		case GPIOA:
			GPIOA_PCLK_EN();
			break;
		case GPIOB:
			GPIOB_PCLK_EN();
			break;
		case GPIOC:
			GPIOC_PCLK_EN();
			break;
		case GPIOD:
			GPIOD_PCLK_EN();
			break;
		case GPIOE:
			GPIOE_PCLK_EN();
			break;
		case GPIOF:
			GPIOF_PCLK_EN();
			break;
		case GPIOG:
			GPIOG_PCLK_EN();
			break;
		case GPIOH:
			GPIOH_PCLK_EN();
			break;
		default:
			break;
		}
	}
}

/* GPIO Initialization */
void GPIO_Initialize(GPIO_Handler_TypedDef* pGPIO_Handler)
{

}

/* GPIO Deinitialization */
void GPIO_DeInitialize(GPIO_RegDef_t* pGPIO)
{
	switch(pGPIO)
	{
	case GPIOA:
		GPIOA_RESET();
		break;
	case GPIOB:
		GPIOB_RESET();
		break;
	case GPIOC:
		GPIOC_RESET();
		break;
	case GPIOD:
		GPIOD_RESET();
		break;
	case GPIOE:
		GPIOE_RESET();
		break;
	case GPIOF:
		GPIOF_RESET();
		break;
	case GPIOG:
		GPIOG_RESET();
		break;
	case GPIOH:
		GPIOH_RESET();
		break;
	default:
		break;
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
	if(value == GPIO_PIN_ENABLE)
	{
		pGPIO->ODR |= (1 << PinNumber);
	}
	else if(value == GPIO_PIN_DISABLE)
	{
		pGPIO->ODR &= ~(1 << PinNumber);
	}
}

/* GPIO Write Output Port */
void GPIO_WriteOutputPORT(GPIO_RegDef_t* pGPIO, uint16_t value)
{
	pGPIO->ODR = value;
}

/* GPIO Toogle Output PIN */
void GPIO_ToggleOutputPIN(GPIO_RegDef_t* pGPIO, uint8_t PinNumber)
{
	pGPIO->ODR ^= (0x00000001 << PinNumber);
}
