/*
 * stm32f446xx_gpio_driver_course.c
 *
 *  Created on: Apr 8, 2020
 *      Author: quentin
 */
#ifdef QUENTIN


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
	uint32_t temp = 0;
	//Configure the mode
	//First, check if interrupt mode selected, if not do this
	if(pGPIO_Handler->GPIO_Pin_Config.GPIO_Mode <= GPIO_MODE_ANALOG)
	{
		pGPIO_Handler->pGPIO->MODER |= ((pGPIO_Handler->GPIO_Pin_Config.GPIO_Mode) << (2*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));
		pGPIOHandle->pGPIOX->MODER |= temp; //Pourquoi OR 0?
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
			temp = (pGPIO_Handler->GPIO_Pin_Config.GPIO_ << (4*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));
			pGPIO_Handler->pGPIO->AFRL &= ~(15ul << (4*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));//clear the requested port bits
			pGPIO_Handler->pGPIO->AFRL |= temp;
			temp = 0;
		}
		else
		{
			temp = (pGPIO_Handler->GPIO_Pin_Config.GPIO_Speed << (4*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));
			pGPIO_Handler->pGPIO->AFRH &= ~(15ul << (4*pGPIO_Handler->GPIO_Pin_Config.GPIO_Pin_Number));//clear the requested port bits
			pGPIO_Handler->pGPIO->AFRH |= temp;
			temp = 0;
		}
	}
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

#endif /*QUENTIN*/
