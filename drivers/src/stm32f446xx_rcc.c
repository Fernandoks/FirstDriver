/*
 * stm32f446xx_rcc.c
 *
 *  Created on: Apr 27, 2020
 *      Author: fernandoks
 */





/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Mar 29, 2019
 *      Author: admin
 */


#include "stm32f446xx_rcc.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};


#define EXTERNALCRISTAL 	8000000 //inform your external cristal value

/*********************************************************************
 * @fn      		  - RCC_GetPCLK1
 *
 * @brief             This function return the PCLK1, clock for the APB1
 * 					  peripherals
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            pclk1
 *
 * @Note              -
 */
typedef enum
{
	HSI = 0,
	HSE = 1,
	PLL = 2
} clocksource_t;

uint32_t RCC_GetPCLK1(void)
{
	uint32_t pclk1 = 0;
	uint32_t ahbp = 0;
	uint32_t apb1p = 0;
	/*
	 * SYSTEM CLOCK
	 */
	clocksource_t clksource = ( (RCC->CFGR >> 2) & 0x3); //get RCC SW

	if (clksource == HSI)
	{
		pclk1 = 16000000;
	}
	else if (clksource == HSE)
	{
		pclk1 = EXTERNALCRISTAL;
	}
	else
	{
		pclk1 = RCC_GetPLLOutputClock();
	}

	/*
	 * AHB Prescaller
	 */
	uint32_t temp = (( RCC->CFGR >> 4) & 0xF);
	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	/*
	 * APB1 Prescaller
	 */
	temp = (( RCC->CFGR >> 10) & 0x7);
	if (temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}


	pclk1 =  (pclk1 / ahbp) /apb1p;

	return pclk1;
}



/*********************************************************************
 * @fn      		  - RCC_GetPCLK2
 *
 * @brief             This function return the PCLK2, clock for the APB2
 * 					  peripherals
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            pclk2
 *
 * @Note              -

 */
uint32_t RCC_GetPCLK2(void)
{
	uint32_t pclk2 = 0;
	uint32_t ahbp = 0;
	uint32_t apb2p = 0;
	/*
	 * SYSTEM CLOCK
	 */
	clocksource_t clksource = ( (RCC->CFGR >> 2) & 0x3); //get RCC SW

	if (clksource == HSI)
	{
		pclk2 = 16000000;
	}
	else if (clksource == HSE)
	{
		pclk2 = EXTERNALCRISTAL;
	}
	else
	{
		pclk2 = RCC_GetPLLOutputClock();
	}

	/*
	 * AHB Prescaller
	 */
	uint32_t temp = (( RCC->CFGR >> 4) & 0xF);
	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	/*
	 * APB2 Prescaller
	 */
	temp = (( RCC->CFGR >> 13) & 0x7);
	if (temp < 4)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB1_PreScaler[temp-4];
	}


	pclk2 =  (pclk2 / ahbp) /apb2p;

	return pclk2;
}



uint32_t  RCC_GetPLLOutputClock()
{

	//TODO: Implement
	return 0;
}
