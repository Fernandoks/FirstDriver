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

uint32_t RCC_GetPCLK1(void)
{
	uint32_t pclk1;

	/*
	 * get system clock
	 * AHB Prescaller
	 * APB1 Prescaller
	 */

	//pclk1 =  (SystemClk / ahbp) /apb1p;
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

	uint32_t pclk2;
	return pclk2;
}



uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}
