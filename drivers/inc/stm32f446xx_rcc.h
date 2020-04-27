/*
 * stm32f446xx_rcc.h
 *
 *  Created on: Apr 27, 2020
 *      Author: fernandoks
 */

#ifndef INC_STM32F446XX_RCC_H_
#define INC_STM32F446XX_RCC_H_


#include <stdint.h>



uint32_t RCC_GetPCLK1(void);
uint32_t RCC_GetPCLK2(void);

uint32_t  RCC_GetPLLOutputClock();


#endif /* INC_STM32F446XX_RCC_H_ */
