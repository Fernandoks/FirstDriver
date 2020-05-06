/*
 * main.h
 *
 *  Created on: May 4, 2020
 *      Author: fernandoks
 */

#ifndef MAIN_H_
#define MAIN_H_


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


#endif /* MAIN_H_ */
