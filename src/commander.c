/*
 * commander.c
 *
 *  Created on: Apr 14, 2020
 *      Author: fernandoks
 */

#include <stdint.h>
#include "commander.h"

uint8_t board_id[20] = "Nucleo STM32F446RE";

#define NACK 0xA5
#define ACK 0xF5


//command codes
#if 0
#define COMMAND_LED_CTRL          	0x50
#define COMMAND_SENSOR_READ       	0x51
#define COMMAND_LED_READ          	0x52
#define COMMAND_PRINT           	0x53
#define COMMAND_ID_READ				0x54
#endif


CMDHeader_type CMD[] =
{
		{COMMAND_LED_CTRL, CMD_Led_Ctrl},
		{COMMAND_SENSOR_READ, CMD_Sensor_Read},
		{COMMAND_LED_READ, CMD_Led_Read},
		{COMMAND_PRINT, CMD_Printl},
		{COMMAND_ID_READ, CMD_ID_Read},
};

command_type CMD_State = 0;


void CMD_Led_Ctrl(void)
{
	CMD[CMD_State].command_func();
}


void CMD_Sensor_Read(void)
{

}


void CMD_Led_Read(void)
{

}


void CMD_Printl(void)
{

}


void CMD_ID_Read(void)
{

}







