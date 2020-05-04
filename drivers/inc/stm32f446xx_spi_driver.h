/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 27 de jan de 2020
 *      Author: Fernando
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*********************************************************************************
 * SPI states
 ********************************************************************************/
typedef enum
{
	SPI_READY = 0,
	SPI_BUSY_RX = 1,
	SPI_BUSY_TX = 2
} SPI_States_t;

/*********************************************************************************
 * Configuration Structure for SPI
 ********************************************************************************/
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;


/*********************************************************************************
 * Handle Structure for SPIx
 ********************************************************************************/

typedef struct
{
	SPI_RegDef_t	*pSPIx;
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	SPI_States_t	TxState;
	SPI_States_t	RxState;
} SPI_Handle_t;

/*********************************************************************************
 * IRQ Events
 * Event Flags
 * TC, RXE, MODF, OVR, CRCERR, FRE
 ********************************************************************************/
typedef enum
{
	SPI_EVENT_TC = 0,
	SPI_EVENT_RXE = 1,
	SPI_EVENT_MODF = 2,
	SPI_EVENT_OVR = 3,
	SPI_EVENT_CRCERR = 4,
	SPI_EVENT_FREE = 5
} SPI_IRQEvents_t;


/*********************************************************************************
 * Configuration Structure MACROS for SPIx
 ********************************************************************************/

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 *  @SPI_BusConfig
 */
#define SPI_BUSCONFIG_FD				1
#define SPI_BUSCONFIG_HD				2
#define SPI_BUSCONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SPI_SclkSpeed
 */
#define SPI_SCLKSPEED_DIV2				1
#define SPI_SCLKSPEED_DIV4				2
#define SPI_SCLKSPEED_DIV8				3
#define SPI_SCLKSPEED_DIV16				4
#define SPI_SCLKSPEED_DIV32				5
#define SPI_SCLKSPEED_DIV64				6
#define SPI_SCLKSPEED_DIV128			7
#define SPI_SCLKSPEED_DIV256			8

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

/*
 * @SPI_SSM
 */
#define SPI_SSM_ENABLE					1
#define SPI_SSM_DISABLE					0


#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG 					(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG					(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG					(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG					(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG					(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG					(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG					(1 << SPI_SR_FRE)



/*********************************************************************************
 *
 * APIs Supported by the Driver
 *
 *********************************************************************************/

/*
 * Peripheral clock
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnableDisable); /* First parameter is to select with peripheral (port) so we use base address */

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIRegDef, uint8_t EnableDisable);

/*
 * Data send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Lenght);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Lenght);
SPI_States_t SPI_SendData_IT(SPI_Handle_t *pSPIx, uint8_t *pTXBuffer, uint32_t Lenght);
SPI_States_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIx, uint8_t *pRXBuffer, uint32_t Lenght);
/*
 * Peripheral Status
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*
 * IRQ Configuration
 */
void SPI_IRQInterruptConfig(IRQn_Type IRQNumber, uint8_t EnableDisable);
void SPI_IRQPriorityConfig(IRQn_Type IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
/*
 * General CALLBACK
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, SPI_IRQEvents_t event);

/*
 * Close APIS
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
/*
 * Other Configuration
 */
void SPI_Config_SSI(SPI_RegDef_t *pSPIRegDef, uint8_t EnableDisable);
void SPI_Config_SSOE(SPI_RegDef_t *pSPIRegDef, uint8_t EnableDisable);
void SPI_Clear_OVER(SPI_RegDef_t *pSPIRegDef);



/***********************************************************
 * ASSERT DEFINITIONS
 ***********************************************************/

#define IS_SPI(INSTANCE) (((INSTANCE) == SPI1) 					|| \
                                        ((INSTANCE) == SPI2) 	|| \
                                        ((INSTANCE) == SPI3) 	|| \
                                        ((INSTANCE) == SPI4))



#define IS_SPI_MODE(MODE) (((MODE) == SPI_DEVICE_MODE_MASTER)       		||\
                            ((MODE) == SPI_DEVICE_MODE_SLAVE))


#define IS_SPI_BUSCONFIG(MODE) (((MODE) == SPI_BUSCONFIG_FD)         		||\
                            ((MODE) == SPI_BUSCONFIG_HD)          			||\
							((MODE) == SPI_BUSCONFIG_SIMPLEX_RXONLY))

#define IS_SPI_SCLKSPEED(INSTANCE) (((INSTANCE) == SPI_SCLKSPEED_DIV2) 			|| \
                                        ((INSTANCE) == SPI_SCLKSPEED_DIV4)		|| \
                                        ((INSTANCE) == SPI_SCLKSPEED_DIV8)		|| \
                                        ((INSTANCE) == SPI_SCLKSPEED_DIV16) 	|| \
                                        ((INSTANCE) == SPI_SCLKSPEED_DIV32) 	|| \
                                        ((INSTANCE) == SPI_SCLKSPEED_DIV64) 	|| \
                                        ((INSTANCE) == SPI_SCLKSPEED_DIV128) 	|| \
										((INSTANCE) == SPI_SCLKSPEED_DIV256))

#define IS_SPI_DFF(MODE) (((MODE) == SPI_DEVICE_MODE_MASTER)           ||\
                            ((MODE) == SPI_DEVICE_MODE_SLAVE))

#define IS_SPI_CPOL(MODE) (((MODE) == SPI_CPOL_HIGH)         ||\
                            ((MODE) == SPI_CPOL_LOW))


#define IS_SPI_CPHA(MODE) (((MODE) == SPI_CPHA_HIGH)         ||\
                            ((MODE) == SPI_CPHA_LOW))


#define IS_SPI_SSM(MODE) (((MODE) == SPI_SSM_ENABLE)         ||\
                            ((MODE) == SPI_SSM_DISABLE))



#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
