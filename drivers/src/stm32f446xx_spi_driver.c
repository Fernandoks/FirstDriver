/*******************************************************************************
* Title                 :   STM32F446 SPI Driver
* Filename              :   stm32f446xx_spi_driver.c
* Author                :   Fernando Kaba Surjus
* Origin Date           :   24/03/2020
* Version               :   1.0.0
* Compiler              :   TODO: COMPILER GOES HERE
* Target                :   TODO: MCU GOES HERE
* Notes                 :   None
*/
/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author         Description
*  24/03/20   1.0.0   Fernando KS  Initial Release.
*
*******************************************************************************/
/** @file : Mstm32f446xx_spi_driver.c
 *  @brief This is the source file for TODO: WHAT DO I DO?
 */

/******************************************************************************
* Includes
*******************************************************************************/
#include "stm32f446xx_spi_driver.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/



/******************************************************************************
* Function Definitions
*******************************************************************************/
/*
 * Interfaces
 * Keep definition in C, and Static to make private functions
 */
static void SPI_TXE_IRQ(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_IRQ(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_ERROR(SPI_Handle_t *pSPIHandle);

/******************************************************************************
* Function : SPI_PeriClockControl()
*//**
* \b Description:
*
* This function is used to initialize the Dio based on the configuration table
*  defined in dio_cfg module.
*
* PRE-CONDITION: call with a valid SPI_RegDef_t
*
* POST-CONDITION: TBD
*
* @return 		VOID
*
* \b Example Example:
* @code
*			SPI_PeriClockControl(SPI1, Enable)
* @endcode
*
* @see SPI_PeriClockControl

*******************************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnableDisable)
{
	assert(IS_SPI(pSPIx));


	if (EnableDisable == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else if (EnableDisable == DISABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}


/******************************************************************************
* Function : SPI_Init()
*//**
* \b Description:
*
* This function is used to initialize the SPI
*
* PRE-CONDITION: Call with a valid SPI_Handle_t, correctly filled.
*
* POST-CONDITION: TBD
*
* @return 		VOID
*
*
* @see SPI_Init

*******************************************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	assert(IS_SPI(pSPIHandle->pSPIx));
	assert(IS_SPI_MODE(pSPIHandle->SPIConfig.SPI_DeviceMode));
	assert(IS_SPI_BUSCONFIG(pSPIHandle->SPIConfig.SPI_BusConfig));
	assert(IS_SPI_SCLKSPEED(pSPIHandle->SPIConfig.SPI_SPI_SclkSpeed));
	assert(IS_SPI_DFF(pSPIHandle->SPIConfig.SPI_DFF));
	assert(IS_SPI_CPOL(pSPIHandle->SPIConfig.SPI_CPOL));
	assert(IS_SPI_CPHA(pSPIHandle->SPIConfig.SPI_CPHA));
	assert(IS_SPI_SSM(pSPIHandle->SPIConfig.SPI_SSM));


	//Clock Enable
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	uint32_t tempreg = 0;

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the BUS Config

	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_FD)
	{
		//BIDI clear
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_HD)
	{
		//BIDI SET
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_SIMPLEX_RXONLY)
	{
		//BIDI clear and RXONLY SET
		tempreg |= (1 << SPI_CR1_RXONLY);
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}

	//3. SPI_SPI_SclkSpeed;
	tempreg |= (pSPIHandle->SPIConfig.SPI_SPI_SclkSpeed << SPI_CR1_BR);


	//4. SPI_DFF;
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. SPI_CPOL;
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. SPI_CPHA;
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. SPI_SSM;
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	//Push to register
	pSPIHandle->pSPIx->CR1 = tempreg;


}

/******************************************************************************
* Function : SPI_DeInit()
*//**
* \b Description:
*
* This function is used to initialize the SPI
*
* PRE-CONDITION: Call with a valid SPI_RegDef_t
*
* POST-CONDITION: TBD
*
* @return 		VOID
*
* \b Example Example:
* @code
*
* @endcode
*
* @see SPI_Init

*******************************************************************************/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	assert(IS_SPI(pSPIx));

	if (pSPIx == SPI1)
	{
		SPI1_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_RESET();
	}
}


/******************************************************************************
* Function : SPI_PeripheralControl()
*//**
* \b Description:
*
* This function is used to Enable or Disable the SPI Peripheral
*
* PRE-CONDITION: Call with a valid SPI_Handle_t, correctly filled.
*
* POST-CONDITION: TBD
*
* @return 		VOID
*
*
* @see SPI_PeripheralControl

*******************************************************************************/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIRegDef, uint8_t EnableDisable)
{
	assert(IS_SPI(pSPIRegDef));

	if (EnableDisable == ENABLE)
	{
		pSPIRegDef->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIRegDef->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

/******************************************************************************
* Function : SPI_Config_SSI()
*//**
* \b Description:
*
* Enable/Disable the SSI register
* SSI (Internal slave select), used to force the SSI bit
*
* PRE-CONDITION: Call with a valid SPI_Handle_t, correctly filled.
*
* POST-CONDITION: TBD
*
* @return 		VOID
*
*
* @see SPI_Config_SSI

*******************************************************************************/
void SPI_Config_SSI(SPI_RegDef_t *pSPIRegDef, uint8_t EnableDisable)
{
	assert(IS_SPI(pSPIRegDef));

	if (EnableDisable == ENABLE)
	{
		pSPIRegDef->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIRegDef->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}

/******************************************************************************
* Function : SPI_Config_SSOE()
*//**
* \b Description:
*
* SS output enable
* 0: SS output is disabled in master mode and the cell can work in multi master configuration
* 1: SS output is enabled in master mode and when the cell is enabled. The cell cannot work
* in a multi master environment.
*
* PRE-CONDITION: Call with a valid SPI_Handle_t, correctly filled.
*
* POST-CONDITION: TBD
*
* @return 		VOID
*
*
* @see SPI_Config_SSOE

*******************************************************************************/
void SPI_Config_SSOE(SPI_RegDef_t *pSPIRegDef, uint8_t EnableDisable)
{
	assert(IS_SPI(pSPIRegDef));

	if (EnableDisable == ENABLE)
	{
		pSPIRegDef->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIRegDef->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}



/*
 * FLAG Status
 */
FLAG_Status_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	assert(IS_SPI(pSPIx));
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data send and Receive - Blocking mode
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Lenght)
{

	assert(IS_SPI(pSPIx));
	assert(pTXBuffer != NULL);
	assert(Lenght > 0);

	while (Lenght > 0)
	{
		//while( !( pSPIx->SR & (1 << SPI_SR_TXE) ) )  //This tests if the bit position is set
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		//Verify the Frame format (8bits or 16 bits) using the CR1_DFF
		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) == 1)
		{
			//16bits
			pSPIx->DR = *((uint16_t*)pTXBuffer);
			Lenght--;
			(uint16_t*)pTXBuffer++;

		}
		else
		{
			//8bits
			pSPIx->DR = *(pTXBuffer);
			pTXBuffer++;
		}
		Lenght--;
	}

}

/*
 * Receive data from SPI- Blocking mode
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Lenght)
{
	assert(IS_SPI(pSPIx));
	assert(pRXBuffer != NULL);
	assert(Lenght > 0);

	while (Lenght > 0)
	{
		//while( !( pSPIx->SR & (1 << SPI_SR_TXE) ) )  //This tests if the bit position is set
		while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		//Verify the Frame format (8bits or 16 bits) using the CR1_DFF
		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) == 1)
		{
			//16bits
			*((uint16_t*)pRXBuffer) = pSPIx->DR;
			Lenght--;
			(uint16_t*)pRXBuffer++;

		}
		else
		{
			//8bits
			*((uint16_t*)pRXBuffer) = pSPIx->DR;
			pRXBuffer++;
		}
		Lenght--;
	}

}

/*
 * Send data from SPI - Interrupt Mode
 * This API get the pointer, save the data in the buffer
 * then put the SPI State to Busy, and enable TXEIE
 *
 */
SPI_States_t SPI_SendData_IT(SPI_Handle_t *pSPIx, uint8_t *pTXBuffer, uint32_t Lenght)
{

	if (pSPIx->TxState != SPI_BUSY_TX)
	{
		pSPIx->pTxBuffer = pTXBuffer;
		pSPIx->TxLen = Lenght;

		pSPIx->TxState = SPI_BUSY_TX;

		pSPIx->pSPIx->CR2 |= (1ul << SPI_CR2_TXEIE);
	}

	return (pSPIx->TxState);
}

/*
 * Reveive data from SPI - Interrupt Mode
 * This API get the pointer, save the data in the buffer
 * then put the SPI State to Busy, and enable TXEIE
 *
 */

SPI_States_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIx, uint8_t *pRXBuffer, uint32_t Lenght)
{

	if (pSPIx->TxState != SPI_BUSY_RX)
	{
		pSPIx->pRxBuffer = pRXBuffer;
		pSPIx->RxLen = Lenght;

		pSPIx->RxState = SPI_BUSY_RX;

		pSPIx->pSPIx->CR2 |= (1ul << SPI_CR2_RXNEIE);
	}

	return (pSPIx->TxState);
}

/*
 * IRQ Configuration
 * TODO: All NVIC configurations must be in a separated driver
 */

void SPI_IRQInterruptConfig(IRQn_Type IRQNumber, uint8_t EnableDisable)
{
	if(EnableDisable == ENABLE)
	{
		if(IRQNumber < 32)
		{
		*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if((IRQNumber >= 32) && (IRQNumber < 64))
		{
			*NVIC_ISER1 |= (1 << IRQNumber);

		}
		else if((IRQNumber >= 64) && (IRQNumber < 96))
		{
			*NVIC_ISER2 |= (1 << IRQNumber);
		}
		else if((IRQNumber >= 96) && (IRQNumber < 128))
		{
			*NVIC_ISER3 |= (1 << IRQNumber);
		}
	}
	else if(EnableDisable == DISABLE)
	{
		if(IRQNumber < 32)
		{
		*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if((IRQNumber >= 32) && (IRQNumber < 64))
		{
			*NVIC_ICER1 |= (1 << IRQNumber);

		}
		else if((IRQNumber >= 64) && (IRQNumber < 96))
		{
			*NVIC_ICER2 |= (1 << IRQNumber);
		}
		else if((IRQNumber >= 96) && (IRQNumber < 128))
		{
			*NVIC_ICER3 |= (1 << IRQNumber);
		}
	}

}

/*
 * SPI Priority config
 * TODO: Create a generic API for NVIC
 */

void SPI_IRQPriorityConfig(IRQn_Type IRQNumber, uint8_t IRQPriority)
{

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_IRQ = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDRESS + (iprx * 4)) = (IRQPriority << shift_IRQ);
}


/*
 * SPI IRQ Handling
 * You must REMAP or call this function in the SPI IRQ
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

	/*
	 * Event Flags
	 * TXE, RXNE, MODF, OVR, CRCERR, FRE
	 */
	uint32_t SRReg;
	uint32_t CRReg;

	SRReg = pSPIHandle->pSPIx->SR;
	CRReg = pSPIHandle->pSPIx->CR2;

	//TXE - Transmission buffer empty
	if ( (SRReg & (1ul << SPI_SR_TXE)) && (CRReg & (1ul << SPI_CR2_TXEIE) ) )
	{
		SPI_TXE_IRQ(pSPIHandle);
	}

	//RXNE - Reception buffer not empty
	if ( (SRReg & (1ul << SPI_SR_RXNE)) && (CRReg & (1ul << SPI_CR2_RXNEIE) ) )
	{
		SPI_RXNE_IRQ(pSPIHandle);
	}

	/*
	 * ERROR INTERRUPTS
	 */
	//OVR - Overrun error (lost data due not reading)
	if ( (SRReg & (1ul << SPI_SR_OVR)) && (CRReg & (1ul << SPI_CR2_ERRIE) ) )
	{
		SPI_OVR_ERROR(pSPIHandle);

	}

	//CRCERR - CRC error
	if ( (SRReg & (1ul << SPI_SR_CRCERR)) && (CRReg & (1ul << SPI_CR2_ERRIE) ) )
	{


	}

	//FREE - Frame format error
	if ( (SRReg & (1ul << SPI_SR_FRE)) && (CRReg & (1ul << SPI_CR2_ERRIE) ) )
	{


	}
	//MODF
	if ( (SRReg & (1ul << SPI_SR_MODF)) && (CRReg & (1ul << SPI_CR2_ERRIE) ) )
	{

	}


}

static void SPI_TXE_IRQ(SPI_Handle_t *pSPIHandle)
{
	if (!pSPIHandle->TxLen)
	{
		//Clear flag and close transmission
		SPI_CloseTransmission(pSPIHandle);
		//Call the Callback event handler
		SPI_IRQEvents_t event = SPI_EVENT_TC;
		SPI_ApplicationEventCallback(pSPIHandle, event);

	}
	//Verify the Frame format (8bits or 16 bits) using the CR1_DFF
	if ( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) == 1)
	{
		//16bits
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;

	}
	else
	{
		//8bits
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer++;
	}

}

static void SPI_RXNE_IRQ(SPI_Handle_t *pSPIHandle)
{
	if (!pSPIHandle->TxLen)
	{
		//Clear flag and close Reception
		SPI_CloseReception(pSPIHandle);
		//Call the Callback event handler
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RXE);

	}
	//Verify the Frame format (8bits or 16 bits) using the CR1_DFF
	if ( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) == 1)
	{
		//16bits
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pRxBuffer);
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;

	}
	else
	{
		//8bits
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pRxBuffer);
		pSPIHandle->pRxBuffer++;
	}
	pSPIHandle->RxLen--;
}

static void SPI_OVR_ERROR(SPI_Handle_t *pSPIHandle)
{

	//Clear the flag if not reading already
	if (pSPIHandle->TxState != SPI_BUSY_TX)
	{
		SPI_Clear_OVER(pSPIHandle->pSPIx);
	}

	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR);
}

/*
 * Close fucntions
 *
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//Clear interrupt TXEIE flag
	pSPIHandle->pSPIx->CR2 &= (1ul < SPI_CR2_TXEIE);
	//Reset buffers
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//Clear interrupt RXNEIE flag
	pSPIHandle->pSPIx->CR2 &= (1ul < SPI_CR2_RXNEIE);
	//Reset buffers
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

/*
 * Clear OVR FLAG
 */
void SPI_Clear_OVER(SPI_RegDef_t *pSPIRegDef)
{
	uint8_t temp = 0;
	//Dummy read to clear OVR Flag
	temp = pSPIRegDef->DR;
	temp = pSPIRegDef->SR;
	(void)temp;
}

/*
 * Generic CALLBACK
 */

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, SPI_IRQEvents_t event)
{
	/*
	 * This is weak implemented, do not TYPE CODE HERE!
	 * Create a new function in your application with the same name
	 * use the Handle to identify the SPI, and get the buffer pointer.
	 * SPI_IRQEvents_t event gives you the Event flag
	 */

	__NOP();

}



