/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 27 de jan de 2020
 *      Author: Fernando
 */


#include "stm32f446xx_spi_driver.h"

/*
 * Peripheral clock
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnableDisable)
{
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


/*
 * Init and De-Init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
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
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_SIMPLEX_READONLY)
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



void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
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

/*
 * FLAG Status
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{

	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Lenght)
{
	while (Lenght > 0)
	{
		//while( !( pSPIx->SR & (1 << SPI_SR_TXE) ) )  //This tests if the bit position is set
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );


		if ( (pSPIx->CR1 << SPI_CR1_DFF) == 1)
		{
			//16bits
			pSPIx->DR = pTXBuffer;
		}
		else
		{
			//8bits
			pSPIx->DR = pTXBuffer;
		}
		Length--;
	}

}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Lenght)
{


}

/*
 * IRQ Configuration
 */

void SPI_IRQInterruptConfig(IRQn_Type IRQNumber, uint8_t EnableDisable)
{

}


void SPI_IRQPriorityConfig(IRQn_Type IRQNumber, uint8_t IRQPriority)
{


}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{


}


