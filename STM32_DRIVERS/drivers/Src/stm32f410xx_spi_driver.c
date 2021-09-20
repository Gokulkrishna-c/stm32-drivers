/*
 * stm32f410xx_spi_driver.c
 *
 *  Created on: 06-Sep-2021
 *      Author: gokul
 */

#include "stm32f410xx_spi_driver.h"


/*
 * @fn         ---	SPI_PeriClockControl
 * @brief      ---	This func. enables or disables the peripheral clock for a given SPI port
 *
 * @para[1]    ---	Base addr of SPI peripheral
 * @para[2]    ---	Enable or Disable macros
 * @para[3]    ---
 *
 * @return     ---	none
 * @note       ---	none
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			if (pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if (pSPIx == SPI5)
			{
				SPI5_PCLK_EN();
			}

		}
	else
		{
			if (pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if (pSPIx == SPI5)
			{
				SPI5_PCLK_DI();
			}


		}
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//  Configure the SPI_CR1 register
	uint32_t tempreg = 0;

	// 1. Device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// Clear BIDI mode
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// Set BIDI mode
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	{
		// BIDI cleared and RX only mode set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 5. Configure CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;




}


// FLAG status function
uint8_t SPI_GetFLagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if (pSPIx->SR & (FlagName))
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}




// SEND data
void SPI_SendData (SPI_RegDef_t *pSPIx , uint8_t *pTxBuffer , uint32_t Len )
{
	while (Len > 0)
	{
		// 1. wait until TXE is set
		while (SPI_GetFLagStatus(pSPIx , SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the DFF bit in CR1
		if (pSPIx -> CR1 & (1 << SPI_CR1_DFF))
		{
			// 1. 16 Bit
			// Load the data into DR
			pSPIx -> DR = *((uint16_t*)pTxBuffer);
			Len-- ;
			Len-- ;
			(uint16_t*)pTxBuffer ++ ;

		}
		else
		{
			// 2. 8 Bits
			pSPIx -> DR = *pTxBuffer ;
			Len -- ;
			pTxBuffer ++ ;

		}

	}
}


//SPI peripheral control
/*
// SPI peripheral control (To enable before sending any data through SPI )
// Should be disabled when any changes are made to the control registers
// disabled by default
*/
void SPI_PeripheralControl (SPI_RegDef_t *pSPIx ,uint8_t EnorDi )
{
	if (EnorDi == ENABLE)
	 	{
		pSPIx -> CR1 |= (1 << SPI_CR1_SPE);
	 	}
	else
		{
		pSPIx -> CR1 &= ~(1 << SPI_CR1_SPE);
		}
}


// Controls the SSI bit to disable MODF error
/*To make the NSS signal internally high to avoid MODF error */

void SPI_SSIConfig (SPI_RegDef_t *pSPIx ,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			pSPIx -> CR1 |= (1 << SPI_CR1_SSI);
		}
	else
		{
			pSPIx -> CR1 &= ~(1 << SPI_CR1_SSI);
		}
}


/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx -> CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx -> CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}



// RECEIVE data
void SPI_ReceiveData (SPI_RegDef_t *pSPIx , uint8_t *pRxBuffer , uint32_t Len )
{
	while (Len > 0)
		{
			// 1. wait until RXNE is set
			while (SPI_GetFLagStatus(pSPIx , SPI_RXNE_FLAG) == FLAG_RESET);

			// 2. Check the DFF bit in CR1
			if (pSPIx -> CR1 & (1 << SPI_CR1_DFF))
			{
				// 1. 16 Bit
				// Receive data from DR
				*((uint16_t*)pRxBuffer) = pSPIx -> DR;
				Len-- ;
				Len-- ;
				(uint16_t*)pRxBuffer ++ ;

			}
			else
			{
				// 2. 8 Bits
				*pRxBuffer = pSPIx -> DR;
				Len -- ;
				pRxBuffer ++ ;

			}

		}
}

