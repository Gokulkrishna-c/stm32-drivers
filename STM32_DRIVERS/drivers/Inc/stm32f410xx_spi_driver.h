/*
 * stm32f410xx_spi_driver.h
 *
 *  Created on: 06-Sep-2021
 *      Author: gokul
 */

#ifndef INC_STM32F410XX_SPI_DRIVER_H_
#define INC_STM32F410XX_SPI_DRIVER_H_
#include "stm32f410xx.h"

// Configuration structure for spix peripheral
typedef struct
{
	uint8_t SPI_DeviceMode ;
	uint8_t SPI_BusConfig ;
	uint8_t SPI_SclkSpeed ;
	uint8_t SPI_DFF ;
	uint8_t SPI_CPOL ;
	uint8_t SPI_CPHA ;
	uint8_t SPI_SSM ;

}SPI_Config_t;

// Handle structure for spix peripheral
typedef struct
{
	SPI_RegDef_t  *pSPIx ;   		//  This holds the base address of spix peripheral
	SPI_Config_t  SPIConfig ;

}SPI_Handle_t;



//@SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE 			0


//@SPI_BusConfig
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RX		3


//@SPI_SclkSpeed

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4			    1
#define SPI_SCLK_SPEED_DIV8			    2
#define SPI_SCLK_SPEED_DIV16		    3
#define SPI_SCLK_SPEED_DIV32		    4
#define SPI_SCLK_SPEED_DIV64		    5
#define SPI_SCLK_SPEED_DIV128	        6
#define SPI_SCLK_SPEED_DIV256			7


//@SPI_DFF
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1


//@SPI_CPOL
#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1


//@SPI_CPHA
#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1

//@SPI_SSM
#define SPI_SSM_EN						1
#define SPI_SSM_DI						0



// SPI related flag status definition
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)







// Peripheral clock enable disable

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi);


// Init and Deinit

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


// Data send and receive

void SPI_SendData (SPI_RegDef_t *pSPIx , uint8_t *pTxBuffer , uint32_t Len );
void SPI_ReceiveData (SPI_RegDef_t *pSPIx , uint8_t *pRxBuffer , uint32_t Len );


//IRQ configuration and ISR handling

void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority );
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
// SPI peripheral control (To enable before sending any data through SPI )
// Should be disabled when any changes are made to the control registers
// disabled by default
*/
void SPI_PeripheralControl (SPI_RegDef_t *pSPIx ,uint8_t EnorDi );


// Controls the SSI bit to disable MODF error
void SPI_SSIConfig (SPI_RegDef_t *pSPIx ,uint8_t EnorDi);





#endif /* INC_STM32F410XX_SPI_DRIVER_H_ */
