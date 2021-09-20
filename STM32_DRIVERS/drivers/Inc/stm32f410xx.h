/*
 * stm32f410xx.h
 *
 *  Created on: Aug 23, 2021
 *      Author: gokul
 */

#ifndef STM32F410XX_H_
#define STM32F410XX_H_
#include <stdint.h>
#define __vo volatile


/* 				**********Processor specific details********						*/

// NVIC interrupt set enable register

#define	NVIC_ISER0 				((__vo uint32_t*)0xE000E100)
#define	NVIC_ISER1 				((__vo uint32_t*)0xE000E104)
#define	NVIC_ISER2 				((__vo uint32_t*)0xE000E108)
#define	NVIC_ISER3 				((__vo uint32_t*)0xE000E10C)

// NVIC interrupt clear enable register
#define	NVIC_ICER0 				((__vo uint32_t*)0xE000E180)
#define	NVIC_ICER1 				((__vo uint32_t*)0xE000E184)
#define	NVIC_ICER2 				((__vo uint32_t*)0xE000E188)
#define	NVIC_ICER3 				((__vo uint32_t*)0xE000E18C)

// NVIC Priority register base address

#define NVIC_PR_BASEADDR		((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED        4


/*							********************************* 						*/

// Base address of flash and sram memory

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR  			0x20000000U
#define ROM							0x1FFF0000U
#define SRAM 						SRAM1_BASEADDR

#define PERIPH_BASE 					APB1PERIPH_BASEADDR
#define APB1PERIPH_BASEADDR 			0x40000000U
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U

//Base address of peripherals hanging on AHB1 bus
#define GPIOA_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOH_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x3800)

//Base address of peripherals hanging on APB1 bus
#define I2C1_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C4_BASEADDR 					(APB1PERIPH_BASEADDR + 0x6000)
#define SPI2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x3800)
#define USART2_BASEADDR 				(APB1PERIPH_BASEADDR + 0x4400)

//Base address of peripherals hanging on APB2 bus
#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)
#define SPI5_BASEADDR					(APB2PERIPH_BASEADDR + 0x5000)
#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)

typedef struct
{
	__vo uint32_t MODER;				// offset 0x00
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_RegDef_t;



typedef struct
{
	__vo uint32_t IMR ;
	__vo uint32_t EMR ;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;


}EXTI_RegDef_t;


#define EXTI 	((EXTI_RegDef_t*)(EXTI_BASEADDR))


typedef  struct
{
	__vo uint32_t MEMRMP ;
	__vo uint32_t PMC ;
	__vo uint32_t EXTICR[4] ;
	__vo uint32_t RESERVED1 ;
	__vo uint32_t CFGR2 ;
	__vo uint32_t CMPCR ;
	__vo uint32_t CFGR ;




}SYSCFG_RegDef_t;

#define SYSCFG 	((SYSCFG_RegDef_t*)(SYSCFG_BASEADDR))

#define GPIOA   (GPIO_RegDef_t*) GPIOA_BASEADDR
#define GPIOB   (GPIO_RegDef_t*) GPIOB_BASEADDR
#define GPIOC   (GPIO_RegDef_t*) GPIOC_BASEADDR
#define GPIOH   (GPIO_RegDef_t*) GPIOH_BASEADDR



typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t RESERVED0[3];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t RESERVED2[3];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t RESERVED4[3];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t RESERVED7[2];
	__vo uint32_t DCKCFGR;
	__vo uint32_t RESERVED8;
	__vo uint32_t DCKCFGR2;

}RCC_RegDef_t;

#define RCC 		((RCC_RegDef_t*)(RCC_BASEADDR))





// SPI RegDef
typedef struct
{
	__vo uint32_t  CR1;				// OFFSET 0X00
	__vo uint32_t  CR2;
	__vo uint32_t  SR;
	__vo uint32_t  DR;
	__vo uint32_t  CRCPR;
	__vo uint32_t  RXCRCR;
	__vo uint32_t  TXCRCR;
	__vo uint32_t  I2SCFGR;
	__vo uint32_t  I2SCR;
	__vo uint32_t  PR;
}SPI_RegDef_t;



#define SPI1        				((SPI_RegDef_t*)(SPI1_BASEADDR))
#define SPI2        				((SPI_RegDef_t*)(SPI2_BASEADDR))
#define SPI5        				((SPI_RegDef_t*)(SPI5_BASEADDR))


//clock enable macros for GPIOx peripherals

#define GPIOA_PCLK_EN()  			(RCC -> AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()  			(RCC -> AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()  			(RCC -> AHB1ENR |= (1<<2))
#define GPIOH_PCLK_EN()  			(RCC -> AHB1ENR |= (1<<7))

//clock enable macros for I2C peripherals

#define I2C1_PCLK_EN()				(RCC -> APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()				(RCC -> APB1ENR |= (1<<22))
#define I2C4_PCLK_EN()				(RCC -> APB1ENR |= (1<<24))


// clock enable macros for SPI peripherals

#define SPI1_PCLK_EN()				(RCC -> APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC -> APB1ENR |= (1<<14))
#define SPI5_PCLK_EN()				(RCC -> APB2ENR |= (1<<20))

//clock enable macros for USART peripherals

#define USART1_PCLK_EN()			(RCC -> APB2ENR |= (1<<4))
#define USART2_PCLK_EN()			(RCC -> APB1ENR |= (1<<17))
#define USART6_PCLK_EN()			(RCC -> APB2ENR |= (1<<5))

//clock enable macros for SYSCFG peripheral

#define SYSCFG_PCLK_EN()			(RCC -> APB2ENR |= (1<<14))

//clock disable macros for GPIO peripherals

#define GPIOA_PCLK_DI()  			(RCC -> AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()  			(RCC -> AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()  			(RCC -> AHB1ENR &= ~(1<<2))
#define GPIOH_PCLK_DI()  			(RCC -> AHB1ENR &= ~(1<<7))

// clock disable macros for I2C peripherals

#define I2C1_PCLK_DI()				(RCC -> APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()				(RCC -> APB1ENR &= ~(1<<22))
#define I2C4_PCLK_DI()				(RCC -> APB1ENR &= ~(1<<24))

//clock disable macros for SPI peripherals

#define SPI1_PCLK_DI()				(RCC -> APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()				(RCC -> APB1ENR &= ~(1<<14))
#define SPI5_PCLK_DI()				(RCC -> APB2ENR &= ~(1<<20))

// clock disable macros for USART peripherals

#define USART1_PCLK_DI()			(RCC -> APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()			(RCC -> APB1ENR &= ~(1<<17))
#define USART6_PCLK_DI()			(RCC -> APB2ENR &= ~(1<<5))

//clock disable macros for SYSCFG peripherals

#define SYSCFG_PCLK_DI())			(RCC -> APB2ENR &= ~(1<<14))


// GPIO Register reset macros

#define GPIOA_REG_RESET() 			do{(RCC -> AHB1RSTR |= (1<<0)) ; (RCC -> AHB1RSTR &= ~(1<<0));}	while(0)
#define GPIOB_REG_RESET() 			do{(RCC -> AHB1RSTR |= (1<<1)) ; (RCC -> AHB1RSTR &= ~(1<<1));}	while(0)
#define GPIOC_REG_RESET() 			do{(RCC -> AHB1RSTR |= (1<<2)) ; (RCC -> AHB1RSTR &= ~(1<<2));}	while(0)
#define GPIOH_REG_RESET() 			do{(RCC -> AHB1RSTR |= (1<<7)) ; (RCC -> AHB1RSTR &= ~(1<<7));}	while(0)



// returns port code for given GPIOx Base address


#define GPIO_BASEADDR_TO_CODE(x)  	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOH) ? 3 : 0)

// some useful macros
#define ENABLE  					1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				1
#define GPIO_PIN_RESET				0

// IRQ numbers of EXTI

#define IRQ_NO_EXTI0 				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3 				9
#define IRQ_NO_EXTI4 				10
#define IRQ_NO_EXTI9_5 				23
#define IRQ_NO_EXTI15_10 			40




// Bit definition macros for SPIx peripherals
// 1. SPI_CR1 REGISTER
#define SPI_CR1_CPHA 				0
#define SPI_CR1_CPOL 				1
#define SPI_CR1_MSTR 				2
#define SPI_CR1_BR 				 	3
#define SPI_CR1_SPE 				6
#define SPI_CR1_LSBFIRST 			7
#define SPI_CR1_SSI 				8
#define SPI_CR1_SSM 				9
#define SPI_CR1_RXONLY 				10
#define SPI_CR1_DFF 				11
#define SPI_CR1_CRCNEXT 		    12
#define SPI_CR1_CRCEN 				13
#define SPI_CR1_BIDIOE 				14
#define SPI_CR1_BIDIMODE 			15

// 2. SPI_CR2 REGISTER
#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

// 3. SPI_SR REGISTER
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

// FLAG conditions
#define FLAG_SET					1
#define FLAG_RESET					0

#endif /* STM32F410XX_H_ */
