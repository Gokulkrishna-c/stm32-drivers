/*
 * stm32f410xx_gpio_driver.c
 *
 *  Created on: 24-Aug-2021
 *      Author: gokul
 */


#include "stm32f410xx_gpio_driver.h"


/*
 * @fn         ---	GPIO_PeriClockControl
 * @brief      ---	This func. enables or disables the peripheral clock for a given GPIO port
 *
 * @para[1]    ---	Base addr of GPIO peripheral
 * @para[2]    ---	Enable or Disable macros
 * @para[3]    ---
 *
 * @return     ---	none
 * @note       ---	none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}

	}

}


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
uint32_t temp = 0 ;

// 1. Configure mode of GPIO pin
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)  //Non interrupt
		{
			temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <<(2* pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle -> pGPIOx -> MODER |= temp;
		}
	else
	{
		// TO be done (interrupt)
		if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Configure the FTSR register and disabling RTSR
			EXTI -> FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI -> RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}
		else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI -> RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI -> FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI -> FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI -> RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

	// Configure IMR register
		EXTI -> IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	// Configure GPIO port selection in SYSCFG_EXTICR
		uint32_t temp1 =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;
		uint32_t temp2 = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();

		SYSCFG -> EXTICR[temp1] = portcode << (temp2*4);




	}




// 2. Configure GPIO pin speed

	temp = 0;
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;

// 3. Configure GPIO PUPD reg

	temp = 0;
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;

// 4. Configure output type

	temp = 0;
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;

// 5. ALT functionality mode

	if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1 , temp2 ;
		temp1 = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber) / 8;  // gives AFR[0] or AFR[1]
		temp2 = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber) % 8;  // gives the pin number which can be multiplied by 4 to get the bit no
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2));
		pGPIOHandle -> pGPIOx -> AFR[temp1] |= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode <<(4*temp2));

	}

}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

			if (pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if (pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}

}

/*
 * @fn         ---	GPIO_ReadFromInputPin
 * @brief      ---	This func. gives the bit stored in IDR
 *
 * @para[1]    ---	Base addr of GPIO peripheral
 * @para[2]    ---	Pin number
 * @para[3]    ---
 *
 * @return     ---	0 or 1
 * @note       ---	none
 */


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber)
{
	uint8_t value ;
	value = (uint8_t)((pGPIOx -> IDR >> PinNumber ) & 0x00000001 );
	return value ;

}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx -> IDR);
	return value;

}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber , uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		(pGPIOx -> ODR |= 1 << PinNumber);
		//write 1 to the ODR at the bit field corresponding to the pin number
	}
	else
	{
		(pGPIOx -> ODR &= ~(1 << PinNumber));
	}

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value)
{
	pGPIOx -> ODR = Value ;

}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
	pGPIOx -> ODR ^= (1 << PinNumber);
	// Toggles
}






/*
 * @fn         ---	GPIO_IRQConfig
 * @brief      ---
 *
 * @para[1]    ---
 * @para[2]    ---
 * @para[3]    ---
 *
 * @return     ---
 * @note       ---
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << (IRQNumber));
		}
		else if (IRQNumber >31 && IRQNumber < 64)
		{
			*NVIC_ISER1  |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2  |= (1 << (IRQNumber % 64));

		}
	}
		else
		{
			if (IRQNumber <= 31)
				{
					*NVIC_ICER0  |= (1 << (IRQNumber));
				}
			else if (IRQNumber >31 && IRQNumber < 64)
				{
					*NVIC_ICER1  |= (1 << (IRQNumber % 32));
				}
			else if (IRQNumber >= 64 && IRQNumber < 96)
				{
					*NVIC_ICER2  |= (1 << (IRQNumber % 64));

				}
		}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority )
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shiftamount = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx)	|= (IRQPriority << shiftamount);




}

/*
 * @fn         ---	GPIO_IRQHandling
 * @brief      ---
 *
 * @para[1]    ---
 * @para[2]    ---
 * @para[3]    ---
 *
 * @return     ---
 * @note       ---
  */

void GPIO_IRQHandling(uint8_t PinNumber)
{

	// Clear the EXTI PR register corresponding to the pin number
	if (EXTI -> PR & (1 << PinNumber))
	{
		//Clear
		EXTI -> PR |= (1 << PinNumber);
	}

}













