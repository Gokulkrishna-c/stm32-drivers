/*
 * stm32f410xx_i2c_driver.c
 *
 *  Created on: 28-May-2022
 *      Author: gokul
 */

#include "stm32f410xx_i2c_driver.h"

uint8_t RCC_GetPLLOutputClock(void)
{
	return 0;
}

uint16_t AHB_Prescalar[8] = {2,4,16,64,128,256,512};
uint8_t APB1_Prescalar[4] = {2,4,8,16};
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAdressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAdressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t*pI2Cx);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx ->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAdressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // Make space for the read write bit at LSB
	SlaveAddr &= ~(1); // Makes LSB zero for write operation
	pI2Cx ->DR = SlaveAddr;

}
static void I2C_ExecuteAdressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // Make space for the read write bit at LSB
	SlaveAddr &= (1); // Makes LSB 1 for read operation
	pI2Cx ->DR = SlaveAddr;

}
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	// Check for device mode
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		// Device is in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandle ->RxSize == 1)
			{
				// first disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// Clear the ADDR flag
				dummy_read = pI2CHandle ->pI2Cx ->SR1;
				dummy_read = pI2CHandle ->pI2Cx ->SR2;
				(void)dummy_read;

			}
		}
		else
		{
			// Clear the ADDR flag
			dummy_read = pI2CHandle ->pI2Cx ->SR1;
			dummy_read = pI2CHandle ->pI2Cx ->SR2;
			(void)dummy_read;
		}

	}
	else
	{
		// Device is in slave mode
		// Clear the ADDR flag
		dummy_read = pI2CHandle ->pI2Cx ->SR1;
		dummy_read = pI2CHandle ->pI2Cx ->SR2;
		(void)dummy_read;
	}
}

static void I2C_GenerateStopCondition(I2C_RegDef_t*pI2Cx)
{
	pI2Cx ->CR1 |= (1 << I2C_CR1_STOP);
}


uint32_t RCC_GetPCLK1Value (void)
{
	uint32_t Pclk1 , systemclck;
	uint8_t clcksrc, temp, ahbp, apb1;

	clcksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clcksrc == 0)
	{
		systemclck = 16000000;
	}
	else if (clcksrc == 1)
	{
		systemclck = 8000000;

	}
	else if (clcksrc == 2)
	{
		systemclck = RCC_GetPLLOutputClock();
	}

	// For AHB
    temp = ((RCC->CFGR >> 4) & 0xF);
    if (temp < 8)
    {
    	ahbp = 1;
    }
    else
    {
    	ahbp = AHB_Prescalar[temp-8];
    }

    // For APB1
    temp = ((RCC->CFGR >> 10) & 0x7);
    if (temp < 4)
    {
    	apb1 = 1;
    }
    else
    {
    	apb1 = APB1_Prescalar[temp-4];
    }

	Pclk1 = (systemclck/ahbp)/apb1;
	return Pclk1;

}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			if (pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}
			else if (pI2Cx == I2C4)
			{
				I2C4_PCLK_EN();
			}

		}
	else
		{
			if (pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}
			else if (pI2Cx == I2C4)
			{
				I2C4_PCLK_DI();
			}


		}
}




// Init and Deinit

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// Enable the clock for I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);



	// ACK control bit
	tempreg |= pI2CHandle ->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle ->pI2Cx ->CR1 = tempreg;

	// Configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//Program the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1<<14); // should be kept to 1 by software
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR Calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Mode is standard
		ccr_value = (RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF); // Mask out rest values since CCR is only 12 bits and rest are not needed

	}
	else
	{
		// Mode is fast
		tempreg |= (1<<15); // Fast mode configure in CCR register
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14); //Configure duty cycle to 14 th bit pos
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle  == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);

		pI2CHandle->pI2Cx->CCR = tempreg;
	}


	// Trise Configuration
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Mode is standard
		tempreg = (RCC_GetPCLK1Value()/1000000U) + 1;
	}
	else
	{
		// Mode is fast
		tempreg = (RCC_GetPCLK1Value() * 300 / 1000000000U) + 1; // In TRISE register given to increment by 1 and storing in the register

	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);






}



void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{


		if (pI2Cx == I2C1)
			{
				I2C1_REG_RESET();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_REG_RESET();
			}
			else if (pI2Cx == I2C4)
			{
				I2C4_REG_RESET();
			}


}


// FLAG status function
uint8_t I2C_GetFLagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if (pI2Cx->SR1 & (FlagName))
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


// Data send and receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm the start condition is set by checking the SB flag in SR1
	// While SB is cleared SCL will be stretched to low
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address to slave with the r/w' set to zero (total 8 bits )
	I2C_ExecuteAdressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in SR1
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear the ADDR flag according to its software
	I2C_ClearAddrFlag(pI2CHandle);

	// 6. Send the data until the length becomes 0
	while (Len > 0)
	{
		while (! I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	// 7. when length becomes 0 wait for TXE=1 and BTF=1 before generating the stop condition
	// When both TXE and BTF are set means that both SR and DR are empty and next transmission should begin
	// When BTF is 1 SCL will be stretched

	while (! I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while (! I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));


	if (Sr == I2C_DISABLE_SR)
	{
	// 8. Now generate the stop condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}

void I2C_MasterReceiveData (I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is complete by checking the SB flag in SR1
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address to slave with the r/w' bit set to 1
	I2C_ExecuteAdressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Wait until address phase is completed by checking the ADDR flag
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// Procedure to read only 1 byte
	if (Len == 1)
	{
		// Disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);


		// Clear the ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		// Wait until RxNE becomes 1
		while (! I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXE));


		// generate the stop condition
		if (Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


		// read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;



	}

	// Procedure to read data from slave when Len > 1
	if (Len >1)
	{
		// clear the ADDR Flag
		I2C_ClearAddrFlag(pI2CHandle);

		// read data until length becomes zero
		for (uint32_t i = Len ; i > 0 ; i--)
		{
			// wait until RxNE becomes 1
			while (! I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXE));

			if (i == 2) // if last 2 bytes are remaining
			{
				// Clear the ACK bit
				// Disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// Generate stop condition
				if(Sr == I2C_DISABLE_SR)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


			}

			// Read the data from DR in to the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;


			// Increment buffer address
			pRxBuffer++;


		}


	}


	// Re enabling the ACK
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

}

// Ack manager
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		// enable the ACK
		pI2Cx ->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx ->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


// I2C Interrupt close receive and sending data
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement code to disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);

	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement code to disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;



}


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle ->TxRxState;
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle ->pTxBuffer = pTxbuffer;
		pI2CHandle ->TxLen = Len;
		pI2CHandle ->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle ->DevAddr = SlaveAddr;
		pI2CHandle ->sr = Sr;

		// Implement code to generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement code to enable ITBUFEN control bit
		pI2CHandle ->pI2Cx->CR2 |= (1<<I2C_CR2_ITBUFEN);

		//Implement code to enable ITEVFEN control bit
		pI2CHandle ->pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);

		// Implement code to enable ITRREN control bit
		pI2CHandle ->pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);

	}
	return busystate;


}



uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{


	uint8_t busystate = pI2CHandle ->TxRxState;
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle ->pTxBuffer = pRxBuffer;
		pI2CHandle ->TxLen = Len;
		pI2CHandle ->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle ->DevAddr = SlaveAddr;
		pI2CHandle ->sr = Sr;

		// Implement code to generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement code to enable ITBUFEN control bit
		pI2CHandle ->pI2Cx->CR2 |= (1<<I2C_CR2_ITBUFEN);

		//Implement code to enable ITEVFEN control bit
		pI2CHandle ->pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);
		// Implement code to enable ITRREN control bit
		pI2CHandle ->pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);

	}
	return busystate;


}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > 0)
	{
	// 1. Load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
	// 2. Decrement the TxLen
		pI2CHandle->TxLen --;

	// 3. Increment the Buffer address
		pI2CHandle->pTxBuffer ++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
 {
	 // We have to do data reception
	 	if (pI2CHandle->RxSize == 1)
	 		{
	 		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
	 		pI2CHandle->RxLen --;

	 		}

	 		if (pI2CHandle->RxSize > 1)
	 		{
	 		if (pI2CHandle->RxLen == 2)
	 		{
	 			// clear Ack
	 			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

	 		}
	 		// Read DR
	 		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
	 		pI2CHandle->pRxBuffer ++;
	 		pI2CHandle->RxLen --;

	 		}

	 		if (pI2CHandle ->RxSize == 0)
	 		{
	 		// close the I2C Data reception and notify the application
	 		// 1. Generate stop condition
	 		if (pI2CHandle ->sr == I2C_DISABLE_SR)
	 		{
	 			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

	 		}
	 	  // 2. Close the I2C rx
	 		I2C_CloseReceiveData(pI2CHandle);

	 	 // 3. Notify the application
	 		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

	 	    }
 }

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle ->pI2Cx->CR2 & (1<<I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle ->pI2Cx->CR2 & (1<<I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle ->pI2Cx->SR1 & (1<<I2C_SR1_SB);

	//1. Handling interrupt generated by SB event
	// SB flag is only applicable in master mode
	if (temp1 && temp3)
	{
		// SB is set
		// not applicable for slave mode because in slave it is always set to 0
		// lets execute the address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAdressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);

		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAdressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	//2. Handling interrupt generated by ADDR event
	// when master mode: Address is sent
	// when slave mode : Address is matched with own address

	temp3 = pI2CHandle ->pI2Cx->SR1 & (1<<I2C_SR1_ADDR);
	if (temp1 && temp3)
		{
			// ADDR is set
		//  Clear ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);


		}
	//3. Handle for interrupt generated by BTF(Byte Transfer finished) event

	temp3 = pI2CHandle ->pI2Cx->SR1 & (1<<I2C_SR1_BTF);
	if (temp1 && temp3)
		{
			// BTF is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Make sure that TXE is also set
			if(pI2CHandle ->pI2Cx->SR1 & (1<<I2C_SR1_TXE))
			{
				// BTF and TXE = 1

				if (pI2CHandle->TxLen == 0)
				{

					// 1. Generate the stop condition
					if (pI2CHandle->sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					// 2. Reset all member elements of the handle structure

					I2C_CloseSendData(pI2CHandle);

					// 3. Notify the application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}


		}
	//4. Handle for STOPF event
	// Only applicable in slave mode
	// This code will not be executed by master since STOPF will not be set in master mode

	temp3 = pI2CHandle ->pI2Cx->SR1 & (1<<I2C_SR1_STOPF);
	if (temp1 && temp3)
		{
			// STOPF is set
			// Clear the STOPF flag (ie read SR1 and write to CR1)
			pI2CHandle ->pI2Cx ->CR1 |=  0x0000;

			// notify the application that stop is detected
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

		}

	//5. Handle for interrupt generated by TXE event
	temp3 = pI2CHandle ->pI2Cx->SR1 & (1<<I2C_SR1_TXE);
	if (temp1 && temp3 && temp2)
		{
			// Checking device mode proceed only if device in master mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
			{
				// TXE is set
				// We have to do the data transmission
				if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
				{
				 I2C_MasterHandleTXEInterrupt(pI2CHandle);

				}
			}
		}

	//6. Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle ->pI2Cx->SR1 & (1<<I2C_SR1_RXNE);
	if (temp1 && temp3 && temp2)
		{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// device is master
			// RXNE is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);


			}

		}
	}

}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun

		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
		}

}


