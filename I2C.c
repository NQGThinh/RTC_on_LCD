
#include "I2C.h"

uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_Prescaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_Regdef_t* pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_Regdef_t* pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_Regdef_t* pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle);
static void I2C_GenerateStopCondition(I2C_Regdef_t* pI2Cx);
static void I2C_MasterHandleTXEInterupt(I2C_Handle_t* pI2CHandle);
static void I2C_MasterHandleRXNEInterupt(I2C_Handle_t* pI2CHandle);


uint8_t I2C_GetFlagStatus(I2C_Regdef_t* pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_ManageAcking(I2C_Regdef_t* pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1<<10);
	}
	else 
	{
		pI2Cx->CR1 &= ~(1<<10);
	}
}

static void I2C_GenerateStartCondition(I2C_Regdef_t* pI2Cx)
{
	pI2Cx->CR1 |= (1<<8);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_Regdef_t* pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);    /* SlaveAddr is slave address + r/w bit */
	pI2Cx->DR = SlaveAddr;
}

void I2C_ExecuteAddressPhaseRead(I2C_Regdef_t* pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1);    /* SlaveAddr is slave address + r/w bit */
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle)
{
	uint32_t dummy_read;
	/* check for device mode */
	if(pI2CHandle->pI2Cx->SR2 & 1)  
	{
		/* Master mode */
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				/* First, disable the ACK */
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				
				/* Clear the ADDR flag by reading SR1, SR2 */
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void) dummy_read;  /* avoid compiler warning because unuse variable */
			}
		}
		else 
		{
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
		  (void) dummy_read;   /* avoid compiler warning because unuse variable */
		}
	}
	else 
	{
		/* Slave mode */
		/* Clear the ADDR flag by reading SR1, SR2 */
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void) dummy_read;   /* avoid compiler warning because unuse variable */
	}
}

static void I2C_GenerateStopCondition(I2C_Regdef_t* pI2Cx)
{
	pI2Cx->CR1 |= (1<<9);
}

void I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle)
{
	/* Implement the code to disable ITBUFEN control bit */
	pI2CHandle->pI2Cx->CR2 &= ~(1<<10);
	
	/* Implement the code to disable ITEVFEN control bit */
	pI2CHandle->pI2Cx->CR2 &= ~(1<<9);
	
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if(pI2CHandle->I2C_Config.ACKControl == ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}
	
}
void I2C_CloseSendData(I2C_Handle_t* pI2CHandle)
{
	/* Implement the code to disable ITBUFEN control bit */
	pI2CHandle->pI2Cx->CR2 &= ~(1<<10);
	
	/* Implement the code to disable ITEVFEN control bit */
	pI2CHandle->pI2Cx->CR2 &= ~(1<<9);
	
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
}


void I2C_PeriClockControl(I2C_Regdef_t* pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)  					{I2C1_CLK_EN();}
		else if(pI2Cx == I2C2)			{I2C2_CLK_EN();}
	}
	else if(EnOrDi == DISABLE)
	{
		if(pI2Cx == I2C1)  					{SPI1_CLK_DI();}
		else if(pI2Cx == I2C2)			{SPI2_CLK_DI();}
	}
}

void I2C_PeripheralControl(I2C_Regdef_t* pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
			pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}
	else if(EnOrDi == DISABLE)
	{
			pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}
}


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, Systemclk;
	uint8_t clksrc, temp, ahbp, apb1p;
	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if(clksrc == 0)
	{
		Systemclk = 16000000;   /* HSI */
	}
	else if(clksrc == 1)
	{
		Systemclk = 80000000;		/* HSE */
	}
	
	/* for AHB */
	temp = ((RCC->CFGR >> 4) & 0xf);
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescaler[temp - 8];
	}
	
	/* for APB1 */
	temp = ((RCC->CFGR >> 8) & 0x7);
	if(temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_Prescaler[temp - 4];
	}
	
	pclk1 = ((Systemclk/ahbp)/apb1p);
	
	return pclk1;
}

void I2C_Init(I2C_Handle_t* pI2CHandle)
{
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	/* ack control bit */
	uint32_t tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;
	
	/* configure the freg field of CR2 */
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3f);
	
	/* program the device own address */
	tempreg |= (pI2CHandle->I2C_Config.DeviceAdrress << 1);
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;
	
	/* CCR calculation */
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.SCL_Speed <= I2C_SCL_SPEED_SM )
	{
		/* mode is standard mode */
		ccr_value = (RCC_GetPCLK1Value() / (2*pI2CHandle->I2C_Config.SCL_Speed));
		tempreg |= (ccr_value & 0xfff);
	}
	else
	{
		/* mode is fast mode */
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3*pI2CHandle->I2C_Config.SCL_Speed));
		}
		else if(pI2CHandle->I2C_Config.FMDutyCycle == I2C_FM_DUTY_16_9)
		{
			ccr_value = (RCC_GetPCLK1Value() / (25*pI2CHandle->I2C_Config.SCL_Speed));
		}
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
	
	/* Trise configuration */
	if(pI2CHandle->I2C_Config.SCL_Speed <= I2C_SCL_SPEED_SM )
	{
		/* mode is standard mode */
		tempreg = (RCC_GetPCLK1Value()/1000000) + 1 ;
	}
	else 
	{
		/* mode is fast mode */
		tempreg = ((RCC_GetPCLK1Value()*300)/1000000) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = tempreg & 0x3f;
}

void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	/* 1. Generate the start condition */
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	
	/* 2. confirm that start generation is completed by checking the SB flag in the SR1 */
	/* Note: Until SB is cleared, the SCl will be stretched (pull to low) */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	
	/* 3. Send the address of the slave with the R/W bit set to w(0) ( total 8 bits ) */
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);
	
	/* 4. Confirm that address phase is completed by checking the ADDR flag in SR1 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));
	
	/* 5. Clear the ADDR flag according to its software sequence */
	/* Note: Until ADDR is cleared, SCL will be stretched (pull to low)*/
	I2C_ClearADDRFlag(pI2CHandle);
	
	/* 6. Send data until Len become 0*/
	while(Len>0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	
	/* 7. When Len becomes 0, wait for TXE=1 and BTF=1 before generating the stop condition */
	/* Note: TXE=1, BTF=1 , means that both SR and DR are empty and next transmission should begin */
	/* when BTF=1 SCL will be stretched (pull to low) */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));
	
	/* 8. Generate STOP condition and master need not to wait for the completion of stop condition. */
	/* Note: Generating STOP, automatically clears the BTF */
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	
}

void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	/* 1. Generate start condition */
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	
	/* 2. Confirm that start generation is completed by checking the SB flag in the SR1 */
	/* Note: Until SB is clear SCL will be stretched (pull to low) */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	
	/* 3. Send the address of the slave with r/w bit set to R(1) (total 8 bits) */
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);
	
	/* 4. Wait until address phase is completed by checking ADDR flag in SR1 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));
	
	/* produce to read only 1 byte from slave */
	if(Len == 1)
	{
		/* Disable Acking */
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		/* clear the ADDR flag */
		I2C_ClearADDRFlag(pI2CHandle);
		/* wait until RXNE becomes 1 */
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));
		/* generate stop condition */
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		/* read data into buffer */
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		return;
	}
	
	/* produce to read data from slave when Len > 1 */
	if(Len>1)
	{
		/* Clear the ADDR flag */
		I2C_ClearADDRFlag(pI2CHandle);
		
		/* Read the data until Len becomes zero */
		for(uint32_t i=Len; i>0; i--)
		{
			/* wait until RXNE become 1 */
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));
			if(i==2)
			{
				/* clear the ACK bit */
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				/* generate STOP condition */
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			/* read the data to buffer */
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			/* increment the buffer address*/
			pRxBuffer++;
		}
	}
	
	/* Re-enable Acking */
	if(pI2CHandle->I2C_Config.ACKControl == ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
	if((busystate != I2C_BUSY_IN_RX) && (busystate!= I2C_BUSY_IN_TX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		
		/* Implement the code to generate the START condition */
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		
		/* Implement the code to enable ITBUFEN Control bit */
		pI2CHandle->pI2Cx->CR2 |= (1<<10);
		
		/* Implement the code to enable ITEVTEN Control bit*/
		pI2CHandle->pI2Cx->CR2 |= (1<<9);
		
		/* Implement the code to enable ITERTEN Control bit*/
		pI2CHandle->pI2Cx->CR2 |= (1<<8);
		
	}
	
	return busystate;
}


static void I2C_MasterHandleTXEInterupt(I2C_Handle_t* pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		/* 1. Load data to DR */
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		/* 2. Decrease the TxLen */
		pI2CHandle->TxLen--; 
		/* 3. Increase the TxBuffer */
		pI2CHandle->pTxBuffer++;
	}
}


static void I2C_MasterHandleRXNEInterupt(I2C_Handle_t* pI2CHandle)
{
	/* We have to do data receiption */
			
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pTxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
			
	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			/* clear the ACK bit */
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}
		*pI2CHandle->pTxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pTxBuffer++;
		pI2CHandle->RxLen--;
	}
			
	if(pI2CHandle->RxLen == 0)
	{
		/* close the I2C data reception and notify the application */
		/* Generate the stop condition */
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		/* Close the I2C Rx */
		I2C_CloseReceiveData(pI2CHandle);
	}
}


void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle)
{
	/* Interupt Handling for both master and slave mode of a device */
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1<<9);    /* ITEVTEN */
	temp2 = pI2CHandle->pI2Cx->CR2 & (1<<10);		/* ITBUFEN */
	
	temp3 = pI2CHandle->pI2Cx->SR1 & (1);				/* SB */	
	/* 1. Handle for interupt generate by SB event */
	/* Note: SB flag is only applicable in master mode */
	if(temp1 && temp3)
	{
		/* SB flag is set */
		/* This block is not executed in slave mode because for slave SB is always zero */
		/* In this block lets execute address phase */
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}
	
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<1);	
	/* 2. Handle for interupt generate by ADDR event */
	/* Note: When master mode : Address is sent */
	/* Note: When slave mode : Address matched with its own address */
	if(temp1 && temp3)
	{
		/* ADDR flag is set */
		I2C_ClearADDRFlag(pI2CHandle);
	}
	
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<2);	
	/* 3. Handle for interupt generate by BTF (Byte Transfer Finish) event */
	if(temp1 && temp3)
	{
		/* BTF flag is set */
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->pI2Cx->SR1 &(1<<7))  /* make sure that TXE also set */
			{
				/* BTF, TXE = 1 */
				if(pI2CHandle->TxLen == 0)
				{
					/* 1. Generate the STOP condition */
					if(pI2CHandle->Sr == DISABLE)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					/* 2. Reset all member elements of the handle structure */
					I2C_CloseSendData(pI2CHandle);
				}
			}
		}
	}
	
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<4);	
	/* 4. Handle for interupt generate by STOPF event */
	/* Note: Stop detection flag is applicable only slave mode  */
	if(temp1 && temp3)
	{
		/* STOPF flag is set */
		/* Clear the STOPF by     1. Read SR1  2. Write CR1 */
		pI2CHandle->pI2Cx->CR1 |= 0x0;   /* Do not write any invallid data, otherwise peripheral won't work properly */
	}
	
	
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<7);	
	/* 5. Handle for interupt generate by TXE event */
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & 1)  /* Check for device mode */
		/* TXE flag is set */
		/* We have to do the data transmission */
		if(pI2CHandle->pI2Cx == I2C_BUSY_IN_TX)
		{
			I2C_MasterHandleTXEInterupt(pI2CHandle);
		}
	}
	
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<6);	
	/* 6. Handle for interupt generate by RXNE event */
	if(temp1 && temp2 && temp3)
	{
		/* RXNE flag is set */
		if(pI2CHandle->pI2Cx == I2C_BUSY_IN_RX)
		{
			I2C_MasterHandleRXNEInterupt(pI2CHandle);
		}
	}	
}
void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle)
{
}

	



