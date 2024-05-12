
#ifndef INC_I2C_H
#define INC_I2C_H

#include"stm32f1xx.h"
typedef struct
{
	uint32_t SCL_Speed;
	uint8_t DeviceAdrress;
	uint8_t ACKControl;
	uint16_t FMDutyCycle;
}I2C_PinConfig_t;

typedef struct
{
	I2C_Regdef_t* pI2Cx;
	I2C_PinConfig_t I2C_Config;
	uint8_t* pTxBuffer;
	uint8_t* pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t  TxRxLen;
	uint8_t  TxRxState;				/* To store communication state */
	uint8_t  DevAddr;					/* To store slave/device address */
	uint32_t RxSize;					/* To store Rx size */
	uint8_t  Sr;							/* To store repeat start value */
}I2C_Handle_t;



/* I2C SCL speed */
#define I2C_SCL_SPEED_SM    	100000
#define I2C_SCL_SPEED_FM4K    400000
#define I2C_SCL_SPEED_FM2K    200000

/* I2C ACK control */
#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0

/* I2C FMDutyCycle */
#define I2C_FM_DUTY_2					0
#define I2C_FM_DUTY_16_9			1	

/* I2C application state */
#define I2C_READY							0
#define I2C_BUSY_IN_RX				1
#define I2C_BUSY_IN_TX				2

/* Flag definition */
#define I2C_FLAG_SB 					(1<<0)
#define I2C_FLAG_ADDR 				(1<<1)
#define I2C_FLAG_BTF 					(1<<2)
#define I2C_FLAG_ADD10 				(1<<3)
#define I2C_FLAG_STOPF 				(1<<4)
#define I2C_FLAG_RXNE 				(1<<6)
#define I2C_FLAG_TXE 					(1<<7)
#define I2C_FLAG_BERR 				(1<<8)
#define I2C_FLAG_AF 					(1<<10)
#define I2C_FLAG_OVR 					(1<<11)
#define I2C_FLAG_TIMEOUT 			(1<<14)

/* GPIO clock control */
void I2C_PeriClockControl(I2C_Regdef_t* pI2Cx, uint8_t EnOrDi);

/* GPIO init and deinit */
void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_Regdef_t* pI2Cx);

/* Data Send and Reveive */
void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr);

/* Data Send and Reveive using interupt */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_CloseSendData(I2C_Handle_t* pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle);

/* IRQ Configure and ISR handling */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle);
uint32_t RCC_GetPCLK1Value(void);

void I2C_PeripheralControl(I2C_Regdef_t* pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_Regdef_t* pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_Regdef_t* pI2Cx,uint8_t EnOrDi);


#endif

