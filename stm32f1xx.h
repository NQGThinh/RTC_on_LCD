



#ifndef INC_STM32F1XX_H
#define INC_STM32F1XX_H

#include <stdint.h>

#define NULL  (void*)0

/*
 * Base address of Flash and Sram memmories
 */

#define FLASH_BASEADDR 		0x08000000
#define SRAM_BASEADDR 		0x20000000


/*
 * Base address of different bus domain
 */

#define PERIPH_BASE				0x40000000
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000
#define AHBPERIPH_BASE		0x40018000


/*
 * Base address of peripherals which are hanging on AHB bus
 */

#define CRC_BASEADDR		(AHBPERIPH_BASE + 0xB000)
#define RCC_BASEADDR		(AHBPERIPH_BASE + 0x9000)


/*
 * Base address of peripherals which are hanging on APB1 bus
 */

#define TIM2_BASEADDR							(APB1PERIPH_BASE)
#define TIM3_BASEADDR							(APB1PERIPH_BASE + 0x400)
#define SPI2_BASEADDR							(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR							(APB1PERIPH_BASE + 0x3C00)
#define I2C1_BASEADDR						  (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR						  (APB1PERIPH_BASE + 0x5800)
#define USART2_BASEADDR						(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASE + 0x4800)


/*
 * Base address of peripherals which are hanging on APB2 bus
 */

#define GPIOA_BASEADDR		(APB2PERIPH_BASE + 0x800)
#define GPIOB_BASEADDR		(APB2PERIPH_BASE + 0xC00)
#define GPIOC_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR		(APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR		(APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR		(APB2PERIPH_BASE + 0x2000)
#define AFIO_BASEADDR							 APB2PERIPH_BASE
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x400)
#define ADC1_BASEADDR			(APB2PERIPH_BASE + 0x2400)
#define ADC2_BASEADDR			(APB2PERIPH_BASE + 0x2800)
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x3800)

/******************************Peripheral Register Definition Structures*************/

typedef struct
{
	volatile unsigned int CR[2];			/* Configure register low */
	volatile unsigned int IDR;			
	volatile unsigned int ODR;
	volatile unsigned int BSRR;
	volatile unsigned int BRR;
	volatile unsigned int LCKR;
}GPIO_Regdef_t;

typedef struct
{
	volatile unsigned int EVCR;
	volatile unsigned int MAPR;
	volatile unsigned int EXTICR[4];
	volatile unsigned int MAPR2;
}AFIO_Regdef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBSTR;
	volatile uint32_t CFGR2;
}RCC_Regdef_t;


typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_Regdef_t;



/* Processor - NVIC controller */
typedef struct
{
	volatile uint32_t ISER[3];
	volatile uint32_t dummy1[29];
	volatile uint32_t ICER[3];
	volatile uint32_t dummy2[29];
	volatile uint32_t ISPR[3];
	volatile uint32_t dummy3[29];
	volatile uint32_t ICPR[3];
	volatile uint32_t dummy4[29];
	volatile uint32_t IABR[3];
	volatile uint32_t dummy5[61];
	volatile uint32_t IPR[21];
	volatile uint32_t dummy6[695];
	volatile uint32_t STIR;
}NVIC_Regdef_t;

/* Processor - NVIC Systick */
typedef struct
{
	volatile uint32_t CTRL;
	volatile uint32_t LOAD;
	volatile uint32_t VAL;
	volatile uint32_t CALIB;
}STK_Regdef_t;

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_Regdef_t;


typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_Regdef_t;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
}I2C_Regdef_t;


typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_Regdef_t;


typedef struct
{
	volatile uint32_t ACR;
}FLASH_Regdef_t;
#define FLASH      ((FLASH_Regdef_t*)FLASH_BASEADDR)


#define NVIC_BASEADDR	(0xE000E100)
#define NVIC			((NVIC_Regdef_t*)NVIC_BASEADDR)

#define SYSTICK_BASEADDR	(0xE000E010)
#define STK				((STK_Regdef_t*)SYSTICK_BASEADDR)

#define NVIC_ISER0	((volatile uint32_t*)0xE000E100)

#define NO_PR_BITS_IMPLEMENTED	4			/* Number of pr bits actually specific to MCU, ST case is 4*/






#define RCC				((RCC_Regdef_t*)RCC_BASEADDR)
#define EXTI			((EXTI_Regdef_t*)EXTI_BASEADDR)
#define AFIO			((AFIO_Regdef_t*)AFIO_BASEADDR)

/*
 * Clock enable for GPIOx peripherals
 */
#define GPIOA_CLK_EN()  (RCC->APB2ENR |= (1<<2))
#define GPIOB_CLK_EN()  (RCC->APB2ENR |= (1<<3))
#define GPIOC_CLK_EN()  (RCC->APB2ENR |= (1<<4))
#define GPIOD_CLK_EN()  (RCC->APB2ENR |= (1<<5))
#define GPIOE_CLK_EN()  (RCC->APB2ENR |= (1<<6))


/*
 * Clock disable for GPIOx peripherals
 */
#define GPIOA_CLK_DI()  (RCC->APB2ENR &=~ (1<<2))
#define GPIOB_CLK_DI()  (RCC->APB2ENR &=~ (1<<3))
#define GPIOC_CLK_DI()  (RCC->APB2ENR &=~ (1<<4))
#define GPIOD_CLK_DI()  (RCC->APB2ENR &=~ (1<<5))
#define GPIOE_CLK_DI()  (RCC->APB2ENR &=~ (1<<6))
#endif /* INC_STM32F4XX_H_ */

/* Clock enable for SYSCFG controller*/
#define AFIO_CLK_EN() (RCC->APB2ENR |= (1))

/* Clock disable for SYSCFG controller*/
#define AFIO_CLK_DI() (RCC->APB2ENR &= ~(1))

/* clock enable for SPIx */

#define SPI1_CLK_EN()	(RCC->APB2ENR |= (1<<12))
#define SPI2_CLK_EN()	(RCC->APB1ENR |= (1<<14))
#define SPI3_CLK_EN()	(RCC->APB1ENR |= (1<<15))

#define SPI1_CLK_DI()	(RCC->APB2ENR &= ~(1<<12))
#define SPI2_CLK_DI()	(RCC->APB1ENR &= ~(1<<14))
#define SPI3_CLK_DI()	(RCC->APB1ENR &= ~(1<<15))


/* clock enable for I2Cx */
#define I2C1_CLK_EN()	(RCC->APB2ENR |= (1<<21))
#define I2C2_CLK_EN()	(RCC->APB2ENR |= (1<<22))

/* clock enable for USARTx*/
#define USART1_CLK_EN()	(RCC->APB2ENR |= (1<<14))
#define USART2_CLK_EN()	(RCC->APB1ENR |= (1<<17))
#define USART3_CLK_EN()	(RCC->APB1ENR |= (1<<18))


/* clock disable for USARTx*/
#define USART1_CLK_DI()	(RCC->APB2ENR &= ~(1<<14))
#define USART2_CLK_DI()	(RCC->APB1ENR &= ~(1<<17))
#define USART3_CLK_DI()	(RCC->APB1ENR &= ~(1<<18))



/*
 * Macros to reset peripheral
 */

#define GPIOA_REG_RESET()	do{RCC->APB2RSTR |= (1<<2);   RCC->APB2RSTR &= ~(1<<2);}while(0)  /*do...while...condition zero - technique in C language*/
#define GPIOB_REG_RESET()	do{RCC->APB2RSTR |= (1<<3);   RCC->APB2RSTR &= ~(1<<3);}while(0)
#define GPIOC_REG_RESET()	do{RCC->APB2RSTR |= (1<<4);   RCC->APB2RSTR &= ~(1<<4);}while(0)
#define GPIOD_REG_RESET()	do{RCC->APB2RSTR |= (1<<5);   RCC->APB2RSTR &= ~(1<<5);}while(0)
#define GPIOE_REG_RESET()	do{RCC->APB2RSTR |= (1<<6);   RCC->APB2RSTR &= ~(1<<6);}while(0)

/* Some generic macro */



#define IRQ_EXTI0		6
#define IRQ_EXTI1		7
#define IRQ_EXTI2		8
#define IRQ_EXTI3		9
#define IRQ_EXTI4		10
#define IRQ_SPI1		35
#define IRQ_SPI2		36
#define IRQ_I2C1_EV			31
#define IRQ_I2C1_ER			32
#define IRQ_I2C2_EV			33
#define IRQ_I2C2_ER			34


/* Bit position definitions SPI_CR1 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST	7	
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE		14	
#define SPI_CR1_BIDIMODE	15

/* Bit position definitions SPI_CR2 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE				2
#define SPI_CR2_TXEIE				7
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_ERRIE				5

/* Bit position definitions SPI_SR */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7


/* Bit position definitions I2C_CR1 */
#define I2C_CR1_PE							0
#define I2C_CR1_SMBUS						1
#define I2C_CR1_SMBTYPE					3
#define I2C_CR1_ENARP						4
#define I2C_CR1_ENPEC						5
#define I2C_CR1_ENGC						6
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_START						8
#define I2C_CR1_STOP						9
#define I2C_CR1_ACK							10
#define I2C_CR1_POS							11
#define I2C_CR1_PEC							12
#define I2C_CR1_ALERT						13
#define I2C_CR1_SWRST						15

/* Bit position definitions I2C_CR2 */
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR2_DMAEN						11
#define I2C_CR2_LAST						12

/* Bit position definitions I2C_SR1 */
#define I2C_SR1_SB							0
#define I2C_SR1_ADDR						1
#define I2C_SR1_BTF							2
#define I2C_SR1_ADD10						3
#define I2C_SR1_STOPF						4
#define I2C_SR1_RXNE						6
#define I2C_SR1_TXE							7
#define I2C_SR1_BERR						8
#define I2C_SR1_ARLO						9
#define I2C_SR1_AF							10
#define I2C_SR1_OVR							11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT					14
#define I2C_SR1_SMBALERT				15


/* Bit position definitions I2C_SR2 */
#define I2C_SR2_MSL							0
#define I2C_SR2_BUSY						1
#define I2C_SR2_TRA							2
#define I2C_SR2_GENCALL					4
#define I2C_SR2_SMBDEFAULT			5
#define I2C_SR2_SMBHOST					6
#define I2C_SR2_DUALF						7

/* Bit position definitions I2C_CCR */





/* Bit position definitions USART_CR1 */
#define USART_CR1_SBK									0
#define USART_CR1_RWU									1
#define USART_CR1_RE									2
#define USART_CR1_TE									3
#define USART_CR1_IDLEIE							4
#define USART_CR1_RXNEIE							5
#define USART_CR1_TCIE								6
#define USART_CR1_TXEIE								7
#define USART_CR1_PEIE								8
#define USART_CR1_PS									9
#define USART_CR1_PCE									10
#define USART_CR1_WAKE								11
#define USART_CR1_M										12
#define USART_CR1_UE									13


/* Bit position definitions USART_CR3 */
#define USART_CR3_EIE									0
#define USART_CR3_IREN								1
#define USART_CR3_IRLP								2
#define USART_CR3_HDSEL								3
#define USART_CR3_NACK								4
#define USART_CR3_SCEN								5
#define USART_CR3_DMAR								6
#define USART_CR3_DMAT								7
#define USART_CR3_RTSE								8
#define USART_CR3_CTSE								9
#define USART_CR3_CTSIE								10












/* Peripheral pointers */

#define GPIOA			((GPIO_Regdef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_Regdef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_Regdef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_Regdef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_Regdef_t*)GPIOE_BASEADDR)

#define ADC1			((ADC_Regdef_t*)ADC1_BASEADDR)
#define ADC2			((ADC_Regdef_t*)ADC2_BASEADDR)

#define SPI1			((SPI_Regdef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_Regdef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_Regdef_t*)SPI3_BASEADDR)

#define I2C1			((I2C_Regdef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_Regdef_t*)I2C2_BASEADDR)


#define USART1			((USART_Regdef_t*)USART1_BASEADDR)
#define USART2			((USART_Regdef_t*)USART2_BASEADDR)
#define USART3			((USART_Regdef_t*)USART3_BASEADDR)




#define RESET 		0
#define SET 			1
#define FLAG_RESET 		0
#define FLAG_SET 			1
#define DISABLE 			0
#define ENABLE 			1
#define ERROR 			0
#define SUCCESS 			1


#include "GPIO.h"
#include "I2C.h"


