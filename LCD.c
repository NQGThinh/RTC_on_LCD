
#include "LCD.h"

static void write_4_bits(uint8_t value);


void lcd_init(void)
{
	/* 1. Configure the GPIO pins which are use for lcd connection */
	GPIO_Handle_t lcd_signal;
	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = OUTPUT_10MHZ;
	lcd_signal.GPIO_PinConfig.GPIO_IOType = PUSH_PULL;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&lcd_signal);
	
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);
	
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);
	
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);
	
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);
	
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);
	
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);
	
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RS,0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RW,0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_EN,0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D4,0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D5,0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D6,0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D7,0);
	
	/* 2. Do the LCD init */
	mdelay(40);
	
	/* RS = 0, for LCD commmand */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RS,0);
	
	/* RnW = 0, writing to LCD */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RW,0);
	
	write_4_bits(0x3);
	
	mdelay(5);
	
	write_4_bits(0x3);
	
	udelay(100);
	
	write_4_bits(0x3);
	write_4_bits(0x2);	
}


/* write 4 bits of data/command on to D4,D5,D6,D7 lines */
static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D4,((value>>0) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D5,((value>>1) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D6,((value>>2) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D7,((value>>3) & 0x1));
	
	lcd_enable();
}



