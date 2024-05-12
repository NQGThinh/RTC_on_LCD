

#ifndef LCD_H
#define LCD_H

#include "stm32f1xx.h"

/* bsp APIs */
void lcd_init(void);

/* Application configurable items */
#define LCD_GPIO_PORT		GPIOD
#define LCD_GPIO_RS			GPIO_PIN_0
#define LCD_GPIO_RW			GPIO_PIN_1
#define LCD_GPIO_EN			GPIO_PIN_2
#define LCD_GPIO_D4			GPIO_PIN_3
#define LCD_GPIO_D5			GPIO_PIN_4
#define LCD_GPIO_D6			GPIO_PIN_5
#define LCD_GPIO_D7			GPIO_PIN_6






#endif