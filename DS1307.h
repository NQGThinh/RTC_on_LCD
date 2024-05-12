#ifndef DS1307_H
#define DS1307_H

#include "stm32f1xx.h"

/* application configurable items */
#define DS1307_I2C 												I2C1
#define DS1307_I2C_GPIO_PORT 							GPIOB
#define DS1307_I2C_SDA_PIN 								7
#define DS1307_I2C_SCL_PIN 								6
#define DS1307_I2C_SPEED 									I2C_SCL_SPEED_SM
#define DS1307_I2C_PUPD										AF_OPEN_DRAIN    /* open- drain */ 

/* register addresses */
#define DS1307_ADDR_SEC 					0x00
#define DS1307_ADDR_MIN 					0x01
#define DS1307_ADDR_HRS 					0x02
#define DS1307_ADDR_DAY 					0x03
#define DS1307_ADDR_DATE 					0x04
#define DS1307_ADDR_MONTH 				0x05
#define DS1307_ADDR_YEAR 					0x06


#define TIME_FORMAT_12HRS_AM			0
#define TIME_FORMAT_12HRS_PM			1
#define TIME_FORMAT_24HRS					2

#define DS1307_I2C_ADDRESS				0x68

#define SUNDAY				1
#define MONDAY				2
#define TUESDAY				3	
#define WEDNESDAY			4
#define THURSDAY			5
#define FRIDAY				6
#define SATURDAY			7


typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
}RTC_Date_t;


typedef struct
{
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t time_format;
}RTC_Time_t;



/* Function prototype */
uint8_t ds1307_init(void);
void ds1307_set_current_time(RTC_Time_t* rtc_time);
void ds1307_get_current_time(RTC_Time_t* rtc_time);

void ds1307_set_current_date(RTC_Date_t* rtc_date);
void ds1307_get_current_date(RTC_Date_t* rtc_date);








#endif