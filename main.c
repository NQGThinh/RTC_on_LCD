
#include "stdint.h"
#include "DS1307.h"

#define SYSTICK_TIM_CLK 16000000U


char* get_day_of_week(uint8_t i)
{
	char* day[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
	return day[i-1];
}

void number_to_string(uint8_t num, char* buf)
{
	if(num < 10)
	{
		buf[0] = '0';
		buf[1] = num + 48;
	}
	else if(num>=10 && num<99)
	{
		buf[0] = (num/10) + 48;
		buf[1] = (num%10) + 48;
	}
}

char* time_to_string(RTC_Time_t* rtc_time)
{
	static char buf[9];
	
	buf[2] = ':';
	buf[5] = ':';
	
	number_to_string(rtc_time->hour, buf);
	number_to_string(rtc_time->minute, &buf[3]);
	number_to_string(rtc_time->second, &buf[6]);
	
	buf[8] = '\0';
	return buf; 
}

char* date_to_string(RTC_Date_t* rtc_date)
{
	static char buf[9];
	
	buf[2] = '/';
	buf[5] = '/';
	
	number_to_string(rtc_date->date, buf);
	number_to_string(rtc_date->month, &buf[3]);
	number_to_string(rtc_date->year, &buf[6]);
	
	buf[8] = '\0';
	return buf;
}

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}


int main(void)
{
	RTC_Time_t current_time;
	RTC_Date_t current_date;
	
	if(ds1307_init())
	{
		printf("RTC init failed");
		while(1);
	}
	
	init_systick_timer(1);
	
	
	current_date.day = FRIDAY;
	current_date.date = 5;
	current_date.month = 5;
	current_date.year = 21;
	
	current_time.second = 41;
	current_time.minute = 31;
	current_time.hour = 4;
	current_time.time_format = TIME_FORMAT_12HRS_PM;
	
	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);
	
	
	ds1307_get_current_date(&current_date);
	ds1307_get_current_time(&current_time);
	
	char* am_pm;
	if(current_time.time_format |= TIME_FORMAT_24HRS)
	{
		am_pm = (current_time.time_format) ? "PM" : "AM";
		printf("Current time = %s %s", time_to_string(&current_time), am_pm);  /* 04:07:10 AM */
		
	}
	else 
	{
		printf("Current time = %s %s", time_to_string(&current_time));	/* 04:07:10 */
	}
	
	/* 15/01/21 <Friday> */
	printf("Current date = %s <%s> ", date_to_string(&current_date), get_day_of_week(current_date.day));
	
	
	while(1);
	return 0;
}



void SysTick_Handler(void)
{
	RTC_Time_t current_time;
	RTC_Date_t current_date;
	
	ds1307_get_current_date(&current_date);
	ds1307_get_current_time(&current_time);
	
	char* am_pm;
	if(current_time.time_format |= TIME_FORMAT_24HRS)
	{
		am_pm = (current_time.time_format) ? "PM" : "AM";
		printf("Current time = %s %s", time_to_string(&current_time), am_pm);  /* 04:07:10 AM */
		
	}
	else 
	{
		printf("Current time = %s %s", time_to_string(&current_time));	/* 04:07:10 */
	}
	
	/* 15/01/21 <Friday> */
	printf("Current date = %s <%s> ", date_to_string(&current_date), get_day_of_week(current_date.day));
}

	