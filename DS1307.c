#include "DS1307.h"

static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_address);
static uint8_t ds1307_read(uint8_t reg_address);
static uint8_t bcd_to_binary(uint8_t value);
static uint8_t binary_to_bcd(uint8_t value);
I2C_Handle_t	ds1307_I2C_Handle;

/* return 1: CH = 1, init failed */
/* return 1: CH = 0, init success */
uint8_t ds1307_init(void)
{
	/* 1.Init the I2C pins */
	ds1307_i2c_pin_config();
	
	/* 2.Init the I2C peripheral */
	ds1307_i2c_config();
	
	/* 3. Enable the I2C peripheral */
	I2C_PeripheralControl(DS1307_I2C, ENABLE);
	
	/* 4. Make clock halt bit = 0 */
	ds1307_write(0x00, DS1307_ADDR_SEC);
	
	/* 5. Read back CH bit */
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);
	
	return ((clock_state>>7) & 0x1);
}


void ds1307_set_current_time(RTC_Time_t* rtc_time)
{
	uint8_t seconds, minutes, hours;
	seconds = binary_to_bcd(rtc_time->second);
	seconds &= ~(1<<7);
	ds1307_write(seconds,DS1307_ADDR_SEC);
	
	minutes = binary_to_bcd(rtc_time->minute);
	ds1307_write(minutes,DS1307_ADDR_MIN);
	
	hours = binary_to_bcd(rtc_time->hour);
	if(rtc_time->time_format == TIME_FORMAT_24HRS)
	{
		hours &= ~(1<<6);
	}
	else
	{
		hours |= (1<<6);
		hours = (rtc_time->time_format == TIME_FORMAT_12HRS_PM) ? (hours | (1<<5)) : (hours &~(1<<5));
	}
	ds1307_write(hours, DS1307_ADDR_HRS);
	
	
}
void ds1307_get_current_time(RTC_Time_t* rtc_time)
{
	uint8_t seconds, hours;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	
	seconds &= ~(1<<7);
	rtc_time->second = bcd_to_binary(seconds);
	rtc_time->minute = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));
	
	hours = ds1307_read(DS1307_ADDR_HRS);
	if(hours & (1<<6))
	{
		/* 12 hour format */
		rtc_time->time_format = ((hours & (1<<5)) == 0) ? TIME_FORMAT_12HRS_AM : TIME_FORMAT_12HRS_PM;
	}
	else 
	{
		/* 24 hour format */
		rtc_time->time_format = TIME_FORMAT_24HRS;
	}
	rtc_time->hour = bcd_to_binary(hours);
}

void ds1307_set_current_date(RTC_Date_t* rtc_date)
{
	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);
	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
	ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);
	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
}
void ds1307_get_current_date(RTC_Date_t* rtc_date)
{
	rtc_date->date = ds1307_read(DS1307_ADDR_DATE);
	rtc_date->day = ds1307_read(DS1307_ADDR_DAY);
	rtc_date->month = ds1307_read(DS1307_ADDR_MONTH);
	rtc_date->year = ds1307_read(DS1307_ADDR_YEAR);
}


static void ds1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda, i2c_scl;
	
	memset(&i2c_sda,0,sizeof(i2c_sda));
	memset(&i2c_sda,0,sizeof(i2c_scl));
	
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = OUTPUT_10MHZ;
	i2c_sda.GPIO_PinConfig.GPIO_IOType = AF_OPEN_DRAIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	GPIO_Init(&i2c_sda);
	
	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = OUTPUT_10MHZ;
	i2c_scl.GPIO_PinConfig.GPIO_IOType = AF_OPEN_DRAIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	GPIO_Init(&i2c_scl);
	
}

static void ds1307_i2c_config(void)
{
	ds1307_I2C_Handle.pI2Cx = DS1307_I2C;
	ds1307_I2C_Handle.I2C_Config.ACKControl = I2C_ACK_ENABLE;
	ds1307_I2C_Handle.I2C_Config.SCL_Speed = DS1307_I2C_SPEED;
	I2C_Init(&ds1307_I2C_Handle);
}

static void ds1307_write(uint8_t value, uint8_t reg_address)
{
	uint8_t tx[2];
	tx[0] = reg_address;
	tx[1] = value;
	I2C_MasterSendData(&ds1307_I2C_Handle,tx,2,DS1307_I2C_ADDRESS);
}

static uint8_t ds1307_read(uint8_t reg_address)
{
	uint8_t data;
	I2C_MasterSendData(&ds1307_I2C_Handle, &reg_address, 1, DS1307_I2C_ADDRESS);
	I2C_MasterReceiveData(&ds1307_I2C_Handle, &data, 1, DS1307_I2C_ADDRESS);
	return data;
}

static uint8_t bcd_to_binary(uint8_t value)
{
	uint8_t m,n;
	m = (uint8_t) ((value>>4) * 10);
	n = value & (uint8_t)0x0f;
	return m+n;
}

static uint8_t binary_to_bcd(uint8_t value)
{
	uint8_t m,n;
	uint8_t bcd;
	bcd = value;
	if(value >= 10)
	{
		m = value/10;
		n = value%10;
		bcd = (uint8_t) ((m<<4)|n);
	}
	return bcd;
}


