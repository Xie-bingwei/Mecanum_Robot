#include "key.h"


void keyinit(void)
{
	gpio_init(C31,GPI,0,GPIO_PIN_CONFIG);
	gpio_init(C27,GPI,0,GPIO_PIN_CONFIG);
	gpio_init(C26,GPI,0,GPIO_PIN_CONFIG);
	gpio_init(C4,GPI,0,GPIO_PIN_CONFIG);

}

uint8 key_num(void)
{
	uint16 keynum;
	if(!gpio_get_level(C31)) 
	{
		system_delay_ms(15);
		keynum = 31;
		system_delay_ms(15);
	}
	if(!gpio_get_level(C27)) 
	{
		system_delay_ms(15);
		keynum = 27;
		system_delay_ms(15);
	}
	if(!gpio_get_level(C26)) 
	{
		system_delay_ms(15);
		keynum = 26;
		system_delay_ms(15);
	}
	if(!gpio_get_level(C4)) 
	{
		system_delay_ms(15);
		keynum = 4;
		system_delay_ms(15);
	}
	return keynum;
}

uint8 key_c31(void)
{
	uint8 keynum;
	if(!gpio_get_level(C31))	
	{	
		system_delay_ms(15);
		keynum = 31;
		system_delay_ms(15);
	}
	else keynum = 0;
	return keynum;
}

uint8 key_c27(void)
{
	uint8 keynum;
	if(!gpio_get_level(27))	
	{	
		system_delay_ms(15);
		keynum = 27;
		system_delay_ms(15);
	}
	else keynum = 0;
	return keynum;
}

uint8 key_c26(void)
{
	uint8 keynum;
	if(!gpio_get_level(C26))	
	{	
		system_delay_ms(15);
		keynum = 26;
		system_delay_ms(15);
	}
	else keynum = 0;
	return keynum;
}

uint8 key_c4(void)
{
	uint8 keynum;
	if(!gpio_get_level(C4))	
	{	
		system_delay_ms(15);
		keynum = 4;
		system_delay_ms(15);
	}
	else keynum = 0;
	return keynum;
}