#include "init.h"
#include "zf_common_headfile.h"

void Beep_init()
{
	gpio_init(B11,GPO,0,GPIO_PIN_CONFIG);
}

void Init_all()
{
	/* 片上硬件初始化 */
	uart_init(UART_4,115200,UART4_TX_C16,UART4_RX_C17);
//	uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);//2023年5月15日09:01:35
	
	imu_963r_init();
	Gyrooffset_init();
	Beep_init();
	Encoder_init();  
	motor_init();
	pid_init_all();
	uart_rx_interrupt(UART_4, 1); 
//	uart_rx_interrupt(UART_1, 1); //2023年5月15日09:01
	
	/* Interrupt init*/
	pit_ms_init(PIT_CH0, 5);
	pit_ms_init(PIT_CH1, 5);
	pit_ms_init(PIT_CH2, 5);
	interrupt_set_priority(PIT_IRQn, 0);
}

