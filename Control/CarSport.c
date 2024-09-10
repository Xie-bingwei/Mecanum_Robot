#include "CarSport.h"
#include "Navigation.h"

int motor1basePWM = 0;
int motor2basePWM = 0;
int motor3basePWM = 0;
int motor4basePWM = 0;

void motor_init(void)
{
	  /* motor initialize 
  *pwm初始化	输出占空比为0
  */
 	/* 有关电机控制的gpio的初始化 */
  	gpio_init(D0, GPO, 0, GPIO_PIN_CONFIG);
	gpio_init(D1, GPO, 0, GPIO_PIN_CONFIG);
	gpio_init(D12, GPO, 0, GPIO_PIN_CONFIG);
	gpio_init(D13, GPO, 0, GPIO_PIN_CONFIG);
	/* 4路pwm初始化 */
	pwm_init(PWM2_MODULE3_CHA_D2, 17*1000, 0);
	pwm_init(PWM2_MODULE3_CHB_D3, 17*1000, 0);
	pwm_init(PWM1_MODULE1_CHA_D14, 17*1000, 0);
	pwm_init(PWM1_MODULE1_CHB_D15, 17*1000, 0);
    rt_kprintf("motor initialize complete!\n");

}

int SpeedLimit(int PWM)
{
	if(PWM > 50000) return 50000;
		else return PWM;
}


void CarCtrl(int Xpwm , int Ypwm)
{
	if(Ypwm > 10 || Ypwm < -10)
	{
		if( Xpwm >= -20 && Xpwm <= 20) F(Ypwm*200);
		else if ( Xpwm >= 45 && Xpwm <= 50)  leftF(Xpwm*200,Ypwm*200);//左前后
		else if ( Xpwm <= -45 && Xpwm >= -55)  rightF(Xpwm*200,Ypwm*200);//右前后
	}
	
	else RL(Xpwm*200);
}

void RL(int Xpwm) //左右
{
	motor_r_up_PWM(-Xpwm);
	motor_r_down_PWM(-Xpwm);
	motor_l_up_PWM(Xpwm);
	motor_l_down_PWM(Xpwm);
}

void F(int pwm)//前后
{
	motor_r_up_PWM(pwm+motor1basePWM);
	motor_l_up_PWM(pwm+motor1basePWM);
	motor_r_down_PWM(pwm+motor1basePWM);
	motor_l_down_PWM(pwm+motor1basePWM);
}

void rightF(int Xpwm,int Ypwm)//右前后
{
	if(Ypwm > 0)
	{
		motor_r_up_PWM(Ypwm);
		motor_r_down_PWM(Ypwm);
		motor_l_up_PWM(0);
		motor_l_down_PWM(0);
	}
	else 
	{
		motor_r_up_PWM(0);
		motor_r_down_PWM(0);
		motor_l_up_PWM(Ypwm);
		motor_l_down_PWM(Ypwm);
	}
	
}

void leftF(int Xpwm,int Ypwm)//左前后
{
	if(Ypwm > 0)
	{
		motor_l_up_PWM(Ypwm);
		motor_l_down_PWM(Ypwm);
		motor_r_up_PWM(0);
		motor_r_down_PWM(0);
	}
	else 
	{
		motor_l_up_PWM(0);
		motor_l_down_PWM(0);
		motor_r_up_PWM(Ypwm);
		motor_r_down_PWM(Ypwm);
	}
	
}



void motor_l_up_PWM(int PWM)
{
	
	if(PWM > 0)
	{
		gpio_set_level(D0, 1);
		pwm_set_duty(PWM2_MODULE3_CHA_D2, PWM);
	}
	else
	{
		gpio_set_level(D0, 0);
		pwm_set_duty(PWM2_MODULE3_CHA_D2, -PWM);
	}
	
}

void motor_r_up_PWM(int PWM)
{
	if(PWM > 0)
	{
			gpio_set_level(D1, 1);
			pwm_set_duty(PWM2_MODULE3_CHB_D3, PWM);
	}
	else
	{
			gpio_set_level(D1, 0);
			pwm_set_duty(PWM2_MODULE3_CHB_D3, -PWM);
	}
}

void motor_r_down_PWM(int PWM)
{
		if(PWM > 0)
	{
		gpio_set_level(D12, 0);
		pwm_set_duty(PWM1_MODULE1_CHA_D14, PWM);
	}
	else 
	{
		gpio_set_level(D12, 1);
		pwm_set_duty(PWM1_MODULE1_CHA_D14, -PWM);
	}
}

void motor_l_down_PWM(int PWM)
{
	if(PWM>0)
	{
		gpio_set_level(D13, 0);
		pwm_set_duty(PWM1_MODULE1_CHB_D15, PWM);
	}
	else
	{
			gpio_set_level(D13, 1);
			pwm_set_duty(PWM1_MODULE1_CHB_D15, -PWM);
	}
}



/*
*parametr:
*	dir: shoud be -1 or 1
*/
void motorContorl(uint8 motorNum,int32 pwm,uint8 dir)
{
	if(motorNum == 1) motor_r_up_PWM(pwm*dir);
	if(motorNum == 2) motor_l_up_PWM(pwm*dir);
	if(motorNum == 3) motor_r_down_PWM(pwm*dir);
	if(motorNum == 4) motor_l_down_PWM(pwm*dir);
}

void P(void)
{
	motor_r_up_PWM(0);
	motor_l_up_PWM(0);
	motor_r_down_PWM(0);
	motor_l_down_PWM(0);
}

