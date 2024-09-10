#ifndef _CarSport_H_
#define _CarSport_H_

#include "zf_common_headfile.h"

void motor_r_up_PWM(int PWM);
void motor_l_up_PWM(int PWM);
void motor_r_down_PWM(int PWM);
void motor_l_down_PWM(int PWM);
void leftF(int Xpwm,int Ypwm);//锟斤拷前锟斤拷
void rightF(int Xpwm,int Ypwm);//锟斤拷前锟斤拷
void motorContorl(uint8 motorNum,int32 pwm,uint8 dir);
void motor_out();

void P(void);
void CarCtrl(int Xpwm , int Ypwm);
void F(int pwm);//前锟斤拷
void RL(int Xpwm);//锟斤拷锟斤拷
void motor_init(void);

int SpeedLimit(int PWM);



extern int motor1basePWM ;
extern int motor2basePWM ;
extern int motor3basePWM ;
extern int motor4basePWM ;
extern int motorbasePWM ;

#endif

