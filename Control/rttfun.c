#include "rttfun.h"
#include "Encoder.h"
#include "CarSport.h"
#include "PID.h"
#include "Navigation.h"
#include "Filter.h"
#include "Gyro.h"

/* 线程控制块 */
//rt_thread_t mcWheelCtl_thread;  //麦伦线程
//rt_thread_t camera_thread;      //相机线程
//rt_thread_t Nav_cal_thread;         //导航线程
//rt_thread_t Itrt_thread;        //交互线程
/* 全局变量 */



        	//初始化pwm


/* mcw 控制线程 */
//void mcWheelCtl_thread_entry(void* paramater)
//{
//	int32 tar_pulse[4] = {0};				//目标脉冲值（以脉冲值为准）
//	
//    while(1)
//    {     
//			
//			//Self_position(encd[0], encd[1], encd[2], encd[3]);
//			//angle = Displacement_angle(map[0] - x, map[1] - y, &r);

//			

////			
////			pwm[0] = (int32)out_limit(pid1.Out);
////				motor_r_up_PWM(pwm[0]);
////			pwm[1] = out_limit(pid2.Out);
////				motor_l_up_PWM(pwm[1]);
////			pwm[2] = out_limit(pid3.Out);
////				motor_r_down_PWM(pwm[2]);
////			pwm[3] = out_limit(pid4.Out);
////				motor_l_down_PWM(pwm[3]);
//			
//		//获取目标脉冲值
////		
////        //pid运算
////    pid1.tar  = tar_pulse[0] * 133;	//将目标脉冲值转换成目标pwm
////        pid_init(&pid1,encd[0]*133);		//将读取的脉冲值转换成pwm值输入到pid控制器
////		pwm[0] = (int32)out_limit(pid1.Out);	//读取pid运算后的out值
////        motor1PWM(pwm[0]);				//电机2 //输出pwm到电机
////		
////		pid2.tar  = 10 * 133;	//将目标脉冲值转换成目标pwm
////        pid_init(&pid2,encd[1]*133);		//将读取的脉冲值转换成pwm值输入到pid控制器
////		pwm[1] = out_limit(pid2.Out);	//读取pid运算后的out值
////        motor2PWM(pwm[1]);				//电机2 //输出pwm到电机
////        
////		pid3.tar  = tar_pulse[2] * 133;	//将目标脉冲值转换成目标pwm
////        pid_init(&pid3,encd[2]*133);		//将读取的脉冲值转换成pwm值输入到pid控制器
////		pwm[2] = out_limit(pid3.Out);	//读取pid运算后的out值
////        motor3PWM(pwm[2]);				//电机2 //输出pwm到电机
////        
////		pid4.tar  = tar_pulse[3] * 133;	//将目标脉冲值转换成目标pwm
////        pid_init(&pid4,encd[3]*133);		//将读取的脉冲值转换成pwm值输入到pid控制器
////		pwm[3] = out_limit(pid4.Out);	//读取pid运算后的out值
////        motor4PWM(pwm[3]);				//电机2 //输出pwm到电机
////		
//        rt_thread_delay(5);
//    }
//}

//void Nav_cal_thread_entry(void* parameter) 			//导航线程
//{
//	//mpu_6050_init();								//mpu初始化（使用dmp单元）
////	int32 car_x,car_y;car_x = car_y = 0;			//车身坐标初始化
////	int32 tar_x,tar_y;tar_x =tar_y = 0;				//目标点坐标初始化
////	int32 ang = 0;									//xy坐标系的角度
////	int32 tar_pulse[4] = {0};						//目标转速值（以脉冲值为准）
////	carSpdPsn_typedef carSP;
//	
//	while(1)
//	{
//		//mpu6050_get(&mpu6050);
////		car_ang_cvt(0/*入口参数，需要修改*/);
////		C_spd_pos(&carSP);
//			Encoder_get();
//		rt_thread_delay(5);
//	}
//}

//void Itrt_thread_entry(void* paramater)
//{
//    while(1)
//    {
//			
//				//printf("%f, %f\n", yaw_angle, angle_speed);
//        rt_thread_delay(20);
//    }
//}

