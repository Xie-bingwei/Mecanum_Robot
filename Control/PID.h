#ifndef __PID_H
#define __PID_H

#include "zf_common_headfile.h"

typedef struct//增量式PID相关参数
{
	float tar ;
	float crt_speed;
	float Err ;
	float last_err;
	float next_err;
	float add;
	float p;
	float i;
	float d;
	int32 Out;
}PID_initStruncture;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float target;  // 目标值
		float actual;
    float error[2];  //最后一次误差 
    float errSum; // 误差积分
		float pos_out;
}Position_PID;

void pid_init(PID_initStruncture* pid);
void pid_init_all();
void pid_L_up(PID_initStruncture* pid, int32 crt_pwm);
void pid_L_down(PID_initStruncture* pid,int32 crt_pwm);
void pid_R_up(PID_initStruncture* pid,int32 crt_pwm);
void pid_R_down(PID_initStruncture* pid,int32 crt_pwm);
void pid_encoder(PID_initStruncture* pid_init,int32 enco_tar, int32 enco);
int32 pid_correction(Position_PID* pid, float cur_posture);
int32 out_limit(int32 Out);

float Position_PID_Calc(Position_PID* pp);

void wheel_output(float x_cur, float x_tar, float y_cur, float y_tar, float target_angle_speed);

void wheel_X(float x_cur, float x_tar, int16 encd1, int16 encd2, int16 encd3, int16 encd4, int16 encd_tar, 
	PID_initStruncture* pid1,
	PID_initStruncture* pid2,
	PID_initStruncture* pid3,
	PID_initStruncture* pid4);

#endif 
