#include "PID.h"
#include "Filter.h"
#include "rttfun.h"

float Kp = 300;
float Ki = 55;
float Kd = 0;

//float Kp_2 = 400;
//float Ki_2 = 55;
//float Kd_2 = 0;

//float Kp_3 = 400;
//float Ki_3 = 55;
//float Kd_3 = 0;

//float Kp_4 = 400;
//float Ki_4 = 55;
//float Kd_4 = 0;

void pid_init(PID_initStruncture* pid)
{
	pid->tar = 0;
	pid->crt_speed = 0;
	pid->Err = 0;
	pid->last_err = 0;
	pid->next_err = 0;
	pid->add = 0;
	pid->p = Kp;
	pid->i = Ki;
	pid->d = Kd;
	pid->Out = 0;
}


void pid_L_up(PID_initStruncture* pid_init, int32 crt_pwm)
{
	pid_init->tar = crt_pwm;
	//pid_init->crt_speed = LowPass_encoder(speed_L_up[1],speed_L_up[0]);
	pid_init->crt_speed = encd[1];
	pid_init->Err = pid_init->tar - pid_init->crt_speed;
	pid_init->add = pid_init->p*(pid_init->Err - pid_init->last_err) + pid_init->i * (pid_init->Err) + pid_init->d * (pid_init->Err+pid_init->next_err-2*pid_init->last_err);
	pid_init->Out += pid_init->add;
	pid_init->next_err = pid_init->last_err;
	pid_init->last_err = pid_init->Err;
}

void pid_L_down(PID_initStruncture* pid_init,int32 crt_pwm)
{ 
	pid_init->tar = crt_pwm;
	//pid_init->crt_speed = LowPass_encoder(speed_L_down[1],speed_L_down[0]);
	pid_init->crt_speed = encd[3];
	pid_init->Err = pid_init->tar - pid_init->crt_speed;
	pid_init->add = pid_init->p*(pid_init->Err - pid_init->last_err) + pid_init->i * (pid_init->Err) + pid_init->d * (pid_init->Err+pid_init->next_err-2*pid_init->last_err);
	pid_init->Out += pid_init->add;
	pid_init->next_err = pid_init->last_err;
	pid_init->last_err = pid_init->Err;
}

void pid_R_up(PID_initStruncture* pid_init,int32 crt_pwm)
{
	pid_init->tar = crt_pwm;
	//pid_init->crt_speed = LowPass_encoder(speed_R_up[1],speed_R_up[0]);
	pid_init->crt_speed = encd[0];
	pid_init->Err = pid_init->tar - pid_init->crt_speed;
	pid_init->add = 100 *(pid_init->Err - pid_init->last_err) + 10 * (pid_init->Err) + pid_init->d * (pid_init->Err+pid_init->next_err-2*pid_init->last_err);
	pid_init->Out += pid_init->add;
	pid_init->next_err = pid_init->last_err;
	pid_init->last_err = pid_init->Err;
}

void pid_R_down(PID_initStruncture* pid_init,int32 crt_pwm)
{
	pid_init->tar = crt_pwm;
	//pid_init->crt_speed = LowPass_encoder(speed_R_down[1],speed_R_down[0]);
	pid_init->crt_speed = encd[2];
	pid_init->Err = pid_init->tar - pid_init->crt_speed;
	pid_init->add = pid_init->p*(pid_init->Err - pid_init->last_err) + pid_init->i * (pid_init->Err) + pid_init->d * (pid_init->Err+pid_init->next_err-2*pid_init->last_err);
	pid_init->Out += pid_init->add;
	pid_init->next_err = pid_init->last_err;
	pid_init->last_err = pid_init->Err;
}

//int32 out_limit(int32 Out)
//{
//	if(Out > 14000) return 14000;
//	else if(Out < -14000) return -14000;
//	else return Out;
//}
//2023年5月15日21:30:20
int32 out_limit(int32 Out)
{
	if(Out > 8000) return 8000;
	else if(Out < -8000) return -8000;
	else return Out;
}

void Position_pid(Position_PID* pid)
{
	pid->Kp = 0; 
	pid->Ki = 0; 
	pid->Kd = 0;
	pid->target = 0;
	pid->error[0] = 0;
	pid->error[1] = 0;
	pid->errSum = 0;
}

float Position_PID_Calc(Position_PID* pp)  // PD controller(There is P-controller)
{
		double dErr;
    //pp->errSum += Err; 
    dErr = pp->error[0] - pp->error[1]; 
    pp->error[1] = pp->error[0];      
    //return (pp->Kp * Err + pp->Ki * pp->errSum + pp->Kd * dErr);
		return (pp->Kp * pp->error[0] + pp->Kd * dErr);
}

// 左偏为负增长，右偏为正增长
int32 pid_correction(Position_PID* pid, float cur_posture)  
{
	int32 out;
	pid->Kp = 0.5, pid->Ki = 0.3, pid->Kd = 0.05;
	pid->error[0] = cur_posture - 0;
	pid->errSum += pid->error[0];
	out = pid->Kp * pid->error[0] + pid->Ki * pid->errSum + pid->Kd * pid->error[1];
	pid->error[1] = pid->error[0];
	return out;
}

 

