#include "Encoder.h"
#include "Filter.h"

void Encoder_init()
{
	encoder_quad_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1);
	encoder_quad_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
	encoder_quad_init(QTIMER3_ENCODER2, QTIMER3_ENCODER2_CH1_B18, QTIMER3_ENCODER2_CH2_B19);
	encoder_quad_init(QTIMER2_ENCODER1, QTIMER2_ENCODER1_CH1_C3, QTIMER2_ENCODER1_CH2_C25);
}

int16 speed_L_up[2];
int16 speed_L_down[2];
int16 speed_R_up[2];
int16 speed_R_down[2];
int16 encd[4] = {0};
int16 encd_sum[4];

void Encoder_get()
{
		
//	static int16 encd_r_up[5], encd_r_down[5], 
//				encd_l_up[5],  encd_l_down[5];
//	
//		encd_r_up[4] = encd_r_up[3];
//		encd_r_up[3] = encd_r_up[2];
//		encd_r_up[2] = encd_r_up[1];
//		encd_r_up[1] = encd_r_up[0];
		encd[0] = encoder_get_count(QTIMER1_ENCODER1);
	encd_sum[0] += encd[0];
//		speed_R_up[1] = speed_R_up[0];
//		speed_R_up[0] = (encd_r_up[4]*0.5f+encd_r_up[3]*0.5f+encd_r_up[2]*2.0f+encd_r_up[1]*3.0f+encd_r_up[0]*4.0f)/10.0f;
		//encd[0] = LowPass_encoder(speed_R_up[1],speed_R_up[0]);
			
//		encd_l_up[4] = encd_l_up[3];
//		encd_l_up[3] = encd_l_up[2];
//		encd_l_up[2] = encd_l_up[1];
//		encd_l_up[1] = encd_l_up[0];
		encd[1] = -encoder_get_count(QTIMER1_ENCODER2);
	encd_sum[1] += encd[1];
//		speed_L_up[1] = speed_L_up[0];
//		speed_L_up[0] = (encd_l_up[4]*0.5f+encd_l_up[3]*0.5f+encd_l_up[2]*2.0f+encd_l_up[1]*3.0f+encd_l_up[0]*4.0f)/10.0f;
//		encd[1] = LowPass_encoder(speed_L_up[1],speed_L_up[0]);
			
//		encd_r_down[4] = encd_r_down[3];
//		encd_r_down[3] = encd_r_down[2];
//		encd_r_down[2] = encd_r_down[1];
//		encd_r_down[1] = encd_r_down[0];
		encd[2] = encoder_get_count(QTIMER3_ENCODER2);
		encd_sum[2] += encd[2];
//		speed_R_down[1] = speed_R_down[0];
//		speed_R_down[0] = (encd_r_down[4]*0.5f+encd_r_down[3]*0.5f+encd_r_down[2]*2+encd_r_down[1]*3.0f+encd_r_down[0]*4.0f)/10.0f;
//		encd[2] = LowPass_encoder(speed_R_down[1],speed_R_down[0]);
			
//		encd_l_down[4] = encd_l_down[3];
//		encd_l_down[3] = encd_l_down[2];
//		encd_l_down[2] = encd_l_down[1];
//		encd_l_down[1] = encd_l_down[0];
		encd[3] = -encoder_get_count(QTIMER2_ENCODER1);
		encd_sum[3] += encd[3];
//		speed_L_down[1] = speed_L_down[0];
//		speed_L_down[0] = (encd_l_down[4]*0.5f+encd_l_down[3]*0.5f+encd_l_down[2]*2+encd_l_down[1]*3.0f+encd_l_down[0]*4.0f)/10.0f;
//		encd[3] = LowPass_encoder(speed_L_down[1],speed_L_down[0]);
	
	encoder_clear_count(QTIMER1_ENCODER1);
		encoder_clear_count(QTIMER1_ENCODER2);
		encoder_clear_count(QTIMER3_ENCODER2);
		encoder_clear_count(QTIMER2_ENCODER1);
}

float kp = 10, ki = 8, kd = 8;
void pid_encoder(PID_initStruncture* pid_init,int32 enco_tar, int32 enco)
{
	pid_init->tar = enco_tar;
	pid_init->crt_speed = enco;
	pid_init->Err = pid_init->tar - pid_init->crt_speed;
	pid_init->add = kp*(pid_init->Err - pid_init->last_err) + ki * (pid_init->Err) + kd * (pid_init->Err+pid_init->next_err-2*pid_init->last_err);
	pid_init->Out += pid_init->add;
	pid_init->next_err = pid_init->last_err;
	pid_init->last_err = pid_init->Err;
}

int32 speed_calc(int16 x)
{
	 return x / (1024 * 10);
}




