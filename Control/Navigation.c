#include "Navigation.h"
#include "Trigonometric.h"
#include "Filter.h"
//#include "mpu6050.h"
#include "inv_mpu.h"
#include "Gyro.h"

PID_initStruncture pid1;
PID_initStruncture pid2;
PID_initStruncture pid3;
PID_initStruncture pid4;

uint8 motor_flag;
float distance, x_tar = 50;

float pid1_flag, pid2_flag, pid3_flag, pid4_flag;

void pid_init_all()
{
	pid_init(&pid1);
	pid_init(&pid2);
	pid_init(&pid3);
	pid_init(&pid4);
}

float tar_angle_speed = 0;

uint8 open_flag = 0;     // 任务开启标志位

//void pid_init_all()
//{
//	pid_init(&pid1);
//	pid_init(&pid2);
//	pid_init(&pid3);
//	pid_init(&pid4);
//}

void motor_out()
{
	pid1.Out = (pid1.Out > 15000) ? 15000 : pid1.Out;
	pid2.Out = (pid2.Out > 15000) ? 15000 : pid2.Out;
	pid3.Out = (pid3.Out > 15000) ? 15000 : pid3.Out;
	pid4.Out = (pid4.Out > 15000) ? 15000 : pid4.Out;
	pid1.Out = (pid1.Out < -15000) ? -15000 : pid1.Out;
	pid2.Out = (pid2.Out < -15000) ? -15000 : pid2.Out;
	pid3.Out = (pid3.Out < -15000) ? -15000 : pid3.Out;
	pid4.Out = (pid4.Out < -15000) ? -15000 : pid4.Out;
	pid1_flag = pid1.Out;
	pid2_flag = pid2.Out;
	pid3_flag = pid3.Out;
	pid4_flag = pid4.Out;
	//2023年5月12日17:41:42
	if(60 - y > 0.35 && 60 - x > 0.35 && x > 0.2 && y > 0.2) {
		motor_r_up_PWM(pid1.Out + angle_speed * 400);
		motor_l_up_PWM(pid2.Out - angle_speed * 400);
		motor_r_down_PWM(pid3.Out + angle_speed * 400);
		motor_l_down_PWM(pid4.Out - angle_speed * 400);
//			motor_r_up_PWM(10000);
//		motor_l_up_PWM(10000);
//		motor_r_down_PWM(10000);
//		motor_l_down_PWM(10000);
	}
	else
	{
			motor_r_up_PWM(pid1.Out);
		motor_l_up_PWM(pid2.Out);
		motor_r_down_PWM(pid3.Out);
		motor_l_down_PWM(pid4.Out);
	}
}

void limit_pwm()
{
			R_up = (int32)out_limit(R_up);
			L_up = out_limit(L_up);
			R_down = out_limit(R_down);
			L_down = out_limit(L_down);
}

void imu_963r_init(void)
{
	uint8 m = imu963ra_init();
	if(m == 1)
	{
		printf("imu_init failed\n");
		return;
	}
}

float nav_fabs(float x) {
    return (x >= 0) ? x : -x;
}

float arctan_90(float x, float y, float* r)   // 角度，位移计算
{
    float angle[50] = {45.0, 26.565051177077986, 14.036243467926479, 7.125016348901799,
                        3.5763343749973515, 1.7899106082460694, 0.8951737102110744,
                        0.4476141708605531, 0.22381050036853808, 0.1119056770662069,
                        0.05595289189380367, 0.02797645261700368, 0.013988227142265015,
                        0.006994113675352919, 0.003497056850704011, 0.0017485284269804495,
                        0.0008742642136937803, 0.00043713210687233457, 0.00021856605343934782,
                        0.00010928302672007149, 5.464151336008544e-05, 2.732075668004893e-05,
                        1.3660378340025243e-05, 6.830189170012718e-06, 3.4150945850063712e-06,
                        1.7075472925031871e-06, 8.537736462515938e-07, 4.2688682312579694e-07,
                        2.1344341156289847e-07, 1.0672170578144923e-07, 5.336085289072462e-08,
                        2.668042644536231e-08, 1.3340213222681154e-08, 6.670106611340577e-09,
                        3.3350533056702886e-09, 1.6675266528351443e-09, 8.337633264175721e-10,
                        4.1688166320878607e-10, 2.0844083160439303e-10, 1.0422041580219652e-10,
                        5.211020790109826e-11, 2.605510395054913e-11, 1.3027551975274565e-11,
                        6.513775987637282e-12, 3.256887993818641e-12, 1.6284439969093206e-12,
                        8.142219984546603e-13, 4.0711099922733015e-13, 2.0355549961366507e-13,
                        1.0177774980683254e-13};		// 50 times

    float k = 0.6072529;                          // 旋转矩阵比例因子
    float direction = 0;
    float convention_tan = 1;                                   //  约定的tan的值(为2^(-i))

    for(int i = 0; i < 50; i++)
    {
        int sigma = (y < 0) ? 1 : -1;
        float pre_x = x;
        x -= sigma * convention_tan * y;
        y += sigma * convention_tan * pre_x;

        direction -= sigma * angle[i];
        convention_tan = convention_tan / 2;
    }

    *r = x * k;
    return direction;
}

float Displacement_angle(float x, float y, float* r)
{
    if(x >= 0 && y > 0)
        return arctan_90(x, y, r);
    else if(x < 0 && y > 0)
        return 180 - arctan_90(-x, y, r);
    else if(x >= 0 && y < 0)
        return 360 - arctan_90(x, -y, r);
    else if(x < 0 && y <= 0)
        return 180 + arctan_90(-x, -y, r);
    return 0;
}

float x_pwm, y_pwm;

float Wheel_control(float x, float y, float x_target, float y_target)            // y_speed = x_speed * tan(n); (n为目标角度)
{
    float Err_x, Err_y;
    //float dErr_x[2], dErr_y[2];
    float x_Kp, y_Kp;
    float x_Kd = 40, y_Kd = 40;
    Err_x = x_target - x;
    Err_y = y_target - y;
    if(abs(Err_x) >= 20)
    {
        x_Kp = 200;
    }
    else if(abs(Err_x) < 20 && abs(Err_x) > 8)
    {
        x_Kp = 270;
    }
    else if(abs(Err_x) <= 8)
    {
        x_Kp = 350;
    }
    if(abs(Err_y) >= 20)
    {
        y_Kp = 200;
    }
    else if(abs(Err_y) < 20 && abs(Err_y) > 8)
    {
        y_Kp = 270;
    }
    else if(abs(Err_y) <= 8)
    {
        y_Kp = 350;
    }
    x_pwm = x_Kp * Err_x;
    y_pwm = y_Kp * Err_y;

//    Err_x[1]=Err_x[0];
//    Err_x[0]= Err_x;
//    Err_y[1]=Err_y[0];
//    Err_y[0]= Err_y;

    if(y_pwm > 3000) y_pwm = 3000;
    if(y_pwm < -3000) y_pwm = -3000;
    if(x_pwm > 3000) x_pwm = 3000;
    if(x_pwm < -3000) x_pwm = -3000;

    return x_pwm;
		return y_pwm;
}

float x = 0.0f, y = 0.0f;                    // 存放x, y坐标
//float dt_encoder = 0.005;      // 编码器采样时间
//float diameter = 0.059;        // 车轮直径

void Self_position(int left_up, int left_down, int right_up, int right_down) //*
{
    y += (1.0f / distance_y) * ((float)(right_down + left_down + left_up + right_up) * nav_cos(yaw_angle) + (float)(right_down - left_down + left_up - right_up) * nav_sin(yaw_angle));
    x += (1.0f / distance_x) * ((float)(-right_down + left_down - left_up + right_up) * nav_cos(yaw_angle) - (float)(-right_down - left_up - left_down - right_up) * nav_sin(yaw_angle));
}


/* 角度环 */

float angle_error_sum;           
float yaw_error;
float angle_speed;

float Angle_PID(float expect_angle)
{
	 float Kp = 1.2f; float Ki = 0; float Kd = -0.2f;          
	 
	 static float angle_error[2];
	 yaw_error = expect_angle - yaw_angle;
	 angle_error_sum += yaw_error;
	
	 angle_error[1] = angle_error[0];
	 angle_error[0] = yaw_error;
	 
	 angle_speed = Kp * angle_error[0] + Kd * (angle_error[0] - angle_error[1]);
	 
	 if(angle_speed >30)angle_speed = 30;
	 if(angle_speed < -30)angle_speed = -30;
	return angle_speed;
}

float angle_encoder_speed;
float Angle_Encoder_PID(float expect_angle)
{
	 float Kp = 2000; float Ki = 0; float Kd = -0.2;          
	 static float angle_error[2];
	 yaw_error = expect_angle - yaw_angle;
	 angle_error_sum += yaw_error;
	
	 angle_error[1] = angle_error[0];
	 angle_error[0] = yaw_error;
	 
	 angle_encoder_speed = Kp * angle_error[0] + Ki * angle_error_sum + Kd * (angle_error[0] - angle_error[1]);
	 
//	 if(angle_speed >30)angle_speed = 30;
//	 if(angle_speed < -30)angle_speed = -30;
	angle_encoder_speed = (angle_encoder_speed > 15000) ? 15000 : angle_encoder_speed;
	angle_encoder_speed = (angle_encoder_speed < -15000) ? -15000 : angle_encoder_speed;
	return angle_encoder_speed;
}

/* 位置环 */
Position_PID position_x, position_y;
float actual_x = 0, actual_y = 0;

float x_position_PID(float x_cur, float x_tar)
{
	position_x.Kp = 30.0f, position_x.Kd = 0, position_x.Ki = 0;
	position_x.error[0] = x_tar - x_cur;
	actual_x = Position_PID_Calc(&position_x);
	return actual_x;
}

float y_position_PID(float y_cur, float y_tar)
{
	position_y.Kp = 30.0f, position_y.Kd = 0, position_y.Ki = 0;
	position_y.error[0] = y_tar - y_cur;
	actual_y = Position_PID_Calc(&position_y);
	return actual_y;
}


float L_up, L_down, R_up, R_down;
void wheel_output(float x_cur, float x_tar, float y_cur, float y_tar, float target_angle_speed)
{
	//
	float x_pos, y_pos;
		x_pos = x_position_PID(x_cur, x_tar);
		y_pos = y_position_PID(y_cur, y_tar);
		L_up = x_pos + y_pos, L_down = -x_pos + y_pos, R_up = -x_pos + y_pos, R_down = x_pos + y_pos;
	limit_pwm();
//	pid_L_up(&pid2, (L_up - target_angle_speed));
//	pid_L_down(&pid4, (L_down - target_angle_speed));
//	pid_R_up(&pid1, (R_up + target_angle_speed));
//	pid_R_down(&pid3, (R_down + target_angle_speed));
	if((60 - y) > 0.35 && (60  - x) > 0.35 && y > 0.2 && x > 0.2) {
		pid_L_up(&pid2, L_up);
		pid_L_down(&pid4, L_down);
		pid_R_up(&pid1, R_up);
		pid_R_down(&pid3, R_down);
	}
	else
	{
		pid_L_up(&pid2, (L_up - target_angle_speed));
		pid_L_down(&pid4, (L_down - target_angle_speed));
		pid_R_up(&pid1, (R_up + target_angle_speed));
		pid_R_down(&pid3, (R_down + target_angle_speed));
	}
//		pid_encoder(&pid1, 70, encd[0]);
//		pid_encoder(&pid2, 70, encd[1]);
//		pid_encoder(&pid3, 70, encd[2]);
//		pid_encoder(&pid4, 70, encd[3]);
	
}


//void gesture_PID(float angle_expect)
//{
//	
//}

//void Navigation_init(Navigation* p, float x, float y, float m, float n, float j, float k)
//{
//    x = p->x_offset, y = p->x_offset;
//    m = p->displacement, n = p->angle;
//    j = p->velocity, k = p->accelerate;
//}

//void Navigation_calc(float target_x, float current_x, float target_angle, float current_angle)
//{
//    if(target_x <= current_x)
//    {
//        //PWM给0
//    }
//
//
//}
//

//float nav_ang;
//float ange_calc(float x)   // 放角度计算函数
//{
//    // 角度计算函数，直接调逐飞的库
//    //mpuRead;
//    mpu6050_get_gyro();
//	nav_ang = mpu6050_gyro_transition(mpu6050_gyro_z);
//    return nav_ang;
//}

//float Current_angle_calc(int t)                    // 输入：积分上限(下限默认为0)，传出值为这段时间总共角度值
//{
//    int n = 1, h = t;
//    float fb, fa, value1, value2, precision, integral_sum, second_parameter;
//    fa = ange_calc(0), fb = ange_calc(t);
//    value1 = (fa + fb) * h / 2.0;
//    precision = 0.001 + 1;
//    // while(precision >= 0.001)
//    // {
//        integral_sum = 0.0;
//        for(int i = 0; i <= n - 1; i++)
//        {
//            second_parameter = 0 + (i + 0.5) * h;  // 这里0为积分下限
//            integral_sum += ange_calc(second_parameter);
//        }
//        value2 = (value1 + h * integral_sum) / 2;
//        precision = nav_fabs(value1 - value2);
//        value1 = value2;
//        n *= 2;
//        h /= 2.0;
//    // }
//    return value2;
//}

//float acc_calc(float x)               //放加速度计算函数
//{
//    // 加速度计算函数，调用逐飞的库
//    //READ;
//    //
//    return 2 * x;
//}

//float Current_v_calc(int t)            // 输入：积分上限(下限默认为0)，传出值为这段时间的速度
//{
//    int n = 1, h = t;
//    float fb, fa, value1, value2, precision, integral_sum, second_parameter;
//    fa = acc_calc(0), fb = acc_calc(t);
//    value1 = (fa + fb)* h / 2.0;
//    precision = 0.001 + 1;
//    while(precision >= 0.001)
//    {
//        integral_sum = 0.0;
//        for(int i = 0; i <= n - 1; i++)
//        {
//            second_parameter = 0 + (i + 0.5) * h;
//            integral_sum += ange_calc(second_parameter);
//        }
//        value2 = (value1 + h * integral_sum) / 2;
//        precision = nav_fabs(value1 - value2);
//        value1 = value2;
//        n *= 2;
//        h /= 2.0;
//    }
//    return value2;
//}
//float Current_x_calc(int t)                        // 输入：积分上限(下限默认为0)，传出值为这段时间的位移
//{
//    int n = 1, h = t;
//    float fb, fa, value1, value2, precision, integral_sum, second_parameter;
//    fa = Current_v_calc(0), fb = Current_v_calc(6);
//    value1 = (fa + fb)* h / 2.0;
//    precision = 0.001 + 1;
//    while(precision >= 0.001)
//    {
//        integral_sum = 0.0;
//        for(int i = 0; i <= n - 1; i++)
//        {
//            second_parameter = 0 + (i + 0.5) * h;
//            integral_sum += ange_calc(second_parameter);
//        }
//        value2 = (value1 + h * integral_sum) / 2;
//        precision = nav_fabs(value1 - value2);
//        value1 = value2;
//        n *= 2;
//        h /= 2.0;
//    }
//    return value2;
//}
//void Speed_plan()                               // S型速度规划
//{
//
//}
