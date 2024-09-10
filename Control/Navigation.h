#ifndef NAVIGATION_NAVIGATION_H
#define NAVIGATION_NAVIGATION_H

#include "zf_common_headfile.h"
#include "PID.h"

#define distance_x 5359.265540458f//5359.265540458f//5259.265540458f //5167.2283935f////250
#define distance_y 4558.05751553f//4958.05751553f//4858.05751553f //4777.84126984f

extern float pid1_flag, pid2_flag, pid3_flag, pid4_flag;

extern float tar_angle_speed;
extern float angle_encoder_speed;

//struct Navigation {
//    float x_offset;         // x偏移量
//    float y_offset;         // y偏移量
//    float displacement;     // 位移量
//    float angle;            // 角度量
//    float velocity;         // 速度
//    float accelerate;       // 加速度
//};

typedef struct
{
	float yaw ;
	float pich ;
	float roll ;
	float a_x;
	float a_y;

}mpu6050_typedef;

typedef struct
{
	float x ;
	float y ;
	float ang ;
	float speed;

}carSpdPsn_typedef;//车身姿态位置

extern int32 pwm[4];
extern float angle_speed;
extern uint8 open_flag;
extern float L_up, L_down, R_up, R_down;
extern float distance, x_tar;

float nav_fabs(float x);
void limit_pwm();


//void Navigation_init(Navigation* p, float x, float y, float m, float n, float j, float k);
float Current_angle_calc(int t);
float Current_v_calc(int t);
float Current_x_calc(int t);
void Navigation_calc(void);
float Angle_Encoder_PID(float expect_angle);

/* 角度逆解算函数
 * 输入：小车与目标的x坐标差，y坐标差，位移变量
 * 输出：小车当前与目标的角度，当前相差的位移(用传入的位移变量地址接收)
 */

extern float x, y;
extern float z_gyro, yaw1, yaw_angle;
extern int8 encoder_bit;
extern uint8 motor_flag;
float Displacement_angle(float x, float y, float* r);
void imu_963r_init(void);
void Self_position(int left_up, int left_down, int right_up, int right_down);
float Angle_PID(float expect_angle);
float x_position_PID(float x_cur, float x_tar);
float y_position_PID(float y_cur, float y_tar);
//void wheel_output(float x_cur, float x_tar, float y_cur, float y_tar, float target_angle_speed);


void Speed_plan();

#endif //NAVIGATION_NAVIGATION_H
