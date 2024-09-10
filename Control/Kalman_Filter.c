#include "Kalman.h"

float K1 = 0.02;
float angle, angle_dot;

float kalman_filter(kal_filter* k_flt, float input)
{
    /*量测更新，3组方程*/
    k_flt->input = input;
    k_flt->K = (k_flt->Last_prediction_cov) / (k_flt->Last_prediction_cov + k_flt->Measure_noise_cov);
    k_flt->Optimal_output = k_flt->System_state_prediction_matrix + k_flt->K * (k_flt->input - k_flt->System_state_prediction_matrix);
    k_flt->Optimal_cov_matrix = (1 - k_flt->K) * (k_flt->Last_prediction_cov);

    /*时间更新，2组方程*/
    k_flt->System_state_prediction_matrix = k_flt->Optimal_output;
    k_flt->Last_prediction_cov = k_flt->Optimal_cov_matrix + k_flt->Process_noise_cov;

    return k_flt->Optimal_output;
}

void Kalman_Filter(float Accel, float Gyro)
{
    const float Q_angle = 0.001, // 过程噪声的协方差
                Q_gyro = 0.003, //0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
                R_angle = 0.5, // 测量噪声的协方差 既测量偏差
                dt = 0.005; // 卡尔曼滤波器采样时间(5ms)
    static float Pdot[4] =
            { 0, 0, 0, 0 };
    static float PP[2][2] =
            {
                    { 1, 0 },
                    { 0, 1 } };
    static const char C_0 = 1;
    static float Q_bias, Angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

    angle += (Gyro - Q_bias) * dt;		   //先验估计
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3] = Q_gyro;
    PP[0][0] += Pdot[0] * dt;   		   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   		   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle;			   //zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;		 		    //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle += K_0 * Angle_err;	 			//后验估计(最终融合角度)
    Q_bias += K_1 * Angle_err;	 			//后验估计
    angle_dot = Gyro - Q_bias;		 	//输出值(后验估计)的微分=角速度

    if(angle>360.0) angle-=360.0;
    if(angle<0) angle+=360.0;
}

void First_order_filter(float angle_m, float gyro_m)               // 对加速度和角度进行的一阶滤波(同样是低通的)
{
    angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * 0.005);
}

int Lowpass(int X_last,int X_new)                    // 移植的时候记得给编码器进行低通滤波, 把这个函数所有的int改为int16
{
    int new_value,add;

    add = (X_new - X_last)*0.7f;
    new_value = add + X_last;

    return new_value;
}
