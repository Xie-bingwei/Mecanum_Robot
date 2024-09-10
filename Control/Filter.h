#ifndef NAVIGATION_FILTER_H
#define NAVIGATION_FILTER_H

typedef struct Kalman_filter
{
    float Last_prediction_cov; /* (k|k-1)*/
    float System_state_prediction_matrix; /*列矩阵*/

    float Process_noise_cov;
    float Measure_noise_cov;

    float K;                                                  /*卡尔曼增益，列矩阵*/
    float Optimal_output; /*最优估计输出矩阵，列矩阵*/
    float Optimal_cov_matrix; /*最优估计协方差矩阵C(k|k)*/

    float input; /*量测值，即Z(k)*/
} kal_filter;

typedef struct
{
    const float a[3];
    const float b[3];
}Butter_Parameter;

extern float angle, angle2, angle_dot;
void Kalman_Filter(float Accel, float Gyro);
void First_order_filter(float angle_m, float gyro_m);
void Second_order_filter(float angle_m, float gyro_m);
int LowPass_encoder(int X_last,int X_new);
double LowPass_gyro(float data_new, float data_last, float data_pre);
int LowPass_average(int data[], int length);
float Butterworth_Gyro(float curr_inputer,Butter_Parameter *Parameter);
float kalman_filter(kal_filter* k_flt, float input);


#endif //NAVIGATION_FILTER_H
