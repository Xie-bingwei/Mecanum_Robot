#include "Filter.h"

float K1 = 0.02;       
float K2 = 0.08;        // ??????????????
float angle, angle2, angle_dot;   // angle: ??, angle_dot:???
const float dt_kalman = 0.005;     // ????????

float kalman_filter(kal_filter* k_flt, float input)
{
    /*????,3???*/
    k_flt->input = input;
    k_flt->K = (k_flt->Last_prediction_cov) / (k_flt->Last_prediction_cov + k_flt->Measure_noise_cov);
    k_flt->Optimal_output = k_flt->System_state_prediction_matrix + k_flt->K * (k_flt->input - k_flt->System_state_prediction_matrix);
    k_flt->Optimal_cov_matrix = (1 - k_flt->K) * (k_flt->Last_prediction_cov);

    /*????,2???*/
    k_flt->System_state_prediction_matrix = k_flt->Optimal_output;
    k_flt->Last_prediction_cov = k_flt->Optimal_cov_matrix + k_flt->Process_noise_cov;

    return k_flt->Optimal_output;
}

void Kalman_Filter(float Accel, float Gyro)
{
    const float Q_angle = 0.001, //???????
                Q_gyro = 0.003, //????????
                R_angle = 0.5; // ???????? ?????
    static float Pdot[4] =
            { 0, 0, 0, 0 };
    static float PP[2][2] =                               // ???????,??????P
            {
                    { 1, 0 },
                    { 0, 1 } };
    static const char C_0 = 1;
    static float Q_bias, Angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

    angle += (Gyro - Q_bias) * dt_kalman;		   //????
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-????????????

    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3] = Q_gyro;
    PP[0][0] += Pdot[0] * dt_kalman;   		   // Pk-??????????????
    PP[0][1] += Pdot[1] * dt_kalman;   		   // =?????????
    PP[1][0] += Pdot[2] * dt_kalman;
    PP[1][1] += Pdot[3] * dt_kalman;

    Angle_err = Accel - angle;			   //zk-????

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;		 		    //?????????
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle += K_0 * Angle_err;	 			//????(??????)
    Q_bias += K_1 * Angle_err;	 			//????
    angle_dot = Gyro - Q_bias;		 	    //???(????)???=???

    if(angle>360.0) angle-=360.0;
    if(angle<0) angle+=360.0;
}

void First_order_filter(float angle_m, float gyro_m)               // ???????????????(??????)
{
    angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * dt_kalman);
}

void Second_order_filter(float angle_m, float gyro_m)             // ????
{
    static float y = 0, angle2 = 0;
    float x1, x2;
    x1 = (angle_m - angle2) * (1 - K2) * (1 - K2);
    y += x1 * dt_kalman;
    x2 = y + 2 * (1 - K2) * (angle_m - angle2) + gyro_m;
    angle2 += x2 * dt_kalman;
}

int LowPass_encoder(int X_last,int X_new)                    // ?????????????????, ????????int??int16
{
    int new_value,add;

    add = (X_new - X_last) * 0.7f;
    new_value = add + X_last;

    return new_value;
}

double LowPass_gyro(float data_new, float data_last, float data_pre)
{
    return 0.5f * data_new + 0.3f * data_last + 0.2f * data_pre;
}

int LowPass_average(int data[], int length)                  // ????????int???uint16
{
    int add = 0;
    for(int i = 0; i < length; i++)
    {
        add += data[i];
    }
    return add / length;
}

// Butterworth low-pass filter
Butter_Parameter Butter_80HZ_Parameter_Acce={
        //200hz---80hz
        1,                  1.14298050254,    0.4128015980962,
        0.638945525159,    1.277891050318,    0.638945525159
};

Butter_Parameter Butter_60HZ_Parameter_Acce={
        //200hz---60hz
        1,                 0.3695273773512,   0.1958157126558,
        0.3913357725018,   0.7826715450035,   0.3913357725018
};

Butter_Parameter Butter_51HZ_Parameter_Acce={
        //200hz---51hz
        1,                0.03680751639284,   0.1718123812701,
        0.3021549744157,   0.6043099488315,   0.3021549744157,
};

Butter_Parameter Butter_30HZ_Parameter_Acce={
        //200hz---30hz
        1,                -0.7477891782585,   0.272214937925,
        0.1311064399166,   0.2622128798333,   0.1311064399166
};

Butter_Parameter Butter_20HZ_Parameter_Acce={
        //200hz---20hz
        1,                  -1.14298050254,   0.4128015980962,
        0.06745527388907,  0.1349105477781,   0.06745527388907
};

Butter_Parameter Butter_15HZ_Parameter_Acce={
        //200hz---15hz
        1,                 -1.348967745253,   0.5139818942197,
        0.04125353724172, 0.08250707448344,   0.04125353724172
};

Butter_Parameter Butter_10HZ_Parameter_Acce={
        //200hz---10hz
        1,                 -1.561018075801,   0.6413515380576,
        0.02008336556421, 0.04016673112842,   0.02008336556421
};

Butter_Parameter Butter_5HZ_Parameter_Acce={
        //200hz---5hz
        1,                  -1.778631777825,  0.8008026466657,
        0.005542717210281, 0.01108543442056,  0.005542717210281
};

Butter_Parameter Butter_2HZ_Parameter_Acce={
        //200hz---2hz
        1,                  -1.911197067426,  0.9149758348014,
        0.0009446918438402,0.00188938368768,  0.0009446918438402
};

float Input[3]={0};
float Output[3]={0};

float Butterworth_Gyro(float curr_inputer, Butter_Parameter *Parameter)     // ????????butterworth????
{

    Input[2]  = curr_inputer;


    Output[2] =  Parameter->b[0] * Input[2]
                 + Parameter->b[1] * Input[1]
                 + Parameter->b[2] * Input[0]
                 - Parameter->a[1] * Output[1]
                 - Parameter->a[2] * Output[0];


    Input[0]=Input[1];
    Input[1]=Input[2];


    Output[0]=Output[1];
    Output[1]=Output[2];
    return (Output[2]);
}