#include "Gyro.h"
#include "Filter.h"
#include "Navigation.h"

gyro_param_t Gyrooffset;

void Gyrooffset_init(void)         // Zero drift offset of gyro;
{
	uint16_t i;
	Gyrooffset.Zdata = 0;
	
	for(i = 0; i < 1000; i++) 
	{
		imu963ra_get_gyro();
		Gyrooffset.Zdata += imu963ra_gyro_z;
		rt_thread_delay(5);
	}
	
	Gyrooffset.Zdata /= 1000;
}

float z_gyro_pre = 0, z_gyro_last = 0, z_gyro_now = 0, z_gyro = 0, yaw1 = 0, yaw_angle = 0;
float dt_gyro = 0.005;

void Self_posture()                                  // should be in interruput
{
    z_gyro_pre = z_gyro_last;
    z_gyro_last = z_gyro;
		imu963ra_get_gyro();
    z_gyro_now = (float)imu963ra_gyro_z - Gyrooffset.Zdata;             
    z_gyro = LowPass_gyro(z_gyro_now, z_gyro_last, z_gyro_pre);
    yaw1 = (float)(z_gyro / 14.3f) * dt_gyro;
    if(nav_fabs(yaw1) < 0.01) yaw1 = 0;
    yaw_angle += yaw1;                                           
}


//#define delta_T     0.005f  //1ms����һ��
//#define M_PI        3.1415926f

//float z=0;

//float I_ex, I_ey, I_ez;  // ������

//quater_param_t Q_info = {1, 0, 0};  // ȫ����Ԫ��
//euler_param_t eulerAngle; //ŷ����

//icm_param_t icm_data;
//gyro_param_t GyroOffset;

//bool GyroOffset_init = 0;

//float param_Kp = 0.17f;   // ���ٶȼƵ��������ʱ������� 
//float param_Ki = 0.004f;   //�������������ʵĻ������� 0.004


//float fast_sqrt(float x) {
//    float halfx = 0.5f * x;
//    float y = x;
//    long i = *(long *) &y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float *) &i;
//    y = y * (1.5f - (halfx * y * y));
//    return y;
//}


//void gyroOffset_init(void)      /////////��������Ʈ��ʼ��
//{
//    GyroOffset.Xdata = 0;
//    GyroOffset.Ydata = 0;
//    GyroOffset.Zdata = 0;
//    for (uint16_t i = 0; i < 100; ++i) {
//        get_icm20602_gyro_spi();
//        get_icm20602_accdata_spi();
//        GyroOffset.Xdata += icm_gyro_x;
//        GyroOffset.Ydata += icm_gyro_y;
//        GyroOffset.Zdata += icm_gyro_z;
//        systick_delay_ms(2);
//    }

//    GyroOffset.Xdata /= 100;
//    GyroOffset.Ydata /= 100;
//    GyroOffset.Zdata /= 100;

//    GyroOffset_init = 1;
//}


//#define alpha           0.3f

////ת��Ϊʵ������ֵ
//void ICM_getValues() {
//    //һ�׵�ͨ�˲�����λg/s
//    icm_data.acc_x = (((float) icm_acc_x) * alpha) * 8 / 4096 + icm_data.acc_x * (1 - alpha);
//    icm_data.acc_y = (((float) icm_acc_y) * alpha) * 8 / 4096 + icm_data.acc_y * (1 - alpha);
//    icm_data.acc_z = (((float) icm_acc_z) * alpha) * 8 / 4096 + icm_data.acc_z * (1 - alpha);


//    //�����ǽǶ�ת����
//    icm_data.gyro_x = ((float) icm_gyro_x - GyroOffset.Xdata) * M_PI / 180 / 16.4f;
//    icm_data.gyro_y = ((float) icm_gyro_y - GyroOffset.Ydata) * M_PI / 180 / 16.4f;
//    icm_data.gyro_z = ((float) icm_gyro_z - GyroOffset.Zdata) * M_PI / 180 / 16.4f;
//	
//}


////�����˲�
//void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
//    float halfT = 0.5f * delta_T;
//    float vx, vy, vz;    //��ǰ�Ļ�������ϵ�ϵ�������λ����
//    float ex, ey, ez;    //��Ԫ������ֵ����ٶȼƲ���ֵ�����
//    float q0 = Q_info.q0;
//    float q1 = Q_info.q1;
//    float q2 = Q_info.q2;
//    float q3 = Q_info.q3;
//    float q0q0 = q0 * q0;
//    float q0q1 = q0 * q1;
//    float q0q2 = q0 * q2;
//    float q0q3 = q0 * q3;
//    float q1q1 = q1 * q1;
//    float q1q2 = q1 * q2;
//    float q1q3 = q1 * q3;
//    float q2q2 = q2 * q2;
//    float q2q3 = q2 * q3;
//    float q3q3 = q3 * q3;
//    // float delta_2 = 0;

//    //�Լ��ٶ����ݽ��й�һ�� �õ���λ���ٶ�
//    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
//    ax = ax * norm;
//    ay = ay * norm;
//    az = az * norm;

//    //���ݵ�ǰ��Ԫ������ֵ̬����������������������ںͼ��ټ�ʵ�ʲ��������ĸ������������жԱȣ��Ӷ�ʵ�ֶ�������̬������
//    vx = 2 * (q1q3 - q0q2);
//    vy = 2 * (q0q1 + q2q3);
//    vz = q0q0 - q1q1 - q2q2 + q3q3;
//    //vz = (q0*q0-0.5f+q3 * q3) * 2;

//    //�������������������ʵ�ʲ�����������������������֮�����
//    ex = ay * vz - az * vy;
//    ey = az * vx - ax * vz;
//    ez = ax * vy - ay * vx;

//    //�ò���������PI����������ƫ��
//    //ͨ������ param_Kp��param_Ki ����������
//    //���Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶȡ�
//    I_ex += halfT * ex;   // integral error scaled by Ki
//    I_ey += halfT * ey;
//    I_ez += halfT * ez;

//    gx = gx + param_Kp * ex + param_Ki * I_ex;
//    gy = gy + param_Kp * ey + param_Ki * I_ey;
//    gz = gz + param_Kp * ez + param_Ki * I_ez;


//    /*����������ɣ���������Ԫ��΢�ַ���*/


//    //��Ԫ��΢�ַ��̣�����halfTΪ�������ڵ�1/2��gx gy gzΪ�����ǽ��ٶȣ����¶�����֪��������ʹ����һ��������������Ԫ��΢�ַ���
//    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
//    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
//    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
//    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
//    //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
//    // ������Ԫ����    ��Ԫ��΢�ַ���  ��Ԫ�������㷨�����ױϿ���
//    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			
//    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT


//    // normalise quaternion
//    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//    Q_info.q0 = q0 * norm;
//    Q_info.q1 = q1 * norm;
//    Q_info.q2 = q2 * norm;
//    Q_info.q3 = q3 * norm;
//}


///*����Ԫ��ת����ŷ����*/
//void ICM_getEulerianAngles(void) {

//    //�ɼ�����������
//    get_icm20602_gyro_spi();
//    get_icm20602_accdata_spi();

//    ICM_getValues();
//    ICM_AHRSupdate(icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z, icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
//    float q0 = Q_info.q0;
//    float q1 = Q_info.q1;
//    float q2 = Q_info.q2;
//    float q3 = Q_info.q3;

//    //��Ԫ������ŷ����
//    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI; // yaw
//    //eulerAngle.yaw+=eulerAngle.yaw;
///*   ��̬����*/
//    if (eulerAngle.roll > 90 || eulerAngle.roll < -90) {
//        if (eulerAngle.pitch > 0) {
//            eulerAngle.pitch = 180 - eulerAngle.pitch;
//        }
//        if (eulerAngle.pitch < 0) {
//            eulerAngle.pitch = -(180 + eulerAngle.pitch);
//        }
//    }

//    if (eulerAngle.yaw > 360) {
//        eulerAngle.yaw -= 360;
//    } else if (eulerAngle.yaw < 0) {
//        eulerAngle.yaw += 360;
//    }
//}