#ifndef __GYRO_H
#define __GYRO_H

#include "zf_common_headfile.h"

extern float yaw_angle;
extern float z_gyro_now;
extern float Z_gyro;

typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
	  float X_magdata;
	  float Y_magdata;
	  float Z_magdata;
} gyro_param_t;

extern float yaw;
extern float yaw_add;
extern float dt_gyro;

void Self_posture();
void Gyrooffset_init(void);

#endif