#ifndef __ENCODER_H
#define __ENCODER_H

#include "zf_common_headfile.h"
#define u32 uint32_t

extern int16 speed_L_up[2];
extern int16 speed_L_down[2];
extern int16 speed_R_up[2];
extern int16 speed_R_down[2]; 
extern int16 encd_sum[4];
extern int16 encd[4];

void Encoder_init(void);
void Encoder_get(void);
void Pulse_calc(void);
int32 speed_calc(int16 x);
// static int16 pulse1, pulse2, pulse3, pulse4;
uint32 read_speed(encoder_index_enum encoder_x);

#endif

