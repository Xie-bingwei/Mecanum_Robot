#ifndef _rttfun_h
#define _rttfun_h

#define uint8_t      u8
#define uint32_t     u32
#define uint16_t     u16

/* 包含的头文件 */
#include "zf_common_headfile.h"
#include "zf_driver_timer.h"

/* 线程控制块 */
extern rt_thread_t mcWheelCtl_thread;  //麦伦控制线程
extern rt_thread_t camera_thread;      //相机线程
extern rt_thread_t Nav_cal_thread;         //导航线程
extern rt_thread_t Itrt_thread;        //交互线程

void mcWheelCtl_thread_entry(void* paramater);
void Itrt_thread_entry(void* paramater);
void Nav_cal_thread_entry(void* parameter) ;




#endif

/* end of file */