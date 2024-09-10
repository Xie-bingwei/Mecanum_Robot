#ifndef _KEY_H
#define _KEY_H

#define GPIO_PIN_CONFIG         SPEED_100MHZ | DSE_R0 | PULLUP_47K | PULL_EN

#include "zf_common_headfile.h"

void keyinit(void);
uint8 key_c31(void);
uint8 key_c27(void);
uint8 key_c26(void);
uint8 key_c4(void);
uint8 key_num(void);


#endif 