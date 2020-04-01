#ifndef TIME_H
#define TIME_H
#include "stm32f10x.h"


void Timer4Init(void);
uint32_t Sys_GetTick(void);
void Systick_init(void);


#endif

