#ifndef DIAN_JI_H
#define DIAN_JI_H
#include "stm32f10x.h"

extern u16 cnt1,cnt2 ;            //电机1，2 在10ms里的编码器计数


typedef struct                    //TIM1PWM初始化结构体
{
	u16 psc;
	u16 arr;
	u16 ccr1;
	u16 ccr2;
	u16 ccr3;
	u16 ccr4;
	
}TIMX;
 
void Pwm_Init(TIMX *TIM);           //TIM1初始化
void Dian_Ji_Init(void);            //电机PWM输出初始化
void Dian_Ji_Read(void);            //电机编码器初始化
void DianJi_TiaoSu(u8 wei,int su);   //电机调速
void Dian_Ji_T(void);               //电机数据采集时间初始化(10ms)
u8 Dian_Ji_Direction(u8 n);         //电机方向采集

#endif

