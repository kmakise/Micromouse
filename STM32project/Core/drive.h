#ifndef DRIVE_H
#define DRIVE_H
#include "stm32f10x.h"
#include "rcc.h"
#include "nvic.h"
#include "dian_ji.h"
#include "infrared.h"
#include "ZLG7289.h"
#include "led.h"
#include "delay.h"
#include "drive.h"
#include "usart.h"
#include "time.h"


extern u16 ADC_MeasureBuf[6];     //buf[0]-buf[3]:红外接收二极U1-U4;  buf[4]: TEMP温度AD; buf[5]:RDOUT SUMJ 角速度AD
extern u16 cnt1,cnt2 ;                  //电机1，2 在10ms里的编码器计数   cnt1:'z'    cnt2:'y';  满载0xffff

//驱动
void DriveInit(void);                     


//**************延时函数相关
void delay_ms(u32 ms);                  //ms延时
void delay_us(u32 us);                  //us延时
//**************

//**************电机相关
void DianJi_TiaoSu(u8 wei,int su);       //电机pwm调速 
u8 Dian_Ji_Direction(u8 n);             //电机方向采集 n:'z':返回左边电机方向, 'y':返回右边电机方向;   0:前 1:后
//**************

//**************红外相关
void Infrared_Send_Control(u8 led);     //红外发射二极管控制
void Adc_Transition(void);              //adc的一次一组采样(更新buf);
//**************

//**************数码管相关
void Zlg7289_Write_Data(u8 instruct,u8 data);   //数码管调控
void LEDDisplay(int16_t dat1,int16_t dat2);
//**************

//**************串口相关
void UsartSendData(u8 * tmp); 						//串口字符串发送

//***************系统运行时间戳
uint32_t Sys_GetTick(void);

//*****************
#endif

