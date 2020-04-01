#ifndef ADC_H
#define ADC_H

#include "stm32f10x.h"
#include "ZLG7289.h"

#define G_ODR_A   (GPIOA_BASE  + 0x0C)  
#define G_ODR_C   (GPIOC_BASE  + 0x0C)  
#define G_IDR_C   (GPIOC_BASE  + 0x08)  

#define Out_A(n)  BitBand(G_ODR_A,n)
#define Out_C(n)  BitBand(G_ODR_C,n)
#define In_C(n)   BitBand(G_IDR_C,n)

extern u16 ADC_MeasureBuf[6];

void Adc_Init(void);                    //adc1初始化
void Dma_Init(void);                    //DMA1初始化
void Infrared_Send_Init(void);          //红外发射二极管相关IO初始化
void Infrared_Send_Control(u8 led);     //红外发射二极管控制
//void Dma_Start(void);                   //DMA启动
void Adc_Transition(void);              //adc的一次一组采样(更新buf);
#endif

