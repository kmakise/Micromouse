#include "nvic.h"


void Tim4_Nvic_Init(void)
{
	
  NVIC->IP[TIM4_IRQn]|=0x5<<4;                    //抢占1 亚优先级1
	NVIC->ISER[TIM4_IRQn/32]|=(1<<TIM4_IRQn%32);    //使能中断
}

