#include "led.h"

void LedAnKey_Init(void)
{
	RCC->APB2ENR|=(1<<4);    //GPIOC Clock,Key Start
	RCC->APB2ENR|=(1<<3);    //GPIOB clock
	
	GPIOC->CRH&=~(0xf<<12);
	GPIOC->CRH|=(0x8<<12);
	GPIOC->ODR|=(1<<11);
	
	GPIOB->CRH&=~(0xf<<16);  //LED 
	GPIOB->CRH|=(0x3<<16);
	GPIOB->ODR|=(1<<12);
}













