#include "time.h"

#include "process.h"

void Timer4Init()
{
	RCC->APB1ENR|= 0x4;        //开tim4时钟
	RCC->APB2ENR|=(1<<3);    //GPIOB clock
	
	GPIOB->CRH&=~(0xf<<0);
	GPIOB->CRH|=(0xb<<0); //PB8推挽复用输出50M
	
	TIM4->SMCR&=~0x7;     //选择内部时钟
	TIM4->PSC=0;       //预分频
	TIM4->ARR=3600 - 1;       //重装载值
	TIM4->CCR3=0;       //比较值
	TIM4->CR1&=~(1<<4);   //向上计数
  TIM4->CCMR2|=(0x6<<4); //OC3为PWM1模式
	TIM4->CCMR2&=~(0x3<<0); //OC3为输出
	TIM4->CCER&=~(1<<9);    //OC3为高电平有效
  TIM4->CR1|=(1<<7);      //允许预装载
	TIM4->CCMR2|=(1<<3);   //OC3预装载
	TIM4->EGR|=(0x1);     //计数器初始化
	TIM4->CCER|=(0x1<<8);    //oc使能
	TIM4->CR1|=1;      //使能计数器
	
	TIM3->CR1 |= 1;   
	TIM2->CR1 |= 1;             //使能定时器
}

#define SYS_CLK   72000000

void Systick_init(void)
{
	 if (SysTick_Config(SYS_CLK/1000))     //1ms
	{        
			while (1);
	}
	NVIC_SetPriority(SysTick_IRQn, 0x04);
}


uint32_t g_SysTick = 0;

uint32_t Sys_GetTick(void)
{
	return g_SysTick;
}


void SysTick_Handler()
{
	MainInterrupt();
	g_SysTick++;
}
