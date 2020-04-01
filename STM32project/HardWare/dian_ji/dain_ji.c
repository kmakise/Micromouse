#include "dian_ji.h"
#include "ZLG7289.h"

u16 cnt1 = 0,cnt2 = 0;        //46 9 25mm


void Pwm_Init(TIMX *TIM)
{
	RCC->APB2ENR |= (1<<2);     //开GPIOA时钟
	RCC->APB2ENR |= (1<<11);    //开TIM1时钟	
	GPIOA->CRH &= ~(0xffff);
	
	GPIOA->CRH |= (0xbbbb);     //PA8 9 10 11推挽复用输出50M
	
	TIM1->SMCR &= ~0x7;         //选择内部时钟
	
	TIM1->PSC = TIM->psc;       //预分频
	TIM1->ARR = TIM->arr;       //重装载值
	
	TIM1->CCR1 = TIM->ccr1;     //比较值  
	TIM1->CCR2 = TIM->ccr2;    
	TIM1->CCR3 = TIM->ccr3;
	TIM1->CCR4 = TIM->ccr4;
	
	TIM1->CR1 &= ~(1<<4);       //向上计数
	TIM1->CR1|=(1<<7);          //允许预装载

	
	TIM1->CCMR1|=(0x6<<4);      //OC1为PWM1模式
	TIM1->CCMR1&=~(0x3<<0);     //OC1为输出
	TIM1->CCER &= ~(1<<1);      //OC1为高电平有效
	
  TIM1->CCMR1 |= (0x6<<12);   //OC2为PWM1模式
	TIM1->CCMR1&=~(0x3<<8);     //OC2为输出
	TIM1->CCER &= ~(1<<5);      //OC2为高电平有效
	
	TIM1->CCMR2|=(0x6<<4);      //OC3为PWM1模式
	TIM1->CCMR2&=~(0x3<<0);     //OC3为输出
	TIM1->CCER &= ~(1<<9);      //OC3为高电平有效
	
	TIM1->CCMR2|=(0x6<<12);     //OC4为PWM1模式
	TIM1->CCMR1&=~(0x3<<8);     //OC4为输出
	TIM1->CCER &= ~(1<<13);     //OC4为高电平有效
	
	TIM1->CCMR1|=(1<<11);       //OC2预装载使能
	TIM1->CCMR1|=(1<<3);        //OC1预装载使能
	TIM1->CCMR2|=(1<<11);       //OC4预装载使能
	TIM1->CCMR2|=(1<<3);        //OC3预装载使能
	
	TIM1->EGR|=(0x1);           //计数器初始化
	
	
  TIM1->CCER|=(0x1<<4);      //oc2使能
	TIM1->CCER|=(0x1<<0);      //oc1使能
	TIM1->CCER|=(0x1<<8);      //oc3使能
	TIM1->CCER|=(0x1<<12);     //oc4使能
	
	TIM1->CR1|=1;              //使能计数器	
	TIM1->BDTR|=(1<<15);       //禁止刹车，死区，开OC通道
}

void Dian_Ji_Init(void)
{
	TIMX TIM;
	
	TIM.arr = 999;
	TIM.psc = 8;
	TIM.ccr1 = 0;
	TIM.ccr2 = 0;
	TIM.ccr3 = 0;
	TIM.ccr4 = 0;
	
	Pwm_Init(&TIM);
}


void Dian_Ji_Read()
{
	
	RCC->APB2ENR |= (1<<2);
	
	GPIOA->CRL &= 0x00ffff00;
	GPIOA->CRL |= 0x44000044;     //浮空输入
	
	RCC->APB1ENR |= 0x1;          //开TIM2 3时钟
	RCC->APB1ENR |= 0x2;
	
	TIM3->SMCR &= ~0x7;
	TIM2->SMCR &= ~0x7;         //选择内部时钟
	
	TIM3->CR1 &= ~(1<<4);
	TIM2->CR1 &= ~(1<<4);       //向上计数
	TIM3->PSC =  0;            
	TIM2->PSC =  0;             //预分频
	TIM3->ARR = 0xffff;   
	TIM2->ARR = 0xffff;         //重装载值
	TIM3->SMCR |= 0x2;      
	TIM2->SMCR |= 0x2;          //编码器模式2
	
	TIM3->CCMR1 |= 0x1;         
	TIM2->CCMR1 |= 0x1;         //IC1 -> TI1
	TIM3->CCMR1 |= (0x1<<8);    
	TIM2->CCMR1 |= (0x1<<8);    //IC2 -> TI2
	TIM3->CCMR1 |= (0x3<<4);
	TIM2->CCMR1 |= (0x3<<4);
	TIM3->CCMR1 |= (0x3<<12); 
	TIM2->CCMR1 |= (0x3<<12);   //通道一二滤波八次
	
	TIM3->CCER |=  (1<<1);      //TIM3 IC1反向
	
	TIM2->CCER &= ~(1<<1);
	TIM3->CCER &= ~(1<<5); 
	TIM2->CCER &= ~(1<<5);       //通道1 2不反向
	
	TIM3->CR1|=(1<<7);  
	TIM2->CR1|=(1<<7);          //预装载
	
	TIM2->SR = 0x0000;
	//TIM3->CR1 |= 1;   
	//TIM2->CR1 |= 1;             //使能定时器
}

void DianJi_TiaoSu(u8 wei,int su)
{
	switch(wei)
	{
		case 0x03:if(su>=0)
		           { 
			           TIM1->CCR1 = 0;TIM1->CCR2 = su;break;		
							 }
							else
							{
								 TIM1->CCR2 = 0;TIM1->CCR1 = -su;break;
							}
		case 0x0a:if(su>=0)
		           {
								TIM1->CCR3 = 0;TIM1->CCR4 = su;break;
							}
							else
							{
								TIM1->CCR4 = 0;TIM1->CCR3 = -su;break;
							}
		case 0x01:TIM1->CCR1 = TIM1->CCR2 = 0;break;
		case 0x11:TIM1->CCR1 = TIM1->CCR2 = 1000;break;
		case 0x04:TIM1->CCR3 = TIM1->CCR4 = 0;break;
		case 0x14:TIM1->CCR3 = TIM1->CCR4 = 1000;break;
		default:break;
	}
}



u8 Dian_Ji_Direction(u8 n)
{
	if(n == 'z')
	{
		if( (TIM2->CR1 & 0x10)!= 0)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else if(n == 'y')
	{
		if( (TIM3->CR1& 0x10)!= 0)
		{
			return 0x01;
		}
		else
		{
			return 0x00;
		}
	}
	else
	{
		return 12;
	}
}

void getEtrVal(void)
{
	 TIM3->CR1 &= ~0x1;   
	 TIM2->CR1 &= ~0x1;          //失能定时器

	 cnt1 = TIM2->CNT;           //采集数据
	 cnt2 = TIM3->CNT;
	 TIM2->CNT = 0;
	 TIM3->CNT = 0;
	 
	 TIM4->CR1 |= 1;            //开tim4	

	 TIM3->CR1 |= 1;   
	 TIM2->CR1 |= 1;            //使能定时器
}

