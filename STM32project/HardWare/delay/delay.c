#include "stm32f10x.h"
#include "delay.h"
#include "time.h"
/*************
延时函数初始化
*************/

void delay_ms(u32 ms)
{
//	u32 temp;
//	
//	SysTick->LOAD = 9000*ms;//重装载赋值  注意 9 : = 72M/8 ;
//	                        //这里系统时钟为24M 如果72M将3000改为 72/8;
//	SysTick->VAL = 0x00;//清空计数破器
//	SysTick->CTRL = 0x01;//使能并 外部时钟
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&(!(temp&(1<<16))));//等待数到0
//	SysTick->CTRL = 0x00;//关闭
//	SysTick->VAL = 0x00;//清空
	
	uint32_t lasttick = Sys_GetTick();
	while((Sys_GetTick() - lasttick) < ms);
	
}

void delay_us(u32 us)
{
//	u32 temp;
//	
//	//与ms一样
//	
//	SysTick->LOAD = 9*us;
//	SysTick->VAL = 0x00;
//	SysTick->CTRL = 0x01;
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&(!(temp&(1<<16))));
//	SysTick->CTRL = 0x00;
//	SysTick->VAL = 0x00;
	u16 i=0;  
   while(us--)
   {
      i=5;  //自己定义
      while(i--) ;    
   }
}	

