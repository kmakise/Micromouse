#include "usart.h"

//设置NVIC分组
//NVIC_Group:NVIC分组 0~4 总共5组 		   
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//取后三位
	temp1<<=8;
	temp=SCB->AIRCR;  //读取先前的设置
	temp&=0X0000F8FF; //清空先前分组
	temp|=0X05FA0000; //写入钥匙
	temp|=temp1;	   
	SCB->AIRCR=temp;  //设置分组	    	  				   
}
//设置NVIC 
//NVIC_PreemptionPriority:抢占优先级
//NVIC_SubPriority       :响应优先级
//NVIC_Channel           :中断编号
//NVIC_Group             :中断分组 0~4
//注意优先级不能超过设定的组的范围!否则会有意想不到的错误
//组划分:
//组0:0位抢占优先级,4位响应优先级
//组1:1位抢占优先级,3位响应优先级
//组2:2位抢占优先级,2位响应优先级
//组3:3位抢占优先级,1位响应优先级
//组4:4位抢占优先级,0位响应优先级
//NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先	   
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
	u32 temp;	
	//MY_NVIC_PriorityGroupConfig(NVIC_Group);//设置分组
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;								//取低四位  
	NVIC->ISER[NVIC_Channel/32]|=(1<<NVIC_Channel%32);//使能中断位(要清除的话,相反操作就OK) 
	NVIC->IP[NVIC_Channel]|=temp<<4;		//设置响应优先级和抢断优先级   	    	  				   
} 
void UsartInit(u32 baud)
{
  float a = 0;
	int b = 0,c = 0;
//	u32 temp = 0;
	
	RCC->APB1ENR |= 1<<18;      //USART3时钟使能													
	
	GPIOB->CRH &= 0xffff00ff;   //B10,B11清零
	GPIOB->CRH |= 0x00004b00;   //B10复用推挽输出,B11浮空输入
	RCC->APB1RSTR |= (1<<18);   //复位USART3
	RCC->APB1RSTR &= ~(1<<18);  //USART复位结束
	USART3->CR1 &= ~(1<<12);    //一个起始位，8个数据位
	USART3->CR2 &= ~(3<<12);    //1个停止位
  //USART1->CR1|=(1<<10);     //开校验 偶校验
	USART1->CR1|=0X200C;  //1位停止,无校验位.
  a=36000000.0/(16*baud);     //计算波特率
	b=a;                        //取出整数
	c=((a-b)*16);               //取出小数
	c=(b<<4)+c;                 //合并
	USART3->BRR|=c;             //波特率值给BRR;
	USART3->CR1|=(1<<3);        //发送使能
  USART3->CR1|=(1<<2);        //接收使能
	USART3->CR1|=(1<<13);       //USART使能
	
	//使能接收中断 
	USART3->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,0,USART3_IRQn,2);//组2
	
//	temp = USART3->SR;
}

//串口发送
void UsartSendData(u8 * tmp)
{
	while(*tmp)
	{
		USART3->DR = *tmp++;
		while(!(USART3->SR&(1<<6)));  //等待发送完成
	}
}

//串口接受函数
void (* g_usartRxFunc)(uint8_t data,USART_TypeDef* USARTx);

//串口接收函数配置
void USART_RxFuncConfig(void (* func)(uint8_t data,USART_TypeDef* USARTx))
{
		g_usartRxFunc = func;
}


//串口接收中断
void USART3_IRQHandler(void)
{
	uint8_t bt;
	if(USART3->SR&(1<<5))
	{	 
		bt = (USART3->DR&(uint8_t)0x00FF);
		g_usartRxFunc(bt,USART3);
	}
}

