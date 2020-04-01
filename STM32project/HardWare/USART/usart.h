#ifndef USART_H
#define USART_H
#include "stm32f10x.h"

typedef struct
{
	uint8_t Buf[1024];	//串口接收缓冲区
	uint8_t Over;			//串口接收检查
	uint16_t Len;			//串口接收长度
}UartBufTypedef;




//串口初始化
void UsartInit(u32 boud);
//串口发送函数
void UsartSendData(u8 * tmp);
//串口接收函数配置
void USART_RxFuncConfig(void (* func)(uint8_t data,USART_TypeDef* USARTx));

#endif


