/**
  ******************************************************************************
  * @file    comHandle.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-11-26
  * @brief   通信数据处理
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "comHandle.h"
#include "stm32f10x.h"
#include "usart.h"
#include "stdio.h"
#include "odometer.h"
#include "myPlanner.h"
#include "mapSaver.h"

/*Golbal data space ----------------------------------------------------------*/
#define UART_COM_STAT   	'<'				//串口通信开始字节
#define UART_COM_END			'>'				//串口通信结束字节


UartBufTypedef g_ComData;				//串口接收缓冲区


//串口字节流写入到缓冲区
void USART_RxToBuf(uint8_t data,USART_TypeDef* USARTx)
{
	uint8_t cur = 0;
	//将数据写入
	
	//上一次消息处理结束
	if(g_ComData.Over == 0)
	{
		//开始
		if(data == UART_COM_STAT )
		{
			//长度清零
			g_ComData.Len = 0;
		}
		//结束
		else if(data == UART_COM_END)
		{
			//接收结束
			g_ComData.Over = 1;
		}
		//数据
		else
		{
			//写入数据
			g_ComData.Buf[g_ComData.Len] = data;
			//移动光标
			g_ComData.Len = (g_ComData.Len + 1) % 1024;
		}
	}
}

//将静态路径写入flash
void PathWrite(uint8_t * cmd)
{
	//路径写入协议
	//<PXXX...XXX>
	// "X"点描述单元
	//01234 长度5
	//XXYY,
	
	uint16_t cur = 0; //描述点的光标
	
	uint16_t x = 0;	//x坐标
	uint16_t y = 0;	//y坐标
	
	uint16_t cur_buf = 1;//缓冲区读取光标
	
	//打印消息
	UsartSendData("\r\n");
	UsartSendData(cmd);
	UsartSendData("\r\n");
	
	//结束点与超出最大值跳出
	while(x != 99 && y != 99 && cur < 250)
	{
		//坐标计算
		x = (cmd[cur_buf + 0] - 0x30) * 10 + 
				(cmd[cur_buf + 1] - 0x30) * 1;
		
		y = (cmd[cur_buf + 2] - 0x30) * 10 + 
		    (cmd[cur_buf + 3] - 0x30) * 1;
		
		
		PosTypedef pos;
		
		pos.x = x;
		pos.y = y;
		
		//将坐标写入路径序列
		setPathNode(cur,pos);
		
		//移动节点光标
		cur++;
		
		//移动缓冲区读取光标
		cur_buf += 5;
		
		//打印坐标点信息
		uint8_t str[50];
		sprintf((char *)str,"P%2d,X=%2d,Y=%2d.\r\n",cur,pos.x,pos.y);
		UsartSendData(str);
	}
	
	//设置节点数量
	setPathCurNum(cur);
	//将配置写入flash
	writeStaticPathToFlash();
	
	UsartSendData("\r\n");
	UsartSendData("OK \r\n");
}

//数据接收处理
void USART3_RxBufAnalysis(UartBufTypedef * buf)
{
	//完整性被使能
	if(buf->Over == 1)
	{	
		//通信协议
		//开始 ‘<’ 结束 '>'
		//[0] 标识 [n] 数据
		switch(buf->Buf[0])
		{
			case 'P'://静态路径
			{
				PathWrite(buf->Buf);
				break;
			}
		}
		//清除未完成标志位
		buf->Over = 0;
	}
}

void comHandleInit(void)
{
	//配置接收函数
	USART_RxFuncConfig(USART_RxToBuf);
}

void comHandleLoop(void)
{
	//接收解析
	USART3_RxBufAnalysis(&g_ComData);
}











