/**
  ******************************************************************************
  * @file    OccupyingGrid.c.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-9-30
  * @brief   占据栅格地图读写与相关执行方法合集
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "OccupyingGrid.h"
#include "odometer.h"
#include "InfraredCalc.h"

#include "drive.h"
#include "stdio.h"

/*Global Date Space ----------------------------------------------------------*/
uint8_t g_GridMay[16][16];	//地图可活动性的记录
uint8_t g_GridAct = 0;

/*
	地图的存储方式
	
		可活动性
			低四位存储地图绝对坐标的可活动性信息
			地图绝对坐标系参考里程计的绝对坐标系定义
			
			L4B 					Active	Inactive
				|---B0	0°     1				0
				|---B1  90°    1        0
				|---B2	180°   1        0
				|---B3	270°	 1        0
		
			H4B
				|---B0	已访问
				|---B1 	保留
				|---B2	保留
				|---B3	保留
 */
 
 
//读取地图
uint8_t readMapGrid(uint8_t x,uint8_t y)
{
	return g_GridMay[x][y];
}
//写入地图
void writeMapGrid(uint8_t x,uint8_t y,uint8_t mp)
{
	g_GridMay[x][y] = mp;
}


//地图初始化
void MapInit(void)
{
	for(int i = 0;i < 16;i++)
	{
		for(int j = 0;j < 16;j++)
		{
			g_GridMay[i][j] = 0;
		}
	}
}

void sendMes(uint8_t w)
{
	PosTypedef mp = getMousePos();
	uint8_t str[20];
	
	//开始标志
	str[0] = '<';	
	
	//消息格式 xx,yy,ttt,w
	str[1] = mp.x / 10 % 10 + 0x30;
	str[2] = mp.x % 10 + 0x30;
	
	str[3] = ',';
	
	str[4] = mp.y / 10 % 10 + 0x30;
	str[5] = mp.y % 10 + 0x30;
	
	str[6] = ',';
	
	//姿态
	str[7] = mp.th / 100 % 10 + 0x30;
	str[8] = mp.th / 10 % 10 + 0x30;
	str[9] = mp.th % 10 + 0x30;
	
	str[10] = ',';
	
	//障碍物
	str[11] = w;
	
	//结束
	str[12] = '>';
	
	str[13] = 0;
	str[14] = 0;
	
	UsartSendData(str);
	UsartSendData(str);
}


//保存地图活动性信息
void GridActSave(uint8_t num)
{
	if(num == 0)
	{
		g_GridAct = getActDirState() & 0x05;
	}
	else
	{
		g_GridAct |= getActDirState() & 0x02;
	}
}



//栅格地图的记录
void MapActivityRecord(void)
{
	PosTypedef mp = getMousePos();
	
	//当前坐标未记录
//	if(g_GridMay[mp.x][mp.y] == 0)
//	{
	
	
	//读取可活动性信息
	//返回状态用而进制表示三个方向的可活动性
	//0000 0LFR L左侧 F前方 R右侧
	uint8_t activity = g_GridAct;
	
	//分离各个角度
	uint8_t m0   = (((~activity) & 0x02) >> 1);
	uint8_t m90  = ((activity & 0x04) >> 2);
	uint8_t m180 = 0x01;
	uint8_t m270 = (activity & 0x01);
	
	//使能已访问标志位
	g_GridMay[mp.x][mp.y] |= 0x10;
	
	//通过平台姿态进行绝对坐标的偏移
	switch(mp.th)
	{
		case 0:
		{
			//0°不进行变换
			g_GridMay[mp.x][mp.y] |= 0x01 &  m0				 ;//0-0
			g_GridMay[mp.x][mp.y] |= 0x02 & (m90  << 1);//90-90
			g_GridMay[mp.x][mp.y] |= 0x04 & (m180 << 2);//180-180
			g_GridMay[mp.x][mp.y] |= 0x08 & (m270 << 3);//270-270
			break;
		}
		case 90:
		{
			//变换方式 0-90 90-180 270-0 180-270
			g_GridMay[mp.x][mp.y] |= 0x01 & m270			 ;//0-270
			g_GridMay[mp.x][mp.y] |= 0x02 & (m0   << 1);//90-0
			g_GridMay[mp.x][mp.y] |= 0x04 & (m90  << 2);//180-90
			g_GridMay[mp.x][mp.y] |= 0x08 & (m180 << 3);//270-180
			break;
		}
		case 180:
		{
			//变换方式 0-180 90-270 270-90 180-0
			g_GridMay[mp.x][mp.y] |= 0x01 & m180		   ;//0-180
			g_GridMay[mp.x][mp.y] |= 0x02 & (m270 << 1);//90-270
			g_GridMay[mp.x][mp.y] |= 0x04 & (m0   << 2);//180-0
			g_GridMay[mp.x][mp.y] |= 0x08 & (m90	<< 3);//270-90
			break;
		}
		case 270:
		{
			//变换方式 0-270 90-0  270-180 180-90
			g_GridMay[mp.x][mp.y] |= 0x01 & m90				 ;//0-90
			g_GridMay[mp.x][mp.y] |= 0x02 & (m180 << 1);//90-180
			g_GridMay[mp.x][mp.y] |= 0x04 & (m270 << 2);//180-270
			g_GridMay[mp.x][mp.y] |= 0x08 & (m0   << 3);//270-0
			break;
		}
	}
//	}
	
	//上传位置信息
	if(g_GridMay[mp.x][mp.y] != 0)
	{
		sendMes(g_GridMay[mp.x][mp.y]);
	}
}

void GridSetup(void)
{
	delay_ms(50);
	//保存初始环境
	g_GridAct = getActDirState();
	MapInit();
	MapActivityRecord();
}

void GridLoop(void)
{
	//MapActivityRecord();
}





