#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "stm32f10x.h"

//地图坐标系与绝对方向的定义
//建立平面直角坐标系
/*	
		//坐标轴的定义
		
		y正方向
		|	
		|	th	      90 	
		|			180	 姿态  0
		|					 270
		|
		|
		|
		|起始点
		------------------------ X 正方向
*/




typedef struct
{
	int x;
	int y;
	int th;
}PosTypedef;//位置姿态类型定义


//设定里程计辅助修正偏移最小允许值
void setOdometerFixOff(float num);
//设定编码器累计值
void setOdometerEtr(int32_t num);
//读取编码器累计值
int32_t readOdometerEtr(void);
//使用当前姿态修改里程位置
void odometerPosAdd(uint8_t num);

//读取平台位姿信息
PosTypedef getMousePos(void);
//平台姿态更新
void MouseDirUpdate(int th);
//获得坐标点更新状态
uint8_t getPosState(void);
//里程计记录循环
void odometerLoop(void);

#endif /*__ODOMETER_H*/
