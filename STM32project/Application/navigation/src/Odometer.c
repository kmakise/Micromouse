/**
  ******************************************************************************
  * @file    odometer.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-9-28
  * @brief   里程计计算
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "odometer.h"
#include "pidctrller.h"
#include "attitude.h"
#include "InfraredCalc.h"
#include "OccupyingGrid.h"
#include "parameter.h"

#include "drive.h"
#include "stdio.h"

/*Global Date Space ----------------------------------------------------------*/

//里程计参数								//46179
const int32_t STDGRIDETR = ODOMETER_STD_GRID_ETR_VAL;//标准栅格的编码器数

//平台的位置和姿态
PosTypedef MousePos = { 
	.x  = 0 ,
	.y  = 0 ,
	.th = 90,
};

uint8_t g_StdGridUpdateFlag = 0;
/*----------------------------------------------------------------------------*/

//地图坐标系与绝对方向的定义
//建立平面直角坐标系
/*	
		//坐标轴的定义
		
		y正方向
		|	
		|	  	     90 	
		|			180	 th  0
		|				   270
		|
		|
		|
		|起始点
		------------------------ X 正方向
		x = p * cos(th)
		y = p * sin(th)
		p^2 = x^2 + y^2
		tan(th) = y/x
		
		cos(0)   =  1    sin(0)   =  0
		cos(90)  =  0    sin(90)  =  1
		cos(180) = -1    sin(180) =  0
		cos(270) =  0    sin(270) = -1

*/
/*Golobal data space --------------------------------------------------------*/
//编码器累积
int32_t g_OdEtrSum = 0;
//里程辅助修正的最小偏移允许
float g_OdFixOffx = ODOMETER_STD_GRID_ETR_FIX;

//设定里程计辅助修正偏移最小允许值
void setOdometerFixOff(float num)
{
	g_OdFixOffx = num;
}
//读取平台位姿信息
PosTypedef getMousePos(void)
{
	return MousePos;
}

//平台姿态更新
void MouseDirUpdate(int th)
{
	int16_t angel;
	
	angel = ((MousePos.th + th) < 0) ? 360 + th : MousePos.th + th;
	
	angel = angel % 360;
	
	MousePos.th = angel;
}

//获得坐标点更新状态
uint8_t getPosState(void)
{
	uint8_t temp = g_StdGridUpdateFlag;
	
	//为了消除中断和后台的读取写入冲突
	if(g_StdGridUpdateFlag == 1 && temp == 1)
	{
		g_StdGridUpdateFlag = 0;
	}
	
	return temp;
}

//固定角度的余弦值
int8_t xcos(int16_t th)
{
	switch(th)
	{
		case 0   : return  1;
		case 90  : return  0;
		case 180 : return -1;
		case 270 : return  0;
		default:while(1);
	}
}

//固定角度的正弦值
int8_t ysin(int16_t th)
{
	switch(th)
	{
		case 0   : return  0;
		case 90  : return  1;
		case 180 : return  0;
		case 270 : return -1;
		default:while(1);
	}
}

//使用当前姿态修改里程位置
void odometerPosAdd(uint8_t num)
{
	MousePos.x += num * xcos(MousePos.th);
	MousePos.y += num * ysin(MousePos.th);
}
//设置里程计编码器
void setOdometerEtr(int32_t num)
{
	g_OdEtrSum = num;
}
//读取里程计编码器
int32_t readOdometerEtr(void)
{
	return g_OdEtrSum;
}

//坐标错误检查 在里程计坐标计算发生致命错误时复位
void odometerFatalErrorReset(void)
{
	//当写入坐标不合法时 整个地图记录发生无法回溯的致命错误
	if(MousePos.x > 15 || MousePos.x < 0 ||
		 MousePos.y > 15 || MousePos.y < 0)
	{
		//系统复位
		NVIC_SystemReset();
	}
}


//标注栅格累计里程计
void StandardGridCumulative(void)
{

	static uint8_t gridflag = 0;//更新地图标志位
	//etrsum += getEncoderVal(0) + getEncoderVal(1);
	//仅在直线模式有效 自转与调整不算做里程记录
	if(getAttitudeState() == ASStraight ||
		 getAttitudeState() == AFStraight)
	{
		//记录编码器增量
		g_OdEtrSum += getEncoderVal(0) + getEncoderVal(1);
		
		//扫描环境
		if(g_OdEtrSum >= (int)(STDGRIDETR * ODOMETER_ENVIR_SIDE_START) && 
			 g_OdEtrSum <= (int)(STDGRIDETR * ODOMETER_ENVIR_SIDE_END) &&
			 gridflag == 0)
		{
			GridActSave(0);
			gridflag = 1;
		}
		
		//达到标准栅格数量
		if(STDGRIDETR <= g_OdEtrSum)
		{
			//删除已记录数量
			g_OdEtrSum -= STDGRIDETR;
			//记录里程计
			MousePos.x += 1 * xcos(MousePos.th);
			MousePos.y += 1 * ysin(MousePos.th);
			//位置更新标志位
			g_StdGridUpdateFlag = 1;
			//更新地图
			GridActSave(1);
			MapActivityRecord();
			gridflag = 0;
		}
		//抵近障碍检测的里程计辅助修正
		else if(getBarrierAct() && ((g_OdEtrSum > (int)(STDGRIDETR* g_OdFixOffx))))
		{
			//累积清零
			g_OdEtrSum = 0;
			//记录里程计
			MousePos.x += 1 * xcos(MousePos.th);
			MousePos.y += 1 * ysin(MousePos.th);
			//位置更新标志位
			g_StdGridUpdateFlag = 1;
			//更新地图
			GridActSave(1);
			MapActivityRecord();
			gridflag = 0;
		}
	}
	//姿态记录由其他方法进行计算
	else
	{
		g_OdEtrSum = 0;
	}
	//错误检查
	odometerFatalErrorReset();
	
}

//里程计记录分频器 
void ETR_Divider(void)
{
	static uint16_t div = 0;
	div++;
	if(div >= 10)
	{
		div = 0;
		StandardGridCumulative();
	}
}

//里程计中断循环
void odometerLoop(void)
{
	ETR_Divider();
}










