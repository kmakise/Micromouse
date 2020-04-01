/**
  ******************************************************************************
  * @file    mouseApp.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-10-7
  * @brief   移动平台逻辑应用主程序
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "mouseapp.h"
#include "drive.h"
#include "mouseFuncLib.h"
#include "ErgodicMap.h"
#include "myplanner.h"
#include "OccupyingGrid.h"
#include "MoveCtrl.h"
#include "InfraredCalc.h"
#include "navigator.h"
#include "attitude.h"
#include "mapSaver.h"
#include "parameter.h"
//C11key
//B12LED

/*Golbla data space ----------------------------------------------------------*/
MouseStateTypedef g_MAppSt 	= REDTEST; 	//电脑鼠状态机
uint32_t g_LEDFlashFreq 		= 100;				//LED闪烁频率 0是常亮 其他数值是变换变化间隔

/*func -----------------------------------------------------------------------*/
void moveSPTest(void)
{
	//while(1)
	mouseMoveSP(MR90);
	mouseMove(MStop);
	while(1);
}




/*key enven -------------------------------------------------------------------*/
//开始按键事件
void startkeyEvent(void)
{
	if((GPIOC->IDR & (0x01 << 11)) == 0)
	{
		while((GPIOB->IDR & (0x01 << 11)) == 0);
		

		//开始搜寻地图
		g_MAppSt = MAPCREAT;
		
		g_LEDFlashFreq = 0;
		
		//等待搜寻策略的选择
		while(getInfraredEventVal() == 0);
		//有效策略选择
		switch(getInfraredEventVal())
		{
			case 0x01:setSearchPlant(0);break;
			case 0x02:setSearchPlant(1);break;
			case 0x04:setSearchPlant(2);break;
			case 0x08:setSearchPlant(2);break;
			default:break;
		}
		
		g_LEDFlashFreq = 100;
		
		delay_ms(2000);
		
		//初始化栅格地图
		GridSetup();
		
//		moveSPTest();
//		TestFuncMoveFast();
//		TestFuncMoveFastTern();
//		ETR_Test();
//		while(1)
//		SendInfrared();
	}
}
//固定路径开始事件
void staticPathEvent(void)
{
	g_LEDFlashFreq = 0;
	if(getInfraredEventVal() != 0)
	{
		g_LEDFlashFreq = 100;
		delay_ms(2000);
		NavigatorStaticPath();
		while(1);
	}
}

//发送按键事件
void sendkeyEvent(void)
{
	if((GPIOC->IDR & (0x01 << 11)) == 0)
	{
		while((GPIOB->IDR & (0x01 << 11)) == 0);
		//SendMaoToUsart();
		delay_ms(2000);
//		moveSPTest();
		NavigatorFastII();
	}
}
	
//保存信息清除
void clearkeyEvent(void)
{
	delay_ms(100);
	if((GPIOC->IDR & (0x01 << 11)) == 0)
	{
		g_LEDFlashFreq = 0;//LED常亮
		delay_ms(500);
		while((GPIOB->IDR & (0x01 << 11)) == 0);
		//如果路径完整性使能标志位被置位
		if(getPathOverFlag()||getStaticPathFlag())
		{
			//清除路径
			delPathofFlash();
		}
		//发送SMC代码
		else
		{
			sendSMCCode();
		}
		delay_ms(1000);
	}
}

/*Setup and Loop func body ---------------------------------------------------*/	
void mouseApp_Setup(void)
{
	//读取flash到缓冲区
	readFlashToBuf();
	
	//按键清除地图
	clearkeyEvent();
	
	//路径完整使能
	if(getPathOverFlag())
	{
		//读取路径信息并将路径信息写入路径序列
		writePathSeqFromFlash();
		//配置里程计辅助修正模式
		setOdometerFixOff(ODOMETER_STD_GRID_FAST_FIX);
		//配置模式到冲刺等待
		g_MAppSt = WAIT;
	}
	
	//已配置静态路径
	if(getStaticPathFlag())
	{
		//读取路径信息并将路径信息写入路径序列
		writePathSeqFromFlash();
		//配置里程计辅助修正模式
		setOdometerFixOff(ODOMETER_STD_GRID_FAST_FIX);
		//配置模式到固定路径
		g_MAppSt = START;
	}
	
	UsartSendData("Sys_OK \r\n");
}

void mouseApp_Loop(void)
{
	switch(g_MAppSt)
	{
		case REDTEST://等待开始
		{
			g_LEDFlashFreq = 500;
			startkeyEvent();
			break;
		}
		case MAPCREAT://地图创建模式
		{
			
			g_LEDFlashFreq = 1000;//闪烁频率到500
			
			//建立地图
			MapCreatLoop();
			
			g_LEDFlashFreq = 500;
			//路径规划
			routeSearcher();
			//保存路径
			writePathToFlash();
			//配置里程计辅助修正模式
			setOdometerFixOff(ODOMETER_STD_GRID_FAST_FIX);
			//冲刺模式
			NavigatorFastII();
			g_MAppSt = WAIT;
			break;
		}
		case WAIT://等待模式
		{
			g_LEDFlashFreq = 100;
			sendkeyEvent();
			break;
		}
		case START://开始固定路径
		{
			//固定路径开始事件
			staticPathEvent();
			break;
		}
	}
}
//中断
void mouseAppInterrupt(void)
{
	LEDFlash(g_LEDFlashFreq);
}
	
	
	
	
	
	