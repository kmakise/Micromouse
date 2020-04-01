/**
  ******************************************************************************
  * @file    navigator.c
  * @author  张10
  * @version V1.0.0
  * @date    2019-10-13
  * @brief   移动导航与路径转义相关程序 通过导航点序列控制平台移动
  ******************************************************************************
  */
	
/*include file ---------------------------------------------------------------*/
#include "navigator.h"
#include "movectrl.h"
#include "odometer.h"
#include "myPlanner.h"
#include "attitude.h"
#include "parameter.h"
#include "InfraredCalc.h"

/*Golbal Data Space ----------------------------------------------------------*/


/*=================================移动导航测试版==============================*/

//按照引导方向移动
void moveNavigator(uint8_t dir,PosTypedef mp,void (* func)())
{
	//调整姿态
	switch(dir)
	{
		case 0x01:
			switch (mp.th)
			{
				case 90  :func(MR90);break;
				case 180 :func(ML180);break;
				case 270 :func(ML90);break;
			}break;
		case 0x02:
			switch (mp.th)
			{
				case 0   :func(ML90);break;
				case 180 :func(MR90);break;
				case 270 :func(ML180);break;
			}break;
		case 0x04:
			switch (mp.th)
			{
				case 0   :func(ML180);break;
				case 90  :func(ML90);break;
				case 270 :func(MR90);break;
			}break;
		case 0x08:
			switch (mp.th)
			{
				case 0   :func(MR90);break;
				case 90  :func(ML180);break;
				case 180 :func(ML90);break;
			}break;
	}

}

//通过下一个目标点计算方向
uint8_t navPosToDir(PosTypedef mp,PosTypedef nmp)
{
	if(mp.x > nmp.x)return 0x04;
	if(mp.y > nmp.y)return 0x08;
	if(mp.x < nmp.x)return 0x01;
	if(mp.y < nmp.y)return 0x02;
	return 0;
}

//移动导航测试版
void NavigatorTest(void)
{
	uint8_t cursor = 0;
	
	
	//前往终点
	while(getMousePos().x != getTargetPos().x ||
				getMousePos().y != getTargetPos().y)
	{
		//获得接下来的活动方向
		PosTypedef mp = getMousePos();
		PosTypedef nmp;
		
		nmp.x = getRoute(cursor).x;
		nmp.y = getRoute(cursor).y;
		
		uint8_t dir = navPosToDir(mp,nmp);
		
		//调整
		moveNavigator(dir,mp,mouseMoveFastI);
		//前进
		mouseMoveFastI(MForward);
		//等待抵达
		while(getMousePos().x != getRoute(cursor).x ||
		      getMousePos().y != getRoute(cursor).y);
		
		cursor++;
	}
	
	
	//返回起点
	cursor -= 2;
	while(getMousePos().x != getRoute(0).x ||
				getMousePos().y != getRoute(0).y)
	{
		//获得接下来的活动方向
		PosTypedef mp = getMousePos();
		PosTypedef nmp;
		
		nmp.x = getRoute(cursor).x;
		nmp.y = getRoute(cursor).y;
		
		
		uint8_t dir = navPosToDir(mp,nmp);
		
		//运动
		moveNavigator(dir,mp,mouseMoveFastI);
		//前进
		mouseMoveFastI(MForward);
		//等待抵达
		while(getMousePos().x != getRoute(cursor).x ||
		      getMousePos().y != getRoute(cursor).y);
		
		cursor--;
	}
	
	
	//获得接下来的活动方向
	PosTypedef mp = getMousePos();
	PosTypedef nmp;
	
	nmp.x = 0;
	nmp.y = 0;
	
	uint8_t dir = navPosToDir(mp,nmp);
	
	//运动
	moveNavigator(dir,mp,mouseMoveFastI);
	//前进
	mouseMoveFastI(MForward);
	//等待抵达
	while(getMousePos().x != 0||
				getMousePos().y != 0);
	
	mouseMove(MStop);
	mouseMove(ML180);
	mouseMove(MStop);
}
//移动导航静态路径
void NavigatorStaticPath(void)
{
	uint8_t cursor = 0;
	
	//前往终点
	while(getRoute(cursor).x != 99 ||
				getRoute(cursor).y != 99)
	{
		//获得接下来的活动方向
		PosTypedef mp = getMousePos();
		PosTypedef nmp;
		
		nmp.x = getRoute(cursor).x;
		nmp.y = getRoute(cursor).y;
		
		uint8_t dir = navPosToDir(mp,nmp);
		
		//调整
		moveNavigator(dir,mp,mouseMoveFastI);
		//前进
		mouseMoveFastI(MForward);
		//等待抵达
		while(getMousePos().x != getRoute(cursor).x ||
		      getMousePos().y != getRoute(cursor).y);
		//移动光标
		cursor++;
	}
	mouseMove(MStop);
}
/*================================移动导航基础版I==============================*/

//配置高速直线控制器
void setLineFastI(uint8_t dis)
{
	setFastStMoveGridNum(dis);
	AttitudeStateSet(AFStraight);
}
//获得两点之间的距离
uint8_t getDisGrid(PosTypedef mp,PosTypedef nmp)
{
	uint8_t dis = 0;
	
	//X坐标相同
	if(mp.x == nmp.x)
	{
		if(mp.y >nmp.y)
		{
			dis = mp.y - nmp.y;
		}
		else if(mp.y < nmp.y)
		{
			dis = nmp.y - mp.y;
		}
	}
	
	//y坐标相同
	else if(mp.y == nmp.y)
	{
		if(mp.x > nmp.x)
		{
			dis = mp.x - nmp.x;
		}
		else if(mp.x < nmp.x)
		{
			dis = nmp.x - mp.x;
		}
	}
	return dis;
}

//移动导航加速第一版本
void NavigatorFastI(void)
{
	uint8_t cursor = 0;
	
	TIM4->CCR3 = 600;
	
	//前往终点
	while(getMousePos().x != getTargetPos().x ||
				getMousePos().y != getTargetPos().y)
	{
		//获得接下来的活动方向
		PosTypedef mp = getMousePos();
		PosTypedef nmp;
		
		nmp.x = getRoute(cursor).x;
		nmp.y = getRoute(cursor).y;
		
		uint8_t dir = navPosToDir(mp,nmp);
		
		//姿态调整
		moveNavigator(dir,mp,mouseMoveFastI);
		//直线运动
		setLineFastI(getDisGrid(mp,nmp));
		
		//等待抵达
		while(getMousePos().x != getRoute(cursor).x ||
		      getMousePos().y != getRoute(cursor).y);
		
		cursor++;
	}
	
	
	//返回起点
	cursor -= 2;
	while(getMousePos().x != getRoute(0).x ||
				getMousePos().y != getRoute(0).y)
	{
		//获得接下来的活动方向
		PosTypedef mp = getMousePos();
		PosTypedef nmp;
		
		nmp.x = getRoute(cursor).x;
		nmp.y = getRoute(cursor).y;
		
		
		uint8_t dir = navPosToDir(mp,nmp);
		
		//姿态调整
		moveNavigator(dir,mp,mouseMoveFastI);
		//直线运动
		setLineFastI(getDisGrid(mp,nmp));
		
		//等待抵达
		while(getMousePos().x != getRoute(cursor).x ||
		      getMousePos().y != getRoute(cursor).y);
		
		cursor--;
	}
	
	
	//获得接下来的活动方向
	PosTypedef mp = getMousePos();
	PosTypedef nmp;
	
	nmp.x = 0;
	nmp.y = 0;
	
	uint8_t dir = navPosToDir(mp,nmp);
	
	//姿态调整
	moveNavigator(dir,mp,mouseMoveFastI);
	//直线运动
	setLineFastI(getDisGrid(mp,nmp));
	
	//等待抵达
	while(getMousePos().x != 0||
				getMousePos().y != 0);
	
	mouseMove(MStop);
	mouseMove(ML180);
	mouseMove(MStop);
	
	TIM4->CCR3 = 0;
}

/*================================移动导航加速版I==============================*/


//移动导航加速第二版本
void NavigatorFastII(void)
{
	uint8_t cursor = 0;
	
	
	
	//加速第二版移动导航路径分为三个阶段
	//1.起点到第一个导航点
	//2.第x个导航点到第x+1导航点
	//3.第n-1个导航点到终点-0.5标准栅格
	
	cursor++;
	//第一阶段从起点到第一个导航点
	{
		//获得接下来的活动方向
		PosTypedef mp = getMousePos();
		PosTypedef nmp;
		
		nmp.x = getRoute(cursor).x;
		nmp.y = getRoute(cursor).y;
		
		uint8_t dir = navPosToDir(mp,nmp);
		
		
		//姿态调整
		moveNavigator(dir,mp,mouseMoveFastI);
		
		//直线运动
		setLineFastI(getDisGrid(mp,nmp));
		
		TIM4->CCR3 = 400;//打开风扇
		
		//等待抵达第一个导航点前1/2个标准栅格
		while(getDisGrid(getMousePos(),nmp) != 1);
		TIM4->CCR3 = 800;//打开风扇
		while(readOdometerEtr() < ODOMETER_STD_GRID_ETR_VAL* ATTITUDE_FROTATION_TPOS &&
					getBarrierAct() == 0);
		
		//抵达第一个临界点
	}
	
	TIM4->CCR3 = 400;//打开风扇
	
	//第二阶段 从第x个导航点到第x+1个导航点
	{
		//即将抵达的导航点不是终点
		while( getRoute(cursor).x != getTargetPos().x ||
					 getRoute(cursor).y != getTargetPos().y  )
		{
			//即将抵达的导航点作为当前位置获得可移动方向
			PosTypedef mp = getMousePos();
			PosTypedef nmp;
			
			mp.x = getRoute(cursor).x;
			mp.y = getRoute(cursor).y;
			
			nmp.x = getRoute(cursor + 1).x;
			nmp.y = getRoute(cursor + 1).y;
			
			uint8_t dir = navPosToDir(mp,nmp);
			
			//姿态调整
			moveNavigator(dir,mp,mouseMoveSP);
//			mouseMove(MStop);
//			while(1);
			//直线运动
			setLineFastI(getDisGrid(mp,nmp));
			
			TIM4->CCR3 = 400;//打开风扇
			
			//等待抵达第一个导航点前1/2个标准栅格
			while(getDisGrid(getMousePos(),nmp) != 1);
			
			TIM4->CCR3 = 700;//打开风扇
			
			while(readOdometerEtr() < ODOMETER_STD_GRID_ETR_VAL* ATTITUDE_FROTATION_TPOS &&
						getBarrierAct() == 0);
			
//			mouseMove(MStop);
//			delay_ms(1000);
				
			
//			while(0);
			
			cursor++;
		}
	}
	
	//第三阶段 第n-1个导航点到终点-0.5标准栅格
	{
//		//获得接下来的活动方向
//		PosTypedef mp = getMousePos();
//		PosTypedef nmp = getTargetPos();
//		
//		//直线运动
//		setLineFastI(getDisGrid(mp,nmp));
		
		//等待抵达
		while(getMousePos().x != getTargetPos().x ||
		      getMousePos().y != getTargetPos().y);
		
		mouseMoveFastI(MStop);
		//while(1);
	}
	
	/*----------------------------------------------------------------返回起点*/
	TIM4->CCR3 = 0;
		//返回起点
	cursor--;
	while(getMousePos().x != getRoute(0).x ||
				getMousePos().y != getRoute(0).y)
	{
		//获得接下来的活动方向
		PosTypedef mp = getMousePos();
		PosTypedef nmp;
		
		nmp.x = getRoute(cursor).x;
		nmp.y = getRoute(cursor).y;
		
		
		uint8_t dir = navPosToDir(mp,nmp);
		
		//姿态调整
		moveNavigator(dir,mp,mouseMoveFastI);
		//直线运动
		setLineFastI(getDisGrid(mp,nmp));
		
		//等待抵达
		while(getMousePos().x != getRoute(cursor).x ||
		      getMousePos().y != getRoute(cursor).y);
		
		cursor--;
	}
	
	
	//获得接下来的活动方向
	PosTypedef mp = getMousePos();
	PosTypedef nmp;
	
	nmp.x = 0;
	nmp.y = 0;
	
	uint8_t dir = navPosToDir(mp,nmp);
	
	//姿态调整
	moveNavigator(dir,mp,mouseMoveFastI);
	//直线运动
	setLineFastI(getDisGrid(mp,nmp));
	
	//等待抵达
	while(getMousePos().x != 0||
				getMousePos().y != 0);
	
	mouseMove(MStop);
	mouseMove(ML180);
	mouseMove(MStop);
	
	
}







