/**
  ******************************************************************************
  * @file    ErgodicMap.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-10-5
  * @brief   地图的扫描与建立
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "ErgodicMap.h"
#include "MoveCtrl.h"
#include "odometer.h"
#include "OccupyingGrid.h"
#include "parameter.h"
#include "myPlanner.h"
#include "navigator.h"
#include "mapSaver.h"

#include "math.h"
/*Golbal data space ----------------------------------------------------------*/
//可活动方向表
uint8_t g_actdir[255];
//回溯坐标 0x 1y
uint8_t g_actpos[255][2];
//回溯存储光标
uint8_t g_actCur = 0;

//扫描路径 0x 1y
PosTypedef g_pathPos[255];
//路径光标
uint16_t g_pathCur = 0;

//搜寻计划
uint16_t g_SearchPlant = ERGODICMAP_SEARCHOR_PLAN;

//配置搜寻计划
void setSearchPlant(uint8_t p)
{
	g_SearchPlant = p;
}


//抵达终点
uint8_t getOver(void)
{
	//可作为终点的坐标
	const PosTypedef tg[4] = {
		ERGODICMAP_TARGET_POS_1	,
		ERGODICMAP_TARGET_POS_2	,  
		ERGODICMAP_TARGET_POS_3	, 
		ERGODICMAP_TARGET_POS_4 ,
	};
	
	PosTypedef mp = getMousePos();
	
	for(int i = 0;i < 4;i++)
	{
		if(	mp.x == tg[i].x &&
				mp.y == tg[i].y	)
		{
			//将第一个抵达的点设为目标点
			setTargetPos(tg[i]);
			return 1;
		}
	}
	return 0;
}

//地图建立完成判断
//针对全部可访问点的已访问性
//可适应任何尺寸以及形状的地图
uint8_t getMapOver(void)
{
	/*
				L4B 					Active	Inactive
				|---B0	0°     1				0
				|---B1  90°    1        0
				|---B2	180°   1        0
				|---B3	270°	 1        0
	*/
	
	uint8_t dir = 0;
	for(int x = 0;x < 16;x++)
	{
		for(int y = 0;y < 16;y++)
		{
			//读取可活动方向
			dir = readMapGrid(x,y);
			
			//所有可活动方向上无未知点即为完成
			if(dir != 0)
			{
				if((dir & 0x01)!=0)//0
				{
					if(x != 15)
					{
						if(readMapGrid(x + 1,y) == 0)
						{
							return 0;
						}
					}
				}
				if((dir & 0x02)!=0)//90
				{
					if(y != 15)
					{
						if(readMapGrid(x,y + 1) == 0)
						{
							return 0;
						}
					}
				}
				if((dir & 0x04)!=0)//180
				{
					if(x != 0)
					{
						if(readMapGrid(x - 1,y) == 0)
						{
							return 0;
						}
					}
				}
				if((dir & 0x08)!=0)//270
				{
					if(y != 0)
					{
						if(readMapGrid(x,y - 1) == 0)
						{
							return 0;
						}
					}
				}
			}
		}
	}
	return 1;
}

uint8_t overFlag(void)
{
	//配置优先抵达终点的坐标信息
	getOver();
	
	//搜寻终止条件
	if(
			((
	
				getOver() //抵达终点
	
			)&&(ERGODICMAP_STOP_ALLCPLT == 0)) ||

			getMapOver()//确认全部扫描完成
		)
	{
		return 0;
	}
	return 1;
}

//获得可活动性方向的数量
uint8_t getDirnum(uint8_t dir)
{
	//可活动方向数量
	uint8_t num = 0;
	
	if(dir&0x01)num++;
	if(dir&0x02)num++;
	if(dir&0x04)num++;
	if(dir&0x08)num++;
	
	return num;
}

//获得优先选择的方向
uint8_t getDirFirst(uint8_t wall,uint8_t plant)
{
	switch(plant)
	{
		case 0://xy正方向优先
		{
			if((wall & 0x01) != 0)return 0x01;
			if((wall & 0x02) != 0)return 0x02;
			if((wall & 0x04) != 0)return 0x04;
			if((wall & 0x08) != 0)return 0x08;
			break;
		}
		case 1://xy负方向优先
		{
			if((wall & 0x08) != 0)return 0x08;
			if((wall & 0x04) != 0)return 0x04;
			if((wall & 0x01) != 0)return 0x01;
			if((wall & 0x02) != 0)return 0x02;
			break;
		}
		case 2:
		{
			//方向表 270 180 90 0
			
			//最优方向
			uint8_t dir = 0;
			//距离中心点的距离
			float dis = 1000;
			
			//计算四个方向取出最优距离
			for(int i = 0;i < 4;i++)
			{
				//该方向可活动性
				if((wall & (0x01 << i)) != 0)
				{
					//计算偏移坐标
					PosTypedef nmp = getMousePos();
					if(i == 0)nmp.x++;
					else if(i == 1)nmp.y++;
					else if(i == 2)nmp.x--;
					else if(i == 3)nmp.y--;
					
					float dis_t = sqrt((7.5 - nmp.x) * (7.5 - nmp.x) +
													(7.5 - nmp.y) * (7.5 - nmp.y) );
					
					if(dis_t < dis)
					{
						dis = dis_t;
						dir = (0x01 << i);
					}
				}
			}
			return dir;
		}
	}
}

//获得下一个点
PosTypedef getNextPos(uint8_t dirf,PosTypedef mp)
{

	//偏移坐标
	if((dirf & 0x01) != 0)mp.x++;
	if((dirf & 0x02) != 0)mp.y++;
	if((dirf & 0x04) != 0)mp.x--;
	if((dirf & 0x08) != 0)mp.y--;
	
	return mp;
}

//通过下一个目标点计算方向
uint8_t nPosToDir(PosTypedef mp,PosTypedef nmp)
{
	if(mp.x > nmp.x)return 0x04;
	if(mp.y > nmp.y)return 0x08;
	if(mp.x < nmp.x)return 0x01;
	if(mp.y < nmp.y)return 0x02;
	return 0;
}


//按照引导方向移动
void moveToNext(uint8_t dir,PosTypedef mp)
{
	//调整姿态
	switch(dir)
	{
		case 0x01:
			switch (mp.th)
			{
				case 90  :mouseMove(MR90);break;
				case 180 :mouseMove(ML180);break;
				case 270 :mouseMove(ML90);break;
			}break;
		case 0x02:
			switch (mp.th)
			{
				case 0   :mouseMove(ML90);break;
				case 180 :mouseMove(MR90);break;
				case 270 :mouseMove(ML180);break;
			}break;
		case 0x04:
			switch (mp.th)
			{
				case 0   :mouseMove(ML180);break;
				case 90  :mouseMove(ML90);break;
				case 270 :mouseMove(MR90);break;
			}break;
		case 0x08:
			switch (mp.th)
			{
				case 0   :mouseMove(MR90);break;
				case 90  :mouseMove(ML180);break;
				case 180 :mouseMove(ML90);break;
			}break;
	}
	//前进
	mouseMove(MForward);
}

//保存可活动回溯点
void savePathPoint(PosTypedef mp,uint8_t dir)
{
	//保存路径
	g_pathPos[g_pathCur].x = mp.x;
	g_pathPos[g_pathCur].y = mp.y;
	g_pathPos[g_pathCur].th = mp.th;
	
	//移动路径光标
	g_pathCur++;
	
	
	//保存可回溯的点
	if(getDirnum(dir) > 0)
	{
		//保存活动性
		g_actdir[g_actCur] = dir;
		//保存坐标
		g_actpos[g_actCur][0] = mp.x;
		g_actpos[g_actCur][1] = mp.y;
		
		//移动可回溯点的光标
		g_actCur++;
	}
}

//删除后侧访问方向
uint8_t delFromDir(uint8_t dir,PosTypedef mp)
{
	switch(mp.th)
	{
		case 0:		return dir & (~0x04);
		case 90:	return dir & (~0x08);
		case 180:	return dir & (~0x01);
		case 270:	return dir & (~0x02);
	}
}

//已访问状态
uint8_t visitedState(uint16_t x,uint16_t y)
{
	for(int i = 0;i < g_pathCur;i++)
	{
		if((g_pathPos[i].x == x &&
			 g_pathPos[i].y == y) ||
			 (readMapGrid(x,y) != 0))
		{
			return 1;
		}
	}
	return 0;
}
//下一点的访问情况
uint8_t visitedCheck(PosTypedef mp,uint8_t dir)
{
	if(visitedState(mp.x + 1,mp.y))dir = dir & (~0x01);
	if(visitedState(mp.x,mp.y + 1))dir = dir & (~0x02);
	if(visitedState(mp.x - 1,mp.y))dir = dir & (~0x04);
	if(visitedState(mp.x,mp.y - 1))dir = dir & (~0x08);
	return dir;
}

uint8_t BackActCheck(void)
{
	PosTypedef mp;

	
	//上一个回溯点的检查
	g_actCur--;
	
	mp.x = g_actpos[g_actCur][0];
	mp.y = g_actpos[g_actCur][1];
	
	uint8_t dir = g_actdir[g_actCur];
	//在可访问方向上如果邻接点已经访问则此方向失能
	g_actdir[g_actCur] = visitedCheck(mp,dir);
	
	if(g_actdir[g_actCur] != 0)
	{
		return 0;
	}
	return 1;
}

//删除死区
void delDeathGrid(void)
{
	for (int x = 0; x < 16; x++)
	{
		for (int y = 0; y < 16; y++)
		{
			//当前点未使能
			if (readMapGrid(x, y) == 0)
			{
				//周围四个点均已访问
				if (((readMapGrid(x + 1, y) != 0) || (x == 15)) &&
					((readMapGrid(x - 1, y) != 0) || (x == 0)) &&
					((readMapGrid(x, y + 1) != 0) || (y == 15)) &&
					((readMapGrid(x, y - 1) != 0) || (y == 0))
					)
				{
					//计算点的状态
					uint8_t state = 0;
					state |= 0x10;//状态已访问
					state |= (x != 15) ? (readMapGrid(x + 1, y) & 0x04) >> 2 : 0;//0		1
					state |= (y != 15) ? (readMapGrid(x, y + 1) & 0x08) >> 2 : 0;//90	2
					state |= (x != 0) ? (readMapGrid(x - 1, y) & 0x01) << 2 : 0;//180	4
					state |= (y != 0) ? (readMapGrid(x, y - 1) & 0x02) << 2 : 0;//270	8
					//写入地图
					writeMapGrid(x,y,state);
				}
			}
		}
	}
}

//删除路径中存在的环 待完成
void pathRingDel(void)
{
	
	
}

/*come back to start -----------------------------------------------------------*/

//返回到起点
void backtoStart(void)
{
	//删除路径中存在的环
	 pathRingDel();
	
	//回溯路径
	g_pathCur--;
	
	//当前位置不是目的回溯位置
	while(getMousePos().x != 0 ||
				getMousePos().y != 0)
	{
		//读取上一个路径点作为目标点
		uint8_t dir = nPosToDir(getMousePos(),g_pathPos[g_pathCur]);
		
		//移动
		moveToNext(dir,getMousePos());		
		
		//等待
		while(getMousePos().x != g_pathPos[g_pathCur].x ||
					getMousePos().y != g_pathPos[g_pathCur].y );
		//光标回退
		g_pathCur--; 
	}
	
	mouseMove(MStop);
	mouseMove(ML180);
	mouseMove(MStop);
}

/*Map Creator ------------------------------------------------------------------*/

//回溯到上一个可活动的坐标 阻塞执行
void backToLastAct(void)
{
	//当前回溯点有效性更新
	while(BackActCheck());
	//回溯路径
	g_pathCur--;

	//当前位置不是目的回溯位置
	while(getMousePos().x != g_actpos[g_actCur][0] ||
				getMousePos().y != g_actpos[g_actCur][1] )
	{

		//读取上一个路径点作为目标点
		uint8_t dir = nPosToDir(getMousePos(),g_pathPos[g_pathCur]);
		
		//移动
		moveToNext(dir,getMousePos());	
		
		//等待
		while(getMousePos().x != g_pathPos[g_pathCur].x ||
					getMousePos().y != g_pathPos[g_pathCur].y );
		//光标回退
		g_pathCur--; 
	}
	
	g_pathCur += 1; 
	//移动到活动方向
	uint8_t dir = g_actdir[g_actCur];
	uint8_t dirf = getDirFirst(dir,g_SearchPlant);//获得优先方向
	PosTypedef nmp  = getNextPos(dirf,getMousePos());//获得下一个点的位置
	//保存路径
	savePathPoint(getMousePos(),dir&(~dirf));
	//移动
	moveToNext(dirf,getMousePos());
	//等待
	while(getMousePos().x != nmp.x ||
				getMousePos().y != nmp.y );
}

//地图建立 每移动到一个新坐标则更新一次
void MapCreator(void)
{
	//平台位姿
	//平台坐标
	PosTypedef mp,nmp;
	//平台可活动方向
	uint8_t dir,dirf;
	
	//等待地图建立完成 全图无未知点或者全图无可活动方向
	while(overFlag())//(getOver())//(getMapOver())//
	{
		//删除死区
		delDeathGrid();
		
		//获得可活动方向
		mp = getMousePos();
		dir = readMapGrid(mp.x,mp.y);
		
		//在可访问方向上如果邻接点已经访问则此方向失能
		dir = visitedCheck(mp,dir);
		
		//有可活动方向 且前方未扫描 - 移动到下一点
		if(getDirnum(dir) > 0)
		{
			dir = delFromDir(dir,mp);
			//移动方向的决策 得到下一个目标点
			dirf = getDirFirst(dir,g_SearchPlant);//获得优先方向
			nmp  = getNextPos(dirf,mp);//获得下一个点的位置
			//保存点和路径
			savePathPoint(mp,dir&(~dirf));
			//移动
			moveToNext(dirf,mp);
			
			//等待抵达点下一个点
			while(getMousePos().x != nmp.x ||
						getMousePos().y != nmp.y );
		}
		//无可活动方向	遭遇已经过点 -	回退到上一个可活动的点
		else
		{
			backToLastAct();
		}
	}
}

/*Setup loop ----------------------------------------------------------------------*/
//地图建立初始化
void MapCreatorInit(void)
{
	for(int i = 0;i < 100;i++)
	{
		g_actdir[i] = 0;
		g_actpos[i][0] = 0;
		g_actpos[i][1] = 0;
	}
	g_actCur = 0;
	
	for(int i = 0;i < 255;i++)
	{
		g_pathPos[i].x = 0;
		g_pathPos[i].y = 0;
		g_pathPos[i].th = 0;
	}
	g_pathCur = 0;
}

void MapCreatLoop(void)
{
	//搜索地图
	MapCreator();
	//停止
	mouseMove(MStop);
	//路径规划
	routeSearcher();	
	//保存路径
	writePathToFlash();
	
	//方案3以起点作为终点故不需要返回
	if(ERGODICMAP_SEARCHOR_PLAN != 3)
	{
		backtoStart();
	}
	else
	{
		mouseMove(MStop);
		mouseMove(ML180);
		mouseMove(MStop);
	}
}
