/**
  ******************************************************************************
  * @file    myPlanner.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-10-8
  * @brief   最优路径计算与移动导航 【未优化，测试版】 
	*					 运用全排列统计择优的 咚氏 路径规划 穷举法
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "myPlanner.h"
#include "drive.h"
#include "Odometer.h"
#include "OccupyingGrid.h"
#include "MoveCtrl.h"
#include "mapSaver.h"

#include "stdio.h"
/*Golbal Data Space ----------------------------------------------------------*/

#define NODE_MAX            255     //最长路径节点数量
#define PATHPLANNER_I				1


PosTypedef g_TargetPos = {.x = 7,.y = 7,.th = 180}; //目标点的位置
PosTypedef g_StartPos  = {.x = 0,.y = 0,.th = 90 }; //初始点的位置

struct
{
	uint8_t          route[NODE_MAX][2];  					 	//节点坐标
	uint8_t          num;            									//节点数量
	uint8_t          cursor;         									//当前光标

}g_Route; //路径层

//移动方向坐标偏移表 1L1 2U2 3R4 4D8
const int8_t MOVE[4][2] = {
	1, 0,//R
	0, 1,//D
 -1, 0,//L
	0,-1,//U
};

#ifdef PATHPLANNER

/*--------------------图的位翻译---------------------*/
//读取字节的一个位的状态
uint8_t readBitStatus(uint8_t ch, uint8_t num)
{
	return ((ch & (0x01 << num)) != 0);
}
//写字节一个位的状态
uint8_t writeBitStatus(uint8_t ch, uint8_t num, uint8_t state)
{
	ch = state ? ch | (0x01 << num) : ch & (~(0x01 << num));
	return ch;
}
//地图状态的写入
void mapBitWrite(uint8_t * ch, uint8_t num, uint8_t state)
{
	ch = (num < 8) ? ch : ch + 1;
	num = (num < 8) ? num : num - 8;
	*ch = writeBitStatus(*ch, num, state);
}
//地图状态的读取
uint8_t mapBitRead(uint8_t * ch, uint8_t num)
{
	ch = (num < 8) ? ch : ch + 1;
	num = (num < 8) ? num : num - 8;
	return ((*ch & (0x01 << num)) != 0);
}
/*--------------------规划的维护---------------------*/
//获得可移动方向
uint8_t getActiveDir(uint8_t x, uint8_t y)
{
	return  readMapGrid(x,y) & 0x0f;
}
//获得评分数
uint8_t getDirScore(uint8_t x1, uint8_t y1)
{
	uint8_t xt, yt;
	uint8_t x2 = g_TargetPos.x;
	uint8_t y2 = g_TargetPos.y;
	uint8_t temp;

	xt = (x1 > x2) ? x1 - x2 : x2 - x1;
	yt = (y1 > y2) ? y1 - y2 : y2 - y1;

	temp = xt * xt + yt * yt;

	return temp;
}
//获得代价最低的移动方向
uint8_t getLeastCostDir(uint8_t x, uint8_t y, uint8_t dir)
{
	uint8_t ct;
	uint16_t sdir[2];
	sdir[0] = 500;
	//方向唯一性
	if (dir == 0x08 || dir == 0x04 || dir == 0x02 || dir == 0x01)
	{
		switch (dir)
		{
		case 0x08:return 4;
		case 0x04:return 3;
		case 0x02:return 2;
		case 0x01:return 1;
		}
	}
	else
	{
		for (int i = 0; i < 4; i++)
		{
			//如果该方向可移动
			if ((dir&(0x01 << i)) != 0)
			{
				//获得评分
				ct = getDirScore(x + MOVE[i][0], y + MOVE[i][1]);
				//取代价最少的方向
				if (sdir[0] > ct)
				{
					sdir[0] = ct;
					sdir[1] = i;
				}
			}
		}
	}
	return (sdir[1] + 1);
}

////打印点
//void printpoint(uint8_t x, uint8_t y, uint16_t color)
//{
//	block.printxy(x * 2, 19 - y, color, (char *)"█");
//}
////显示路径
//void displayRoute(uint8_t num, uint8_t route[30][2], uint16_t color)
//{
//	for (int i = 0; i < num; i++)
//	{
//		printpoint(route[i][0] + 31, route[i][1] + 1, color);
//	}
//}


/*--------------------路径规划---------------------*/
	uint8_t routenow[NODE_MAX][2];  //规划路线节点列表
	uint8_t posdir[NODE_MAX];		//每个节点的未规划方向
	uint8_t info[2];				//最优路径的里程数和运动代价
	uint8_t cost[2];                //当前路径的里程数和运动代价
	uint8_t nodeClose[16][2];       //全部节点的访问情况
	uint8_t turnNode[NODE_MAX][3];	//记录转向点 xy位置和原方向 用于回溯运动代价

uint8_t routeSearcher(void)
{
	uint8_t backtrackingFlag = 0;	//回溯标志位
	uint8_t cursor = 0;             //节点光标
	uint8_t temp = 0;               //临时雇员
	uint8_t dir_save = 0;           //姿态记录
	uint8_t status = 1;             // 路径规划 状态标志位 0 结束 1进行时

	int  backnum = 0;               //回溯次数
	int  searchnum = 0;             //深度搜索次数

	//初始化节点访问表
	for (int i = 0; i < 16; i++)
	{
		nodeClose[i][0] = 0;
		nodeClose[i][1] = 0;
	}

	//设定起始位置，姿态和访问状态
	routenow[0][0] = g_StartPos.x;
	routenow[0][1] = g_StartPos.y;
	dir_save = g_StartPos.th / 90;
	//nodeClose[car.gbl.y][car.gbl.x] = 1;
	mapBitWrite(nodeClose[g_StartPos.y],g_StartPos.x, 1);

	//设定初始里程数和运动代价
	info[0] = 255;
	info[1] = 255;
	cost[0] = 0;
	cost[1] = 0;
	posdir[0] = 0x0F;

	//开始规划路线
	while (status == 1)
	{
		//回溯操作
		if (backtrackingFlag)
		{
			//节点的回溯操作
			//nodeClose[routenow[cursor][1]][routenow[cursor][0]] = 0;
			mapBitWrite(nodeClose[routenow[cursor][1]], routenow[cursor][0], 0);
			cursor--;
			backnum++;
			cost[0]--;
			//运动代价的回溯操作
			if (routenow[cursor][0] == turnNode[cost[1] - 1][0] && routenow[cursor][1] == turnNode[cost[1] - 1][1])
			{
				cost[1]--;
				dir_save = turnNode[cost[1]][2];
			}
		}
		//更新节点属性
		if (!backtrackingFlag)
		{	//获得可移动方向
			posdir[cursor] = getActiveDir(routenow[cursor][0], routenow[cursor][1]);
			//失能周围已经被遍历的路径
			for (int i = 0; i < 4; i++)
			{	//如果该节点已经被访问
				//(nodeClose[routenow[cursor][1] + MOVE[i][1]][routenow[cursor][0] + MOVE[i][0]] == 1)
				if (mapBitRead((nodeClose[routenow[cursor][1] + MOVE[i][1]]), (routenow[cursor][0] + MOVE[i][0])) == 1)
				{
					posdir[cursor] = posdir[cursor] & (~(0x01 << i));
				}
			}
		}
		//节点活动性
		if (posdir[cursor])
		{
			backtrackingFlag = 0;
			//获得最小代价的移动方向
			temp = getLeastCostDir(routenow[cursor][0], routenow[cursor][1], posdir[cursor]);
			posdir[cursor] = posdir[cursor] & (~(0x01 << (temp - 1)));//标记该方向已经移动
			//记录运动代价
			if (dir_save != temp)
			{
				//记录运动代价并保存原方向
				turnNode[cost[1]][0] = routenow[cursor][0];
				turnNode[cost[1]][1] = routenow[cursor][1];
				turnNode[cost[1]][2] = dir_save;
				cost[1]++;
			}
			dir_save = temp;
			//增加光标添加路线节点并使能访问状态
			routenow[cursor + 1][0] = routenow[cursor][0] + MOVE[temp - 1][0];
			routenow[cursor + 1][1] = routenow[cursor][1] + MOVE[temp - 1][1];
			cursor++;
			if (cursor == NODE_MAX)
			{
//				std::cout << "节点数量已超出最大范围";
				return 0;
			}
			else
			{
				//nodeClose[routenow[cursor][1]][routenow[cursor][0]] = 1;
				mapBitWrite(nodeClose[routenow[cursor][1]], routenow[cursor][0], 1);
			}
			cost[0]++;
		}
		else//无法移动
		{	//如果是原点
			if (cursor == 0)
			{
				status = 0;//路径规划完成
			}
			//回溯操作
			backtrackingFlag = 1;
		}
		//到达目标点
		if (routenow[cursor][0] == g_TargetPos.x && routenow[cursor][1] == g_TargetPos.y)
		{
			searchnum++;
			//如果当前里程小于最优路径里程 或者 里程数相等 运动代价更少 当转向点小于4时不在进行规划
			if((cost[0] + cost[1]) < (info[0] + info[1]))//(info[0] > cost[0] || (info[0] == cost[0] && info[1] > cost[1]) || cost[1] < 2)
			{
				//将路径设为最优路径
				memcpy(info, cost, 2);
				for (int i = 0; i < cost[1]; i++)
				{
					g_Route.route[i][0] = turnNode[i][0];
					g_Route.route[i][1] = turnNode[i][1];
				}
				g_Route.num = info[1];
				//将目标点添加进导航图
				g_Route.route[g_Route.num][0] = g_TargetPos.x;
				g_Route.route[g_Route.num][1] = g_TargetPos.y;
				g_Route.num++;
			}
			//当转向点小于4个时 不需要再进行搜索
			if (cost[1] < 2)
			{
				status = 0;//路径规划完成
			}
			//回溯操作
			backtrackingFlag = 1;
		}
		//运动代价大于已有的路径 执行回溯操作
		if(cost[1] > info[1] || cost[0] > info[0])
		{
			//回溯操作
			backtrackingFlag = 1;
		}
	}
	//路径规划结束

//	block.printxy(0, 25, CL_W, (char*)" ");
	//判断规划是否成功
	if (info[1] != 100)
	{
		//路径规划完成
//		std::cout << "最优路径规划完成,导航点规划完成 回溯数：" << backnum << ",深度搜索:" << searchnum << "次";
//		std::cout << "栅格,转向点:" << (int)(info[1] + 1) << "个，线段数量:" << (int)(info[1]) << "条" << std::endl;
//		displayRoute(Pmap.route.num, Pmap.route.route, CL_G);
//		block.printxy(0, 26, CL_W, (char*)" ");
		return 1;
	}
	else
	{
//		std::cout << "最优路径规划失败 无法搜索到有效路径 回溯数:" << backnum << std::endl;
//		block.printxy(0, 26, CL_W, (char*)" ");
		return 0;
	}
	return 0;
}

#endif /*PATHPLANNER*/

/*--------------------初始化相关信息------------------*/
void plannerInit(void)
{
	g_TargetPos.x = 7;
	g_TargetPos.y = 7;
	g_TargetPos.th = 180;
	
	g_StartPos = getMousePos();
}
/*---------------------对外提供信息-------------------*/
//获得节点坐标信息
PosTypedef getRoute(uint8_t cur)
{
	static PosTypedef mp;
	
	mp.x = g_Route.route[cur][0];
	mp.y = g_Route.route[cur][1];
	mp.th = 0;
	
	return mp;
}

//获得目标点的坐标信息
PosTypedef getTargetPos(void)
{
	return g_TargetPos;
}

//获取节点数量
uint8_t getPathCurNum(void)
{
	return g_Route.num;
}

/*---------------------修改信息-----------------*/
//修改节点信息
void setPathNode(uint8_t cur,PosTypedef mp)
{
	g_Route.route[cur][0] = mp.x;
	g_Route.route[cur][1] = mp.y;
}

//设置序列节点数量
void setPathCurNum(uint8_t num)
{
	g_Route.num = num;
}

//修改目标点
void setTargetPos(PosTypedef tg)
{
	g_TargetPos = tg;
}


/*---------------------------------------------------------路径规划第二版本*/

/*核心思想 以起点等距离步进扩散 写入到每个点运动代价 从终点再会到起点
 *
 *关于点的更新	计算点的运动代价小于即将覆盖点的运动代价则覆盖并设置为活动
 *光标的截止	光标无法活动 无法活动：1.地图障碍限制 2.周围所有点的运动代价小于即将预设 
 *光标的存储	环形队列 读取下标 写入下标 连续循环写入和读取
 *光标重复问题	光标已经存在则不写入
 *光标的活动性	参照光标截止
 *移动			在移动前计算可活动性 不可活动则直接忽略
 *终止条件		无可活动光标
 */

#ifdef PATHPLANNER_I

//地图各个点的运动代价
uint16_t g_MapData[16][16];

//光标的数据存储结构
typedef struct
{
	PosTypedef cur[256];
	uint8_t wt;
	uint8_t rd;
}CurPosArrTypedef;

CurPosArrTypedef g_mapCurArr;


//读取一个光标
PosTypedef getCurPos(void)
{
	PosTypedef p;
	p = g_mapCurArr.cur[g_mapCurArr.rd];
	
	g_mapCurArr.cur[g_mapCurArr.rd].th = 0xFFFF;
	g_mapCurArr.cur[g_mapCurArr.rd].x = 0xFF;
	g_mapCurArr.cur[g_mapCurArr.rd].y = 0xFF;
	
	g_mapCurArr.rd = (g_mapCurArr.rd == 255) ? 0 : g_mapCurArr.rd + 1;
	return p;
}
//写入一个光标
void saveCurPos(PosTypedef p)
{
	g_mapCurArr.cur[g_mapCurArr.wt] = p;
	
	g_mapCurArr.wt = (g_mapCurArr.wt == 255) ? 0 : g_mapCurArr.wt + 1;
}
//读取地图一个点的交叉可活动方向
uint8_t readMapPosX(PosTypedef p)
{
	//为防止单向导通而设置安全冗余
	//一个方向的活动性与邻接点对应的活动方向同时存在时有效
	uint8_t dir = readMapGrid(p.x, p.y) & 0x0f;
	
	//各个方向的再次查询再次确认活动方向的正确性
	dir = (p.x == 15) ? dir & (~0x01) : ((readMapGrid(p.x + 1, p.y) & 0x04) != 0) ? dir & (~0x00) : dir & (~0x01);
	dir = (p.x == 0 ) ? dir & (~0x04) : ((readMapGrid(p.x - 1, p.y) & 0x01) != 0) ? dir & (~0x00) : dir & (~0x04);
	dir = (p.y == 15) ? dir & (~0x02) : ((readMapGrid(p.x, p.y + 1) & 0x08) != 0) ? dir & (~0x00) : dir & (~0x02);
	dir = (p.y == 0 ) ? dir & (~0x08) : ((readMapGrid(p.x, p.y - 1) & 0x02) != 0) ? dir & (~0x00) : dir & (~0x08);
 
	return dir;
}
//读取点的运动代价
uint16_t readMapDis(PosTypedef p)
{
	return g_MapData[p.x][p.y];
}
//写入点的运动代价
void writeMapDis(PosTypedef p, uint16_t dis)
{
	g_MapData[p.x][p.y] = dis;
}


//可活动方向到角度转换
uint16_t dirToTh(uint8_t dir)
{
	if (dir == 0x01) return 0;
	if (dir == 0x02) return 90;
	if (dir == 0x04) return 180;
	if (dir == 0x08) return 270;
	return 0;
}
//获得偏移后的坐标
PosTypedef getPosAfterMove(PosTypedef p, uint8_t dir)
{
	if (dir == 0x01)p.x++;
	if (dir == 0x02)p.y++;
	if (dir == 0x04)p.x--;
	if (dir == 0x08)p.y--;

	p.th = dirToTh(dir);

	return p;
}

////显示当前点的运动代价
//void viewDisXY(PosTypedef p,uint32_t color)
//{
//	block.printxy(p.x * 3 + 60, 18 - p.y, color, (char *)"");
//	int dis = readMapDis(p);
//	cout << dis;
//}

//路径计划者I
void pathPlannerI(void)
{
	//规划主循环
	for(PosTypedef cur;(cur = getCurPos()).th != 0xFFFF;)
	{
		//当前光标四个方向的移动
		for (int i = 0; i < 4; i++)
		{
			//当前活动方向
			uint8_t dir = (0x01 << i);

			//当前方向地图允许移动
			if ((readMapPosX(cur) & dir) != 0)
			{
				//当前位置的运动代价
				uint8_t dis = readMapDis(cur) + 1;

				//当前点的姿态与当前活动方向不同
				if (cur.th != dirToTh(dir))
				{
					dis += 1;//转向代价+1
				}
				//坐标偏移
				PosTypedef nmp = getPosAfterMove(cur, dir);

				//即将抵达点的运动代价大于当前运动代价
				if (readMapDis(nmp) > dis)
				{
					//写入运动代价
					writeMapDis(nmp, dis);
					//写入光标
					saveCurPos(nmp);

					//显示当前坐标的运动代价
					//viewDisXY(nmp,CL_W);
				}
			}
		}
	}
}

//路径缓冲区
PosTypedef g_pathAnl[256];
//导航点焦点
PosTypedef navigNode[100];

//路径解析
void pathAnalysis(void)
{

	//路径缓冲区焦点
	uint8_t cur = 0;
	//导航点焦点
	uint8_t ncur = 0;
	//路径初始点
	g_pathAnl[0] = getTargetPos();
	g_pathAnl[0].th = 360;

	//路径提取 从终点开始搜索最低运动代价到起点
	while (g_pathAnl[cur].x != 0 || g_pathAnl[cur].y != 0)
	{
		//上一个运动代价的保存
		uint8_t dis = 255;

		//四个方向的最小运动代价搜寻
		for (int i = 0; i < 4; i++)
		{
			//当期预期活动方向
			uint8_t dir = (0x01 << i);

			//当前方向地图允许移动
			if ((readMapPosX(g_pathAnl[cur]) & dir) != 0)
			{
				//坐标偏移
				PosTypedef nmp = getPosAfterMove(g_pathAnl[cur], dir);
				//当前点的运动代价小于已保存的运动代价
				if (readMapDis(nmp) < dis)
				{
					//保存运动代价
					dis = readMapDis(nmp);
					//保存点
					g_pathAnl[cur + 1] = nmp;
				}
			}
		}
		//移动光标
		cur++;
		//显示路径
		//viewDisXY(path[cur], CL_G);
		//Sleep(200);


		if (g_pathAnl[cur - 1].th != g_pathAnl[cur].th)
		{
			navigNode[ncur] = g_pathAnl[cur - 1];
			//viewDisXY(navigNode[ncur], CL_R);
			ncur++;
			//Sleep(200);
		}

	}
	//将起点添加进导航点序列
	navigNode[ncur].x = 0;
	navigNode[ncur].y = 0;
	ncur++;


	//保存导航点
	for (int i = 0; i < ncur; i++)
	{
		setPathNode(i,navigNode[ncur - 1 - i]);
	}

	while (0);
}
/*
uint8_t map[] =
{
	
	0x12,0x13,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x16,
	0x1A,0x1A,0x13,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x16,0x1A,
	0x1B,0x1E,0x1A,0x11,0x17,0x15,0x15,0x15,0x15,0x15,0x17,0x16,0x13,0x16,0x1A,0x1A,
	0x1A,0x1A,0x1A,0x13,0x1D,0x16,0x13,0x15,0x17,0x14,0x1A,0x19,0x1C,0x19,0x1E,0x1A,
	0x1A,0x1A,0x1A,0x1A,0x12,0x18,0x1B,0x16,0x19,0x16,0x1A,0x11,0x15,0x17,0x1C,0x1A,
	0x1A,0x1A,0x1A,0x19,0x1E,0x12,0x18,0x19,0x16,0x19,0x1C,0x12,0x13,0x1C,0x12,0x1A,
	0x1A,0x1A,0x1A,0x12,0x19,0x1E,0x13,0x15,0x1D,0x16,0x12,0x1B,0x1C,0x13,0x1E,0x1A,
	0x1A,0x1A,0x1A,0x19,0x17,0x1E,0x1B,0x17,0x16,0x1A,0x1B,0x1E,0x13,0x1C,0x1A,0x1A,
	0x1A,0x1A,0x1A,0x13,0x1C,0x1A,0x1A,0x19,0x1C,0x1A,0x1A,0x1A,0x1A,0x13,0x1C,0x1A,
	0x1A,0x1A,0x1A,0x1A,0x12,0x18,0x19,0x15,0x15,0x1C,0x1A,0x1A,0x1A,0x19,0x16,0x1A,
	0x1A,0x1A,0x1A,0x19,0x1F,0x14,0x13,0x15,0x15,0x15,0x1E,0x1A,0x1A,0x13,0x1C,0x1A,
	0x1A,0x1A,0x1B,0x15,0x1D,0x16,0x19,0x15,0x15,0x16,0x1A,0x1A,0x1A,0x1A,0x12,0x1A,
	0x1A,0x1A,0x1A,0x13,0x15,0x1C,0x13,0x15,0x15,0x1C,0x18,0x19,0x1C,0x19,0x1E,0x1A,
	0x1A,0x1A,0x19,0x1D,0x15,0x15,0x1D,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x1D,0x1E,
	0x1A,0x19,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x1E,
	0x19,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x1C,
	
	0x1A,0x12,0x12,0x12,0x13,0x15,0x15,0x15,0x15,0x17,0x15,0x17,0x14,0x11,0x15,0x16,
	0x1A,0x1A,0x1B,0x1C,0x1B,0x16,0x13,0x15,0x17,0x1D,0x16,0x19,0x16,0x13,0x15,0x1E,
	0x1A,0x19,0x1D,0x17,0x1C,0x19,0x1F,0x16,0x1B,0x16,0x1B,0x16,0x19,0x1E,0x12,0x1A,
	0x1B,0x15,0x15,0x1D,0x17,0x14,0x1A,0x19,0x1E,0x1A,0x18,0x1B,0x16,0x19,0x1E,0x1A,
	0x1A,0x12,0x13,0x16,0x1B,0x14,0x1B,0x15,0x1E,0x1A,0x12,0x18,0x19,0x16,0x19,0x1E,
	0x1B,0x1F,0x1C,0x19,0x1D,0x14,0x19,0x15,0x1F,0x1C,0x19,0x15,0x16,0x19,0x16,0x1A,
	0x1A,0x1A,0x12,0x13,0x15,0x14,0x12,0x11,0x1C,0x13,0x16,0x13,0x1D,0x15,0x1E,0x1A,
	0x1A,0x1A,0x19,0x1F,0x15,0x15,0x1C,0x13,0x16,0x1A,0x1B,0x1C,0x13,0x14,0x1A,0x1A,
	0x1B,0x1D,0x15,0x1F,0x14,0x13,0x15,0x1D,0x1C,0x1B,0x1C,0x13,0x1D,0x17,0x1D,0x1E,
	0x1B,0x14,0x11,0x1E,0x13,0x1C,0x13,0x15,0x17,0x1C,0x13,0x1D,0x16,0x1A,0x12,0x1A,
	0x1A,0x13,0x17,0x1E,0x1B,0x17,0x1D,0x15,0x1C,0x12,0x1A,0x13,0x1E,0x19,0x1F,0x14,
	0x19,0x1E,0x1A,0x1A,0x18,0x19,0x15,0x14,0x13,0x1D,0x1D,0x1C,0x19,0x17,0x1D,0x16,
	0x13,0x1D,0x1F,0x1D,0x17,0x17,0x15,0x17,0x1C,0x13,0x14,0x13,0x14,0x19,0x16,0x1A,
	0x1A,0x11,0x1E,0x00,0x1A,0x1B,0x14,0x1A,0x12,0x1B,0x16,0x1B,0x15,0x15,0x1D,0x1E,
	0x1B,0x14,0x1A,0x00,0x1A,0x1A,0x00,0x1A,0x1B,0x1E,0x1B,0x1D,0x17,0x14,0x11,0x1E,
	0x19,0x15,0x1D,0x15,0x1D,0x1D,0x15,0x1D,0x1C,0x19,0x1D,0x15,0x1D,0x15,0x15,0x1C,
	

};*/

//规划数据初始化
void plannerDataInit(void)
{
	for (int x = 0; x < 16; x++)
	{
		for (int y = 0; y < 16; y++)
		{
			g_MapData[x][y] = 0xFFFF;
		}
	}
	for (int i = 0; i < 256; i++)
	{
		g_mapCurArr.cur[i].x = 0;
		g_mapCurArr.cur[i].y = 0;
		g_mapCurArr.cur[i].th = 0;
	}
	g_mapCurArr.cur[0].x = 0;
	g_mapCurArr.cur[0].y = 0;
	g_mapCurArr.cur[0].th = 90;
	g_mapCurArr.wt = 1;
	g_mapCurArr.rd = 0;

	//map[0] &= 0x07;
	
	g_MapData[0][0] = 0;
	
/*
	for(int8_t x = 0;x < 16;x++)
	{
		for(int8_t y = 0;y < 16;y++)
		{
			writeMapGrid(x,y,map[x + y*16]);
		}
	}
	*/
	
}


uint8_t routeSearcher(void)
{
	//导航数据初始化
	plannerDataInit();
	//路径规划
	pathPlannerI();
	//保存路径
	writePathToFlash();
	//路径解析
	pathAnalysis();
	
	return 1;
}

#endif /*PATHPLANNER_I*/
	



