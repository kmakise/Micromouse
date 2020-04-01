#ifndef __MYPLANNER_H
#define __MYPLANNER_H

#include "Odometer.h"


void plannerInit(void);



//路径规划
uint8_t routeSearcher(void);
//获得目标点的坐标信息
PosTypedef getTargetPos(void);
//获得节点坐标信息
PosTypedef getRoute(uint8_t cur);
//获取节点数量
uint8_t getPathCurNum(void);


//修改目标点
void setTargetPos(PosTypedef tg);
//修改节点信息
void setPathNode(uint8_t cur,PosTypedef mp);
//设置序列节点数量
void setPathCurNum(uint8_t num);



#endif /*__MYPLANNER_H*/

