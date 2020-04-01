#ifndef __OCCUPYINGGRID_H
#define __OCCUPYINGGRID_H

#include "stm32f10x.h"

/*
				L4B 					Active	Inactive
				|---B0	0°     1				0
				|---B1  90°    1        0
				|---B2	180°   1        0
				|---B3	270°	 1        0

*/

//读取地图
uint8_t readMapGrid(uint8_t x,uint8_t y);
//写入地图
void writeMapGrid(uint8_t x,uint8_t y,uint8_t mp);

//保存地图活动性信息 0左右1前后
void GridActSave(uint8_t num);
//栅格地图的记录
void MapActivityRecord(void);

void GridSetup(void);
void GridLoop(void);


#endif /*__OCCUPYINGGRID_H*/
