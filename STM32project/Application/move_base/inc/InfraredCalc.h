#ifndef __INFRAREDCALC_H
#define __INFRAREDCALC_H

#include "stm32f10x.h"

typedef enum
{
	LS = 4,
	LF = 1,
	RF = 2,
	RS = 8,
	
}InfredTypedef;

typedef struct
{
	
	uint16_t LS;
	uint16_t LF;
	uint16_t RF;
	uint16_t RS;
	
}RedValTypedef;





void InfraredLoop(void);//红外侦测中断轮询

int16_t getRedOffset(void);//获得偏移量 矢量数值 左负 右正
uint8_t getBarrierAct(void); //障碍物抵近触发方法
uint8_t getActDirState(void);//获得当前坐标的可活动信息
uint8_t getInfraredEventVal(void);//获得红外触发编号
#endif /*__INFRAREDCALC_H*/
