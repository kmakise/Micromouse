#ifndef __MOUSEAPP_H
#define __MOUSEAPP_H



typedef enum
{
	REDTEST		,	//红外测试模式
	MAPCREAT	,	//地图创建模式
	START			,	//冲刺模式
	WAIT			, //等待模式
}MouseStateTypedef;




void mouseApp_Setup(void);
void mouseApp_Loop(void);
void mouseAppInterrupt(void);

#endif /*__MOUSEAPP_H*/
