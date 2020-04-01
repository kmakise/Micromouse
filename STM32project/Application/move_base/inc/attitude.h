#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f10x.h"

typedef enum
{
	AStop					= 0,	//停止模式
	AWaiting			= 1,	//姿态修正等待模式
	ASStraight 		= 2,	//搜寻模式 直线
	ARotate				= 3,	//原地自转
	ARotateFast		= 4,	//原地自转加速版
	AFStraight		= 5,	//直线加速优化1
	AFRotate			= 6,	//快速转弯
	
}AttStaTypedef;//姿态修正状态机


typedef struct
{
	int lift;//左侧速度
	int right;//右侧速度
	
}SpeedTypedef;//速度数据结构



//搜寻模式直线搜索时的运行速度设定
void SearcherSpeed(uint16_t lift,uint16_t right);
//设定旋转角度
void RotationAngle(int16_t angel);
//快速转弯完成标志位
uint8_t RotateFastOver(void);

//设定目标位移栅格数量
void setFastStMoveGridNum(uint16_t num);
//设定高速直线编码器累积初始值
void set_FSLEtrnum(int32_t num);

//姿态修正状态机状态配置
void AttitudeStateSet(AttStaTypedef ast);
//姿态修正状态状态获取
AttStaTypedef getAttitudeState(void);
//姿态调整状态机
void AttitudeStateMachine(void);

#endif /*__ATTITUDE_H*/
