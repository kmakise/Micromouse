#ifndef __PIDCTRLLER_H
#define __PIDCTRLLER_H

#include "stm32f10x.h"

/* Structure type definition -------------------------------------------------*/
typedef struct
{
	float			de  [2];			//各个电机当前误差值 difference error
	float			fe  [2];			//各个电机误差积分
	float			de1 [2];			//各个电机历史误差1
	float			de2 [2];			//各个电机历史误差2
	int				out [2];			//各个通道最终PWM输出
	
}PIDDateBaseTypedef;			//pid数据结构类型

typedef struct
{
	float			kp;						//比例权重
	float			ki;						//积分权重
	float			kd;						//微分权重
	
}PIDParamBaseTypedef;			//pid参数结构类型


void SetTargetSpeed(int16_t lift,int16_t right);
void MotorSpeedPidCtrl(void);
void PID_Divider(void);
int16_t getEncoderVal(uint8_t cmd);	//获取速度 编码器每时间分度
int8_t getMotorState(void);					//获得电机的当前状态 0 停止 1 运转
#endif /*PIDCTRLLER_H*/
