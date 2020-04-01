/**
  ******************************************************************************
  * @file    pidctrller.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-9-21
  * @brief   运动基础组 电机速度微分积分比例控制器 0l 1r
  ******************************************************************************
  */
	
/*include file ---------------------------------------------------------------*/
#include "pidctrller.h"
#include "stm32f10x.h"
#include "drive.h"
#include "stdio.h"
#include "parameter.h"
/*Global Date Space ----------------------------------------------------------*/


const PIDParamBaseTypedef PARAM_PID = {//电机pid速度控制参数

	.kp = MOTOR_SPEED_PID_KP,				//比例权重
	.ki = MOTOR_SPEED_PID_KI,				//积分权重
	.kd = MOTOR_SPEED_PID_KD,				//微分权重
	
};

int16_t g_TargetSpeed[2]	 = {0,0};//目的速度
int16_t g_MotorEncoder[2]	 = {0,0};//当前速度

//速度值 编码器 更新
void updataEncoderVal(void)
{
	g_MotorEncoder[0] = TIM2->CNT;  //l     
	TIM2->CNT = 0;
	g_MotorEncoder[1] = TIM3->CNT;	//r
	TIM3->CNT = 0;
}
//获得电机的当前状态 0 停止 1 运转
int8_t getMotorState(void)
{
	if(
			//左侧电机
			(
				(g_MotorEncoder[0] < PIDCONTROLLER_MOTOR_STOP)  &&
				(g_MotorEncoder[0] > -PIDCONTROLLER_MOTOR_STOP) 
			)
			//右侧电机
			&&
			(
				(g_MotorEncoder[1] < PIDCONTROLLER_MOTOR_STOP)  &&
				(g_MotorEncoder[1] > -PIDCONTROLLER_MOTOR_STOP) 
			)
//			//目的状态
//			&&
//			(
//				(g_TargetSpeed[0] == 0) &&
//				(g_TargetSpeed[1] == 0)
//			)
		)
	{
		return 0;
	}
	return 1;
}

//获取速度 编码器每时间分度
int16_t getEncoderVal(uint8_t cmd)
{
	if(cmd == 0)
	{
		return g_MotorEncoder[0];
	}
	else
	{	
		return g_MotorEncoder[1];
	}
}

//设定目的速度
void SetTargetSpeed(int16_t lift,int16_t right)
{
	g_TargetSpeed[0] = lift;
	g_TargetSpeed[1] = right;
}

//设定输出脉宽
void setMotorPWM(int16_t lift,int16_t right)
{
	//1000
	//左侧电机控制
	DianJi_TiaoSu(0x0a,lift);
	//右侧电机控制
	DianJi_TiaoSu(0x03,right);
}
/**
  * @brief  MotorSpeedPidCtrl.
  * @note		PWM动态函数采用算法“位置式离散比例积分微分方程”
  *         Out = Kp[e(k)]+ki∑e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
  *         其中积分项和微分项受大扰动误差阀值范围生效标志位控制
  * @retval None
  */
void MotorSpeedPidCtrl(void)
{
	static PIDDateBaseTypedef pid = {
		.de		[0] = 0,		.de		[1] = 0,
		.fe		[0] = 0,    .fe		[1] = 0,
		.de1	[0] = 0,    .de1	[1] = 0,
		.de2	[0] = 0,    .de2	[1] = 0,
		.out	[0] = 0,    .out	[1] = 0,
	};
	
	static int16_t 		lastspd[2]	 = {0,0};  //上一次的速度
	static uint32_t   stptick      = 0;      //停止时间
	
	for(int i = 0;i < 2;i++)
	{
		//pid计算
		
		//计算当前误差并移动历史误差
		pid.de2[i] 	 =  pid.de1[i];
		pid.de1[i] 	 =  pid.de[i];
		pid.de[i] 	 =  g_TargetSpeed[i] - getEncoderVal(i);
		pid.fe[i] 	+=  pid.de[i];
		//pid控制器核心方程
		pid.out[i] 	= 	PARAM_PID.kp * pid.de[i] 										+ 
										PARAM_PID.ki * pid.fe[i] * (pid.de[i] < 100) + 
										PARAM_PID.kd * ( pid.de[i] - 2 * pid.de1[i] + pid.de2[i]) * (pid.de[i] < 100);
		//输出限制幅度
		pid.out[i] = (pid.out[i] > MOTOR_OUTPUT_MAX) ? MOTOR_OUTPUT_MAX : pid.out[i];
		pid.out[i] = (pid.out[i] <-MOTOR_OUTPUT_MAX) ?-MOTOR_OUTPUT_MAX : pid.out[i];
	
				
		//振荡抑制
		
		//上一次速度不为0当前等于零
		if(lastspd[i] != 0 && g_TargetSpeed[i] == 0)
		{
			stptick = Sys_GetTick();
		}
		
		//目的速度为0且刹车时间超过3s
		if(g_TargetSpeed[i] == 0 && (Sys_GetTick() - stptick) > 3000)
		{
			pid.out[i] = pid.out[i]*2/3;
		}
	}
	
	//输出到电机控制
	setMotorPWM(pid.out[0],pid.out[1]);
}

//void senddebug(void)
//{
//	static uint32_t div = 0;
//	div++;
//	if(div > 50)
//	{
//		div = 0;
//		uint8_t str[50];
//		sprintf((char *)str,"%f,%d,%d\r\n",pid.de[1],g_TargetSpeed[1],pid.out[1]);
//		UsartSendData(str); 	
//	}
//}

//pid控制器执行周期分频器
void PID_Divider(void)
{
	static uint32_t div = 0;
	div++;
	if(div >= 10)
	{
		div = 0;
		updataEncoderVal();	//更新编码器数据
		MotorSpeedPidCtrl();//执行电机PID控制器
	}
//	senddebug();
}
















