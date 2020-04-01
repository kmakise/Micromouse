/**
  ******************************************************************************
  * @file    MoveCtrl.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-9-28
  * @brief   平台移动控制的整合与执行方法的再封装
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "MOveCtrl.h"
#include "attitude.h"
#include "pidctrller.h"
#include "odometer.h"
#include "parameter.h"
#include "InfraredCalc.h"

#define FASTENABLE 1

#define MCSPEED 700

/*平台的移动的再封装0
 *
 *	以下方法均为阻塞执行
 *	向前直线搜索
 *	原地旋转一个固定角度
 */
 
void mouseMove(MoveCmdTypedef move)
{
	
#ifdef FASTENABLE
	
	mouseMoveFastI(move);
	
#endif /*FASTENABLE*/
	
#ifndef FASTENABLE
	
	switch(move)
	{
		case MStop:
		{
			//设定速度
			SearcherSpeed(0,0);
			//进入停止模式
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			break;
		}
		case MForward:
		{
			//设定速度
			SearcherSpeed(MCSPEED,MCSPEED);
			//进入修正模式
			AttitudeStateSet(ASStraight);
			break;
		}
		case ML90:
		{
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			//设定角度
			RotationAngle(-90);
			//进入姿态调整模式
			AttitudeStateSet(ARotate);
			//等待调整结束
			while(getAttitudeState() != AStop);
			//等待电机停止
			while(getMotorState());
			//里程计姿态更新
			MouseDirUpdate(90);
			break;
		}
		case MR90:
		{
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			//设定角度
			RotationAngle(90);
			//进入姿态调整模式
			AttitudeStateSet(ARotate);
			//等待调整结束
			while(getAttitudeState() != AStop);
			//等待电机停止
			while(getMotorState());
			//里程计姿态更新
			MouseDirUpdate(-90);
			
			break;
		}
		case ML180:
		{
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			//设定角度
			RotationAngle(-180);
			//进入姿态调整模式
			AttitudeStateSet(ARotate);
			//等待调整结束
			while(getAttitudeState() != AStop);
			//等待电机停止
			while(getMotorState());
			//里程计姿态更新
			MouseDirUpdate(180);
			break;
		}
		case MR180:
		{
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			//设定角度
			RotationAngle(180);
			//进入姿态调整模式
			AttitudeStateSet(ARotate);
			//等待调整结束
			while(getAttitudeState() != AStop);
			//等待电机停止
			while(getMotorState());
			//里程计姿态更新
			MouseDirUpdate(-180);
			break;
		}
		default:break;
	}
	
#endif /*FASTENABLE*/

}

//移动控制 搜寻 快速优化第一版本
void mouseMoveFastI(MoveCmdTypedef move)
{
	switch(move)
	{
		case MStop://停止
		{
			//设定速度
			SearcherSpeed(0,0);
			//进入停止模式
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			break;
		}
		case MForward://前进
		{			
			//设定速度
			SearcherSpeed(MCSPEED,MCSPEED);
			//进入修正模式
			AttitudeStateSet(ASStraight);
			break;
		}
		case ML90://左转90度
		{
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			//设定角度
			RotationAngle(-90);
			//进入姿态调整模式
			AttitudeStateSet(ARotateFast);
			//等待调整结束
			while(RotateFastOver());
			//里程计姿态更新
			MouseDirUpdate(90);
			break;
		}
		case MR90://右转90度
		{
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			//设定角度
			RotationAngle(90);
			//进入姿态调整模式
			AttitudeStateSet(ARotateFast);
			//等待调整结束
			while(RotateFastOver());
			//里程计姿态更新
			MouseDirUpdate(-90);
			break;
		}
		case ML180://左转180度
		{
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			//设定角度
			RotationAngle(-180);
			//进入姿态调整模式
			AttitudeStateSet(ARotateFast);
			//等待调整结束
			while(RotateFastOver());
			//里程计姿态更新
			MouseDirUpdate(180);
			break;
		}
		case MR180://右转180度
		{
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			//设定角度
			RotationAngle(180);
			//进入姿态调整模式
			AttitudeStateSet(ARotateFast);
			//等待调整结束
			while(RotateFastOver());
			//里程计姿态更新
			MouseDirUpdate(-180);
			break;
		}
	}
}

//移动控制 冲刺 快速优化第一版本
void mouseMoveSP(MoveCmdTypedef move)
{
		switch(move)
	{
		case MStop://停止
		{
			//设定速度
			SearcherSpeed(0,0);
			//进入停止模式
			AttitudeStateSet(AStop);
			//等待电机停止
			while(getMotorState());
			break;
		}
		case MForward://前进
		{			
			
			break;
		}
		case MR90://右转90度
		{
			//while(getHalfGridIrEn());
			//设定旋转角度
			RotationAngle(90);
			//设定旋转模式
			AttitudeStateSet(AFRotate);
			//等待旋转结束
			while(getAttitudeState() == AFRotate);
			//里程计增量
			odometerPosAdd(1);
			//高速直线控制器增量
			set_FSLEtrnum(ODOMETER_STD_GRID_ETR_VAL * (1 - ATTITUDE_FROTATION_TPOS));
			//里程计姿态计入
			MouseDirUpdate(-90);
			setOdometerEtr(ODOMETER_STD_GRID_ETR_VAL * (1 - ATTITUDE_FROTATION_TPOS));
			
			setFastStMoveGridNum(15);
			AttitudeStateSet(AFStraight);
			break;
		}
		case ML90://左转90度
		{
			//while(getHalfGridIrEn());
			//设定旋转角度
			RotationAngle(-90);
			//设定旋转模式
			AttitudeStateSet(AFRotate);
			//等待旋转结束
			while(getAttitudeState() == AFRotate);
			//里程计增量
			odometerPosAdd(1);
			//高速直线控制器增量
			set_FSLEtrnum(ODOMETER_STD_GRID_ETR_VAL * (1 - ATTITUDE_FROTATION_TPOS));
			//里程计姿态计入
			MouseDirUpdate(90);
			setOdometerEtr(ODOMETER_STD_GRID_ETR_VAL * (1 - ATTITUDE_FROTATION_TPOS));
			
			setFastStMoveGridNum(15);
			AttitudeStateSet(AFStraight);
			break;
		}
		case ML180://左转180度
		{
			mouseMoveFastI(ML180);
			break;
		}
		case MR180://右转180度
		{
			mouseMoveFastI(ML180);
			break;
		}
	}
}

