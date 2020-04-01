/**
  ******************************************************************************
  * @file    attitude.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-9-23
  * @brief   运行过程中的动态姿态调整与特殊运动的执行
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "attitude.h"
#include "InfraredCalc.h"
#include "pidctrller.h"
#include "parameter.h"
#include "odometer.h"

#include	"drive.h"
#include "stdio.h"
/*Golbal data space ----------------------------------------------------------*/
SpeedTypedef 		g_SearcherSpeed = {.lift = 0,.right = 0,};//搜寻模式直线行进的速度设定
AttStaTypedef		g_AttState = AStop;												//姿态修正状态状态
int16_t 				g_rotateDegrees = 0;											//姿态调整角度
int8_t 					g_rotateFast = 0;													//快速旋转完成标志位
int16_t 				g_FSLGridnum = 0;													//快速移动控制的位移数量


//搜寻模式直线搜索时的运行速度设定
void SearcherSpeed(uint16_t lift,uint16_t right)
{
	g_SearcherSpeed.lift = lift;
	g_SearcherSpeed.right = right;
}


//姿态修正状态机状态配置
void AttitudeStateSet(AttStaTypedef ast)
{
	g_AttState = ast;
}
//姿态修正状态状态获取
AttStaTypedef getAttitudeState(void)
{
	return g_AttState;
}

//搜寻时直线运行的姿态修正
void SearcherPidCtrller(void)
{
	//直线姿态修正方式 采用偏移量比例速度修正方式
	//out = target - k * offset
	//偏移量 负值 左侧 - 偏移量 右侧 + 偏移量
	
	//偏移量值域 速度的  -60% ~ 60%
	
	SpeedTypedef speed;//输出速度
	
	int16_t offset = getRedOffset();//偏移量
	float kp = 0.0; //比例控制器动态参数
	
	//比例参数取值目的：限制修正值域到速度百分比的-+60% 
	//比例方程 3000 * kp = 0.6 * Speed，kp = (0.6 * Speed) / 3000 
	kp = (ATTITUDE_STRAIGHT_KP * 
				(float)((g_SearcherSpeed.lift + g_SearcherSpeed.right) / 2)) / ATTITUDE_STRAIGHT_INE_MAX;
	
	//误差幅度限制
	offset = (offset >  ATTITUDE_STRAIGHT_INE_MAX) ?  ATTITUDE_STRAIGHT_INE_MAX : offset;
	offset = (offset < -ATTITUDE_STRAIGHT_INE_MAX) ? -ATTITUDE_STRAIGHT_INE_MAX : offset;
	
	speed.lift  = g_SearcherSpeed.lift + (kp * (float)(offset));//左侧速度修正
	speed.right = g_SearcherSpeed.right - (kp * (float)(offset));//右侧速度修正
	
	//发布速度到速度闭环pid控制器
	SetTargetSpeed(speed.lift,speed.right);
	
//	//调试信息
//	uint8_t str[50];
//	sprintf((char *)str,"%d,%d\r\n",speed.lift,speed.right);
//	UsartSendData(str); 
	
}

//搜寻模式 直线姿态修正控制器 执行周期分频器
void SPC_Divider(void)
{
	static uint16_t div = 0;
	div++;
	if(div >= 10)//分频器10分频s
	{
		div = 0;
		SearcherPidCtrller();
	}
}
/*Move fast -----------------------------------------------------------------*/
int32_t etrtemp[2] =  {0,0};

void FSL_OverDataDel(void)
{
		g_FSLGridnum = 0;
		etrtemp[0]	 = 0;
		etrtemp[1]	 = 0;
}
void set_FSLEtrnum(int32_t num)
{
		etrtemp[0]	 = num;
		etrtemp[1]	 = num;
}


//快速直线运动位移控制器
void FSL_moveController(void)
{
	SpeedTypedef fs;

	
	//累积编码器
	etrtemp[0] += getEncoderVal(0);
	etrtemp[1] += getEncoderVal(1);
	
	//比例以及误差
	float xkp = ATTITUDE_STRAIGHT_FAST_KP;
	float et0 = ((g_FSLGridnum * ODOMETER_STD_GRID_ETR_VAL / 2) - etrtemp[0]);
	float et1 = ((g_FSLGridnum * ODOMETER_STD_GRID_ETR_VAL / 2) - etrtemp[1]);	
	
	//位置点的速度计算
	//位置比例控制式的速度计算
	fs.lift  = xkp * et0;
	fs.right = xkp * et1;
	
	//速度幅度限制
	fs.lift  = (fs.lift  > ATTITUDE_STRAIGHT_SPD_LTD) ? ATTITUDE_STRAIGHT_SPD_LTD : fs.lift;
	fs.right = (fs.right > ATTITUDE_STRAIGHT_SPD_LTD) ? ATTITUDE_STRAIGHT_SPD_LTD : fs.right;
																					 
	fs.lift  = (fs.lift  < -ATTITUDE_STRAIGHT_SPD_LTD) ? -ATTITUDE_STRAIGHT_SPD_LTD : fs.lift;
	fs.right = (fs.right < -ATTITUDE_STRAIGHT_SPD_LTD) ? -ATTITUDE_STRAIGHT_SPD_LTD : fs.right;
	
	//在前25%时保证低输出以确保高速前的航向修正
	//0-25%里程对应速度输出关系 0-100% 
	float ksl = ((float)etrtemp[0] / (float)(g_FSLGridnum * ODOMETER_STD_GRID_ETR_VAL / 2)) / 0.2;
	float ksr = ((float)etrtemp[1] / (float)(g_FSLGridnum * ODOMETER_STD_GRID_ETR_VAL / 2)) / 0.2;
	
	//在后25%低功率输出确保停机误差在可接受范围内
	float kel = (et0 / (float)(g_FSLGridnum * ODOMETER_STD_GRID_ETR_VAL / 2)) / 0.1;
	float ker = (et1 / (float)(g_FSLGridnum * ODOMETER_STD_GRID_ETR_VAL / 2)) / 0.1;
	
	//ks 幅度限制
	ksl = (ksl > 1.0) ? 1.0 : ksl;
	ksr = (ksr > 1.0) ? 1.0 : ksr;
	
	//ke 幅度限制
	kel = (kel > 1.0) ? 1.0 : kel;
	ker = (ker > 1.0) ? 1.0 : ker;
	//预先和结束限制加速控制
	fs.lift = fs.lift * ksl * kel;
	fs.right = fs.right * ksr * ker;
	
	//速度静态偏置
	fs.lift  = fs.lift + ATTITUDE_STRAIGHT_FAST_SET;
	fs.right = fs.right + ATTITUDE_STRAIGHT_FAST_SET;
	
	//直线姿态修正方式 采用偏移量比例速度修正方式
	//out = target - k * offset
	//偏移量 负值 左侧 - 偏移量 右侧 + 偏移量
	
	//偏移量值域 速度的  -60% ~ 60%
	
	int16_t offset = getRedOffset();//偏移量
	
	//比例动态参数的速度值域范围的动态调整 650-MAX = 0.3 - 0.1
	float kv = 0.1 + 0.2 * (
														(float)(
																			(ATTITUDE_STRAIGHT_SPD_LTD + ATTITUDE_STRAIGHT_FAST_SET) -
																			(((float)(getEncoderVal(0) + getEncoderVal(1)) / 2))
																		)
														/ //--------------------------------------------------------------------------
														(float)(
																				ATTITUDE_STRAIGHT_SPD_LTD + ATTITUDE_STRAIGHT_FAST_SET
																		)
													);
														
	//比例参数取值目的：限制修正值域到速度百分比的-+60% 
	//比例方程 3000 * kp = 0.6 * Speed，kp = (0.6 * Speed) / 3000 
	float kp = (kv * (float)((fs.lift + fs.right) / 2)) / ATTITUDE_STRAIGHT_INE_MAX;
	
	//误差幅度限制
	offset = (offset >  ATTITUDE_STRAIGHT_INE_MAX) ?  ATTITUDE_STRAIGHT_INE_MAX : offset;
	offset = (offset < -ATTITUDE_STRAIGHT_INE_MAX) ? -ATTITUDE_STRAIGHT_INE_MAX : offset;
	
	fs.lift = fs.lift + (kp * (float)(offset));//左侧速度修正
	fs.right = fs.right - (kp * (float)(offset));//右侧速度修正
	
	//发布速度到速度闭环pid控制器
	SetTargetSpeed(fs.lift,fs.right);

}

//设定目标位移栅格数量
void setFastStMoveGridNum(uint16_t num)
{
	g_FSLGridnum = num;
}

//直线运动控制分频器
void FSL_DividerFast(void)
{
	static uint16_t div = 0;
	div++;
	if(div >= 10)//分频器10分频ms
	{
		div = 0;
		FSL_moveController();
	}
}

/*Rotation ---------------------------------------------------------------*/

#define MAXSPEED 				ATTITUDE_ROTATION_OUT_MAX
#define POSD 						ATTITUDE_ROTATION_LOST_DR

//旋转编码器数量定义
const int32_t P90D 		= ATTITUDE_ROTATION_ETR__90;	//+90
const int32_t D90D 		= -ATTITUDE_ROTATION_ETR__90;	//-90
const int32_t P180D 	= ATTITUDE_ROTATION_ETR_180;	//+180
const int32_t D180D 	= -ATTITUDE_ROTATION_ETR_180;	//-180
const float		WKP			= ATTITUDE_ROTATION_KP_VALE;	//比例控制器权重


//设定旋转角度
void RotationAngle(int16_t angel)
{
	g_rotateDegrees = angel;
}

//旋转控制器 需要数次执行完成 完成后自动修改姿态调整状态 到 停止
void RotateCtrller(int16_t de)
{

	static int32_t encoder[2] = {0,0};	//编码器增量
	
	int32_t speed[2] = {0,0};
	int32_t angel = 0;//目的角度
	
	switch(de)
	{
		case 90  	: angel = P90D; break;
		case -90 	: angel = D90D;break;
		case 180 	: angel = P180D;break;
		case -180 : angel = D180D;break;
	}
	
	//读取编码器增量并记录
	encoder[0] += getEncoderVal(0);
	encoder[1] -= getEncoderVal(1);
	
	//旋转姿态控制 采用比例位置式控制器
	speed[0] = WKP * (angel/2  - encoder[0]);
	speed[1] = WKP * (angel/2  - encoder[1]);

	
	//速度幅度限制
	speed[0] = (speed[0] > MAXSPEED) ? MAXSPEED : speed[0];
	speed[1] = (speed[1] > MAXSPEED) ? MAXSPEED : speed[1];
	
	speed[0] = (speed[0] < -MAXSPEED) ? -MAXSPEED : speed[0];
	speed[1] = (speed[1] < -MAXSPEED) ? -MAXSPEED : speed[1];
	
	//目的角度完成判断
	if((-POSD < ((angel/2) - encoder[0])) && (((angel/2) - encoder[0]) < POSD) &&
		 (-POSD < ((angel/2) - encoder[1])) && (((angel/2) - encoder[1]) < POSD))
	{
		//变更姿态调整状态
		AttitudeStateSet(AStop);
		//清除编码器增量记录
		encoder[0] = 0;
		encoder[1] = 0;
		//退出控制器
		return;
	}
	
	//发布速度
	SetTargetSpeed(speed[0],-speed[1]);
	
//	uint8_t str[50];
//	sprintf((char *)str,"%d,%d,%d,%d \r\n",red.LS,red.RS,red.LF,red.RF);
//	UsartSendData(str); 
}

//原地 姿态调整控制器执行周期分频器
void RDD_Divider(void)
{
	static uint16_t div = 0;
	div++;
	if(div >= 10)//分频器20分频s
	{
		div = 0;
		RotateCtrller(g_rotateDegrees);
	}
}

//旋转控制器 优化版本
void RotateCtrllerFast(int16_t de)
{
	static int32_t encoder[2] = {0,0};	//编码器增量
	
	int32_t speed[2] = {0,0};
	int32_t angel[2] = {0,0};//目的角度
	
	switch(de)
	{
		case 90  	: 
			angel[0] = P90D * ATTITUDE_ROTATION_FAST_VB * 0.5;
			angel[1] = P90D * ATTITUDE_ROTATION_FAST_VB * 0.5;
		break;
		case -90 	: 
			angel[0] = D90D * ATTITUDE_ROTATION_FAST_VB * 0.5;
			angel[1] = D90D * ATTITUDE_ROTATION_FAST_VB * 0.5;
		break;
		case 180 	: 
			angel[0] = P180D * ATTITUDE_ROTATION_FAST_VB * 0.5;
			angel[1] = P180D * ATTITUDE_ROTATION_FAST_VB * 0.5;
		break;
		case -180 : 
			angel[0] = D180D * ATTITUDE_ROTATION_FAST_VB * 0.5;
			angel[1] = D180D * ATTITUDE_ROTATION_FAST_VB * 0.5;
		break;
	}
	
	//读取编码器增量并记录
	encoder[0] += getEncoderVal(0) * 2;
	encoder[1] -= getEncoderVal(1) * 2;
	
	//旋转姿态控制 采用比例位置式控制器
	speed[0] = WKP * (angel[0]  - encoder[0]);
	speed[1] = WKP * (angel[1]  - encoder[1]);
	
	//速度幅度限制
	speed[0] = (speed[0] > MAXSPEED) ? MAXSPEED : speed[0];
	speed[1] = (speed[1] > MAXSPEED) ? MAXSPEED : speed[1];
	
	speed[0] = (speed[0] < -MAXSPEED) ? -MAXSPEED : speed[0];
	speed[1] = (speed[1] < -MAXSPEED) ? -MAXSPEED : speed[1];
	
	//目的角度完成判断
	if((-POSD < ((angel[0]) - encoder[0])) && (((angel[0]) - encoder[0]) < POSD) &&
		 (-POSD < ((angel[1]) - encoder[1])) && (((angel[1]) - encoder[1]) < POSD))
	{
		//清除编码器增量记录
		encoder[0] = 0;
		encoder[1] = 0;
		//退出控制器
		g_rotateFast = 1;//结束
		return;
	}
	
	//发布速度
	SetTargetSpeed(speed[0],-speed[1]);
}

uint8_t RotateFastOver(void)
{
	uint8_t temp = g_rotateFast;
	if(temp == 1 && g_rotateFast == 1)
	{
		g_rotateFast = 0;
		return 0;
	}
	return 1;
}

//原地 姿态调整控制器执行周期分频器
void RDD_DividerFast(void)
{
	static uint16_t div = 0;
	div++;
	if(div >= 10)//分频器20分频s
	{
		div = 0;
		RotateCtrllerFast(g_rotateDegrees);
	}
}

/*turn move ctrl sp ---------------------------------------------------------*/
//2916.3etr/cm
const int32_t L1ETRSUM = ATTITUDE_FROTATION_ETR_L1; 	//120mm
const int32_t L2ETRSUM = ATTITUDE_FROTATION_ETR_L2;	//55mm

//左转90度
void RotateMoveFPL90(void)
{
	//位移闭环
	static int32_t etrsum[2] = {0,0,};
	
	//记录编码器增量
	etrsum[0] += getEncoderVal(0);
	etrsum[1] += getEncoderVal(1);
	
	//位移闭环速度控制比例参数
	float kpx = ATTITUDE_FROTATION_X_KP;	//max 1500
	
	//误差
	float et0 = ((float)L2ETRSUM - (float)etrsum[0]);
	float et1 = ((float)L1ETRSUM - (float)etrsum[1]);
	
	//结束判断
	if(et0 < ATTITUDE_FROTATION_ETR_ET && et0 > -ATTITUDE_FROTATION_ETR_ET &&
		 (et1 / 1) < ATTITUDE_FROTATION_ETR_ET && 
		 (et1 / 1) > -ATTITUDE_FROTATION_ETR_ET)
	{
		etrsum[0] = 0;
		etrsum[1] = 0;
		AttitudeStateSet(AStop);
		return ;
	}
	//计算当前点的速度
	SpeedTypedef rs;
	rs.lift  = kpx * et0;
	rs.right = kpx * et1;
	
	rs.lift  *= 1;
	rs.right *= ATTITUDE_FROTATION_CRP;
	
	rs.right = (rs.right > 3000)?3000:rs.right;
	
	//发布速度
	SetTargetSpeed(rs.lift,rs.right);
}
//右转90度
void RotateMoveFPR90(void)
{
	//位移闭环
	static int32_t etrsum[2] = {0,0,};
	
	//记录编码器增量
	etrsum[0] += getEncoderVal(0);
	etrsum[1] += getEncoderVal(1);
	
	//位移闭环速度控制比例参数
	float kpx = ATTITUDE_FROTATION_X_KP;	//max 1500
	
	//误差
	float et0 = ((float)L1ETRSUM - (float)etrsum[0]);
	float et1 = ((float)L2ETRSUM - (float)etrsum[1]);

	//结束判断
	if((et0 / 1) < ATTITUDE_FROTATION_ETR_ET && 
		 (et0 / 1) > -ATTITUDE_FROTATION_ETR_ET &&
		 et1 < ATTITUDE_FROTATION_ETR_ET && et1 > -ATTITUDE_FROTATION_ETR_ET)
	{
		etrsum[0] = 0;
		etrsum[1] = 0;
		AttitudeStateSet(AStop);
		return ;
	}
	//计算当前点的速度
	SpeedTypedef rs;
	rs.lift  = kpx * et0;
	rs.right = kpx * et1;
	
	rs.lift  *= ATTITUDE_FROTATION_CRP;
	rs.right *= 1;
	
	
	rs.lift = (rs.lift > 3000)?3000:rs.lift;
	
	
	//发布速度
	SetTargetSpeed(rs.lift,rs.right);
}


void RotateCtrllerFastI(int16_t de)
{
	switch(de)
	{
		case 90:
		{
			RotateMoveFPR90();
			break;
		}
		case -90:
		{
			RotateMoveFPL90();
			break;
		}
		default:break;
	}
}



//特殊转向运动控制分频器
void FTR_DividerFast(void)
{
	static uint16_t div = 0;
	div++;
	if(div >= 10)//分频器10分频ms
	{
		div = 0;
		RotateCtrllerFastI(g_rotateDegrees);
	}
}
/*State Machine -------------------------------------------------------------*/

//姿态修正状态机
void AttitudeStateMachine(void)
{
	
	//
	switch(g_AttState)
	{
		case AStop://停止状态 停止电机运行并维持停止状态
		{
			SetTargetSpeed(0,0);
			break;
		}
		case AWaiting://等待状态 对速度闭环不做任何操作
		{
			break;
		}
		
		/*---------------------------------------------------------直线*/
		case ASStraight://搜寻模式 直线运行的动态姿态修正
		{
			SPC_Divider();
			break;
		}
		case AFStraight://直线加速模式
		{
			FSL_DividerFast();
			break;
		}
		/*---------------------------------------------------------转向*/
		case ARotate://原地 姿态调整 旋转指定角度 
		{
			RDD_Divider();
			break;
		}
		case ARotateFast://原地 姿态调整 优化计算第一版本
		{
			RDD_DividerFast();
			break;
		}
		case AFRotate:	//快速转弯
		{
			FTR_DividerFast();
			break;
		}
		default:break;
	}	
	
	//加速直线位移数据清除
	if(g_AttState != AFStraight)
	{
		FSL_OverDataDel();
	}
}



















