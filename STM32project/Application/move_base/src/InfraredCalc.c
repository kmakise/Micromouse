/**
  ******************************************************************************
  * @file    InfraredCalc.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-9-22
  * @brief   红外传感器数据采集，处理算法和环境感知抽象层
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "InfraredCalc.h"
#include "parameter.h"

#include "drive.h"

#include "stdio.h"

/*Golbal data space ----------------------------------------------------------*/
RedValTypedef g_RedSenVal;//红外扫描结果每20ms更新一次

/*偏移量获取参数----------------------------------------------------*/
const int16_t BASE_L = INFRAREEDCALC_BASE_L;			//左侧基础量 在中心时保持零
const int16_t BASE_R = INFRAREEDCALC_BASE_R;			//右侧基础量 在中心时保持零

const int16_t MISS_L = INFRAREEDCALC_MISS_L - INFRAREEDCALC_BASE_L;			//左侧偏移终止阈值	左侧小于该值时放弃该侧的权衡 -650 + 900
const int16_t MISS_R = INFRAREEDCALC_MISS_R	- INFRAREEDCALC_BASE_R;			//右侧偏移终止阈值	右侧小于该值时放弃该侧的权衡 -730 + 900

const int16_t LOST_L = INFRAREEDCALC_LOST_L - INFRAREEDCALC_BASE_L;			//左侧环境侦测丢失阈值	左侧小于该值时放弃该侧的权衡 -650 + 900
const int16_t LOST_R = INFRAREEDCALC_LOST_R - INFRAREEDCALC_BASE_R;			//右侧环境侦测丢失阈值	右侧小于该值时放弃该侧的权衡 -730 + 900

/*抵近检测获取参数---------------------------------------------------*/
const int16_t OFFSET_LF = INFRAREEDCALC_OFFSET_LF;		//左侧偏置
const int16_t OFFSET_RF = INFRAREEDCALC_OFFSET_RF;		//右侧偏置

const int16_t ACTVAL_LF = INFRAREEDCALC_ACTVAL_LF;	//左侧激活值
const int16_t ACTVAL_RF = INFRAREEDCALC_ACTVAL_RF;	//右侧激活值

const int16_t THRESHOLDS = INFRAREEDCALC_THRESHOLDS;	//和阈值0
/*------------------------------------------------------------------*/



#define RED_DELAY delay_us(1)

//获得周围传感器的值
RedValTypedef getRedVal(void)
{
	RedValTypedef red;
	
	//获得环境值
	Adc_Transition();
	red.LS = ADC_MeasureBuf[2];
	red.RS = ADC_MeasureBuf[3];
	red.LF = ADC_MeasureBuf[0];
	red.RF = ADC_MeasureBuf[1];
	
	
	//计算各个传感器的值
	Infrared_Send_Control(LS); 
	RED_DELAY;
	Adc_Transition();
	red.LS = ADC_MeasureBuf[2] - red.LS;
	red.LS = (red.LS < 4096) ? red.LS : 0;
	
	
	Infrared_Send_Control(RS); 
	RED_DELAY;
	Adc_Transition();
	red.RS = ADC_MeasureBuf[3] - red.RS;
	red.RS = (red.RS < 4096) ? red.RS : 0;
	
	Infrared_Send_Control(LF);
	RED_DELAY;
	Adc_Transition();
	red.LF = ADC_MeasureBuf[0] - red.LF;
	red.LF = (red.LF < 4096) ? red.LF : 0;
	
	Infrared_Send_Control(RF); 
	RED_DELAY;
	Adc_Transition();
	red.RF = ADC_MeasureBuf[1] - red.RF;
	red.RF = (red.RF < 4096) ? red.RF : 0;

	//关闭全部传感器
	Infrared_Send_Control(0);
	
//	uint8_t str[50];
//	sprintf((char *)str,"%d,%d,%d,%d \r\n",red.LS,red.RS,red.LF,red.RF);
//	UsartSendData(str); 
//	
	return red;
}


//获得偏移量
int16_t getRedOffset(void)
{
	//计算原理
	//偏移量左 = 左侧基础 - 左侧数值
	//偏移量右 = 右侧基础 - 右侧数值
	//偏移量 = 偏移量左 - 偏移量右
	
	
	
	int16_t lift_val  = g_RedSenVal.LS - BASE_L;
	int16_t right_val = g_RedSenVal.RS - BASE_R;
	
	int16_t val = 0;
	
	val = lift_val - right_val;
	
	//侧向无反馈的环境感知传感器组策略
	//条件：一侧返回值严重低于正常值则认定为无返回
	//策略：偏移量则只对一侧进行权衡
	//条件：两侧返回值均低于正常值
	//策略：位置当前姿态不做改变
	if(lift_val < MISS_L)//左侧失去返回
	{
		val = -right_val;
	}
	if(right_val < MISS_R)//右侧失去返回
	{
		val = lift_val;
	}
	if((lift_val < MISS_L) && (right_val < MISS_R))//同时失去返回
	{
		val = 0;
	}

	
	
//	uint8_t str[50];
//	sprintf((char *)str,"%d,%d     ",lift_val,right_val);
//	UsartSendData(str); 
//	sprintf((char *)str,"%d\r\n",val);
//	UsartSendData(str); 
//	LEDDisplay(lift_val,right_val);
//	LEDDisplay(val,0);
	
	return val;
}



//障碍物抵近触发方法
uint8_t getBarrierAct(void)
{
	//预处理 返回值做偏置
	//激活条件
	//传感器左侧与右侧均大于激活值
	//数值和大于和阈值
	
	int16_t adcVal_LF = g_RedSenVal.LF + OFFSET_LF;
	int16_t adcVal_RF = g_RedSenVal.RF + OFFSET_RF;
	
	if( ((adcVal_LF > ACTVAL_LF) && ((adcVal_LF + adcVal_RF) > THRESHOLDS)) ||
			((adcVal_RF > ACTVAL_RF) && ((adcVal_LF + adcVal_RF) > THRESHOLDS))
			)
	{
		return 1;
	}
	return 0;
}



//获得当前坐标的可活动信息
uint8_t getActDirState(void)
{
	//返回状态用而进制表示三个方向的可活动性
	//0000 0LFR L左侧 F前方 R右侧
	const uint16_t LFACTVAL = INFRAREEDCALC_EN_LFACTVAL;	//左前激活阈值
	const uint16_t RFACTVAL = INFRAREEDCALC_EN_RFACTVAL;	//右前激活阈值
	const uint16_t FSACTVAL = INFRAREEDCALC_EN_FSACTVAL;	//前和激活阈值
	
	int16_t lift_val  = g_RedSenVal.LS - BASE_L;
	int16_t right_val = g_RedSenVal.RS - BASE_R;
	
	uint8_t actdir = 0x00;
	
	if(lift_val < LOST_L)//左侧失去返回
	{
		actdir |= 0x04;
	}
	if(right_val < LOST_R)//右侧失去返回
	{
		actdir |= 0x01;
	}
	
	int16_t redsum = g_RedSenVal.LF + g_RedSenVal.RF;
	//前方触发
	if((g_RedSenVal.LF > LFACTVAL && (redsum > FSACTVAL)) ||
		 (g_RedSenVal.RF > RFACTVAL && (redsum > FSACTVAL)) )
	{
		actdir |= 0x02;
	}
	return actdir;
}
//int16_t pianyizhi = 0;
void InfraredLoop(void)
{
	static uint32_t div = 0;
	div++;
	if(div >= 10)
	{
		div = 0;
		g_RedSenVal = getRedVal();
		//pianyizhi = getRedOffset();
	}
}


////获得就即将抵近1/2栅格的状态
//uint8_t getHalfGridIrEn(void)
//{
//	const uint16_t LFACTIVE =  200;
//	const uint16_t RFACTIVE =  400;
//	const uint16_t ENSUM =  600;
//	
//	const uint16_t LFDIS =  500;
//	const uint16_t RFDIS =  1400;
//	const uint16_t ENDIS =  1600;
//	
//	//前方可以作为修正基准
//	if((g_RedSenVal.LF > LFACTIVE && (g_RedSenVal.LF + g_RedSenVal.RF) > ENSUM)||
//		 (g_RedSenVal.RF > RFACTIVE && (g_RedSenVal.LF + g_RedSenVal.RF) > ENSUM))
//	{
//		//抵达要求位置
//		if((g_RedSenVal.LF > LFDIS && (g_RedSenVal.LF + g_RedSenVal.RF) > ENDIS)||
//			(g_RedSenVal.RF > RFDIS && (g_RedSenVal.LF + g_RedSenVal.RF) > ENDIS))
//		
//		{
//			return 0;
//		}
//		return 1;
//	}
//	//否则不作为修正
//	return 0;
//	
//}

//获得红外触发编号
uint8_t getInfraredEventVal(void)
{
	//红外触发位
	uint8_t actbit = 0;
	if(g_RedSenVal.LF > 2800)actbit |= 0x01;
	if(g_RedSenVal.LS > 2800)actbit |= 0x02;
	if(g_RedSenVal.RS > 3700)actbit |= 0x04;
	if(g_RedSenVal.RF > 2800)actbit |= 0x08;
	return actbit;
}













