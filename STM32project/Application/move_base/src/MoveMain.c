/**
  ******************************************************************************
  * @file    MoveMain.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-9-21
  * @brief   运动基础组主方法组
  ******************************************************************************
  */
	
/*include file ---------------------------------------------------------------*/
#include "MoveMain.h"
#include "pidctrller.h"
#include "InfraredCalc.h"
#include "attitude.h"
#include "MoveCtrl.h"
#include "odometer.h"

#include "drive.h"
#include "stdio.h"


extern PosTypedef MousePos;
extern int32_t etrsum;
extern RedValTypedef g_RedSenVal;

void path(void)
{
	#define L 0
	#define R 1
	#define B 3
	//											0  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37
	const uint8_t x[50] = {15,15, 2,12, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 6, 3, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, };
	const uint8_t t[50] = { R, R, R, L, L, R, R, L, R, L, L, R, L, R, R, L, R, L, L, R, L, L, L, L, R, L, R, R, R, L, R, L, L, R, R, B,};
	
	static int16_t xt = 0;
	static int16_t cur = 0;
	
	if(getPosState())
	{
//		uint8_t str[50];
//		sprintf((char *)str,"x = %d,y = %d,th = %d\r\n",MousePos.x,MousePos.y,MousePos.th);
//		UsartSendData(str); 
		
		xt++;
		if(xt == x[cur])
		{
			//MapActivityRecord();
			//进入停止模式
			AttitudeStateSet(AStop);
			xt = 0;
			if(t[cur] == R)
			{
				mouseMove(MR90);
			}
			else if(t[cur] == L)
			{
				mouseMove(ML90);
			}
			else if(t[cur] == B)
			{
				mouseMove(ML180);
			}
			
			cur++;
			while(cur == 36);
		}
		mouseMove(MForward);
	}
	
	
}


void MoveSetup(void)//process setup
{
	
//	while(1)
//	{
//		LEDDisplay(g_RedSenVal.LF,g_RedSenVal.RF);
//	}
	
//	while(1)
//	{
//		RotationAngle(90);
//		AttitudeStateSet(Rotate);
//		while(g_AttState != Stop);
//		delay_ms(1000);
//	}
	
//	delay_ms(2000);
//	mouseMove(MForward);
}

void MoveLoop(void)//process loop
{
	
//	if( getBarrierAct())
//	{
//		mouseMove(MStop);
//		
//		uint8_t str[50];
//		sprintf((char *)str,"etr = %d\r\n",etrsum);
//		UsartSendData(str); 
//		
//		while(1);
//	}
//		path();
}

void MoveInterrupt(void)//process interrupt 1ms
{	
	PID_Divider();
	InfraredLoop();
	AttitudeStateMachine();
}

