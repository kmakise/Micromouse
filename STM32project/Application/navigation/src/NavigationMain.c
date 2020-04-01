/**
  ******************************************************************************
  * @file    NavigationMain.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-9-28
  * @brief   导航组主方法
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "NavigationMain.h"
#include "odometer.h"
#include "OccupyingGrid.h"
#include "ErgodicMap.h"


void NavigaSetup(void)
{
	GridSetup();
	MapCreatorInit();
}

void NavigaLoop(void)
{
	//MapCreatLoop();
	
}
void NavigaInterrupt(void)
{
	odometerLoop();
	
}










