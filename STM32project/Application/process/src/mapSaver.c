/**
  ******************************************************************************
  * @file    mapSaver.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-10-19
  * @brief   地图路径读取与保存程序
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "stmflash.h"
#include "mapSaver.h"
#include "myPlanner.h"


/*	需要保存信息以及地址分布
 *	 _______________________________________________
 *	|关于											|	类型				|	空间	|
 *	|—————————————————————————|—————————————|———————|
 *	|路径信息是否完整标志位 		|	uint8_t 		|	1			|
 *	|路径节点数量							|	uint8_t			|	1			|
 *	|路径坐标序列							|	uint8_t[][]	|	200		|
  *	|保留											|							|				|
 *	|_______________________________________________|
 *	合计大小：202字节
 *
 *  STM32F103RET6 FLASH Size 512K address 0x8000000 - 0x80800000
 *
 *	操作首地址 0x8080000 - 0x200 = 0x8079800	
 *	可写入空间 512字节 缓冲区占用202字节 剩余310字节
 *	
 */

/*Golbal Data Space ----------------------------------------------------------*/
#define FLASH_START_ADDRESS 		0X08010000
#define FLASH_BUFFER_SIZE				1024
#define FLASH_PATH_OVER_FLAG		0x55
#define FLASH_PATH_STATIC_FLAG  0xAA
#define FLASH_PATH_START				4

uint8_t flashCustomMap[FLASH_BUFFER_SIZE];		//flash读取与写入缓冲区 400字节


//读取flash到缓冲区	400字节
void readFlashToBuf(void)
{
	STMFLASH_Read(FLASH_START_ADDRESS,(u16 *)flashCustomMap,FLASH_BUFFER_SIZE);
}

//读取路径完整性使能标志位
uint8_t getPathOverFlag(void)
{
	//路径完整标志位被置位
	if(flashCustomMap[0] == FLASH_PATH_OVER_FLAG)
	{
		return 1;
	}
	return 0;
}

//读取固定路径使能标志位
uint8_t getStaticPathFlag(void)
{
	//已配置固定路径
	if(flashCustomMap[0] == FLASH_PATH_STATIC_FLAG)
	{
		return 1;
	}
	return 0;
}


//读取路径信息并将路径信息写入路径序列
void writePathSeqFromFlash(void)
{
	//设置路径节点数量
	setPathCurNum(flashCustomMap[1]);
	
	//设置目标点
	PosTypedef tg;
	
	tg.x = flashCustomMap[2];
	tg.y = flashCustomMap[3];
	
	setTargetPos(tg);
	
	//将路径信息写入路径序列
	for(int i = 0;i < 256; i++)
	{
		PosTypedef mp;
		mp.x = flashCustomMap[(i * 2 + 0) + FLASH_PATH_START];
		mp.y = flashCustomMap[(i * 2 + 1) + FLASH_PATH_START];
		
		setPathNode(i,mp);
	}
}


//清除路径信息
void delPathofFlash(void)
{
	//删除路径完成标志位
	flashCustomMap[0] = 0xFF;
	//将缓冲区写入flash
	STMFLASH_Write(FLASH_START_ADDRESS,(u16 *)flashCustomMap,FLASH_BUFFER_SIZE);
}


//写入路径信息并置位标志位
void writePathToFlash(void)
{
		//写入路径完成使能标志位到缓冲区
	flashCustomMap[0] = FLASH_PATH_OVER_FLAG;
	
	//写入路径节点数量到缓冲区
	flashCustomMap[1] = getPathCurNum();
	
	//写入目标点坐标到FLASH
	flashCustomMap[2] = getTargetPos().x;
	flashCustomMap[3] =	getTargetPos().y;
	
	
	//写入路径序列到缓冲区
	for(int i = 0;i < 256; i++)
	{
		PosTypedef mp = getRoute(i);
		
		flashCustomMap[(i * 2 + 0) + FLASH_PATH_START] = mp.x;
		flashCustomMap[(i * 2 + 1) + FLASH_PATH_START] = mp.y;
	}
	__asm("CPSID I");
	//将缓冲区写入flash
	STMFLASH_Write(FLASH_START_ADDRESS,(u16 *)flashCustomMap,FLASH_BUFFER_SIZE);
	__asm("CPSIE I");
}


//写入静态路径信息并置位标志位
void writeStaticPathToFlash(void)
{
		//写入路径完成使能标志位到缓冲区
	flashCustomMap[0] = FLASH_PATH_STATIC_FLAG;
	
	//写入路径节点数量到缓冲区
	flashCustomMap[1] = getPathCurNum();
	
	//写入目标点坐标到FLASH
	flashCustomMap[2] = getTargetPos().x;
	flashCustomMap[3] =	getTargetPos().y;
	
	
	//写入路径序列到缓冲区
	for(int i = 0;i < 256; i++)
	{
		PosTypedef mp = getRoute(i);
		
		flashCustomMap[(i * 2 + 0) + FLASH_PATH_START] = mp.x;
		flashCustomMap[(i * 2 + 1) + FLASH_PATH_START] = mp.y;
	}
	__asm("CPSID I");
	//将缓冲区写入flash
	STMFLASH_Write(FLASH_START_ADDRESS,(u16 *)flashCustomMap,FLASH_BUFFER_SIZE);
	__asm("CPSIE I");
}


