#ifndef __MAPSAVER_H
#define __MAPSAVER_H

#include "odometer.h"

//读取flash到缓冲区	400字节
void readFlashToBuf(void);
//读取路径完整性使能标志位
uint8_t getPathOverFlag(void);
//读取固定路径使能标志位
uint8_t getStaticPathFlag(void);
	
//读取路径信息并将路径信息写入路径序列
void writePathSeqFromFlash(void);
//清除路径信息
void delPathofFlash(void);
//写入路径信息并置位标志位
void writePathToFlash(void);
//写入静态路径信息并置位标志位
void writeStaticPathToFlash(void);
#endif /*__MAPSVER_H*/
