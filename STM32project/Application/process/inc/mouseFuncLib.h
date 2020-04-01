#ifndef __MOUSEFUNCLIB_H
#define __MOUSEFUNCLIB_H


#include "drive.h"


void LEDFlash(uint32_t num);
//快速自转测试
void TestFuncMoveFastTern(void);

void SendMaoToUsart(void);

//发送红外测试数据
void SendInfrared(void);

void ETR_Test(void);

//直线高速测试
void TestFuncMoveFast(void);

//发送代码
void sendSMCCode(void);

#endif /*__MOUSEFUNCLIB_H*/



