#ifndef __NAVIGATOR_H
#define __NAVIGATOR_H

#include "odometer.h"

//通过下一个目标点计算方向
uint8_t navPosToDir(PosTypedef mp,PosTypedef nmp);
//按照引导方向移动
void moveNavigator(uint8_t dir,PosTypedef mp,void (* func)());
//配置高速直线控制器
void setLineFastI(uint8_t dis);
//获得两点之间的距离
uint8_t getDisGrid(PosTypedef mp,PosTypedef nmp);

//移动导航测试版
void NavigatorTest(void);

//移动导航静态路径
void NavigatorStaticPath(void);

//移动导航加速第一版本
void NavigatorFastI(void);
//移动导航加速第二版本
void NavigatorFastII(void);


#endif /*__NAVIGATOR_H*/
