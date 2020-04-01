#ifndef __MOVECTRL_H
#define __MOVECTRL_H


typedef enum
{
	MStop,
	MForward,
	ML90,
	MR90,
	ML180,
	MR180,
	
}MoveCmdTypedef;

//移动控制基础版
void mouseMove(MoveCmdTypedef move);
//移动控制优化第一版
void mouseMoveFastI(MoveCmdTypedef move);
//移动控制 冲刺 快速优化第一版本
void mouseMoveSP(MoveCmdTypedef move);

#endif /*__MOVECTRL_H*/



