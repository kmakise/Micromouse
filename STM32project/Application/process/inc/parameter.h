#ifndef __PARAMETER_H
#define __PARAMETER_H

//重要参数宏定义集合 2019/10/10 ver.1.0.0 张东

/* 更换平台后的调整策略 2019/10/12
 *
 * 1.姿态调整 证实误差在可接受范围内												[直线状况、自转角度]
 * 2.运动控制 证实无里程计修正时的正确运行										[屏蔽里程计修正 单纯依靠编码器执行固定路径] 
 * 3.环境感知 证实在运动和姿态无任何问题时的正确环境感知
 * 4.速度优化 确认机能正常时的速度调整
 * 5.自动修正 启用并调整里程计的前方障碍位置辅助修正
 *
 */
 //300
 //700
 
/*=================================================红外传感器系列参数*/


/*--------------------偏移值相关参数*/
/*使用近基础值计算偏移量 使用中间值作为偏移放弃值 使用远距离值作为环境侦测依据*/

#define INFRAREEDCALC_BASE_L				1500//1600//1300		 //左侧基础量 	在中心时保持和观测值差为零 900
#define INFRAREEDCALC_BASE_R				2800//2900//2700     //右侧基础量 	在中心时保持和观测值差为零 900

#define INFRAREEDCALC_MISS_L			  400   	 //左侧偏移终止阈值	左侧小于该值时放弃该侧的权衡 -650 + 900
#define INFRAREEDCALC_MISS_R			  700	 	 //右侧偏移终止阈值	右侧小于该值时放弃该侧的权衡 -730 + 900

#define INFRAREEDCALC_LOST_L			  400      //左侧环境侦测丢失阈值	左侧小于该值时放弃该侧的权衡 -650 + 900
#define INFRAREEDCALC_LOST_R			  700  	 	 //右侧环境侦测丢失阈值	右侧小于该值时放弃该侧的权衡 -730 + 900

/*---------------------抵近障碍的触发条件参数 用于里程计的修正 [调整到即将碰撞的数值]*/
#define INFRAREEDCALC_OFFSET_LF			0				//左侧偏置
#define INFRAREEDCALC_OFFSET_RF			0 			//右侧偏置

#define INFRAREEDCALC_ACTVAL_LF			3000	//左侧激活值
#define INFRAREEDCALC_ACTVAL_RF			3200    //右侧激活值

#define INFRAREEDCALC_THRESHOLDS		6000		//和阈值

/*--------------------前方障碍感知*/
#define INFRAREEDCALC_EN_LFACTVAL		1050		//左前激活阈值		当左侧返回数值小于该值时则为丢失	250
#define INFRAREEDCALC_EN_RFACTVAL		1450 		//右前激活阈值		当右侧返回数值小于该值时则为丢失	250
#define INFRAREEDCALC_EN_FSACTVAL		2000    //前和激活阈值		当前方返回数值大于且都大于该值时则为触发 800


/*=================================================编码器常量系列参数*/

/*-----------------原地自转相关编码器常量*/

#define	ATTITUDE_ROTATION_ETR__90		33000	//自转90度时的编码器常量 154714//33000
#define	ATTITUDE_ROTATION_ETR_180		65000	//180

#define ATTITUDE_ROTATION_LOST_DR		400		//姿态调整原地自转控制器允许误差

/*-----------------里程计记录标准编码器常量*/
#define ODOMETER_STD_GRID_ETR_VAL		52494		//一个标准栅格的编码器数值 46200
#define ODOMETER_STD_GRID_ETR_FIX		0.8			//搜索 抵近障碍物时最少的编码器 用于里程计的修正
#define ODOMETER_STD_GRID_FAST_FIX	0.2			  //高速 抵近障碍物时最少的编码器 用于里程计的修正

#define ODOMETER_ENVIR_SIDE_START		0.45		//侧向检测开始 前进该数量标准栅格时保存两侧激活情况
#define ODOMETER_ENVIR_SIDE_END			0.50		//侧向检测结束值

/*--------------------电机停止判断*/
#define PIDCONTROLLER_MOTOR_STOP		10			//编码器分度值内数值小于这个值的时候电机则为停止

/*---------------------高速转弯*/
#define ATTITUDE_FROTATION_ETR_L1		18000 //54970//22000//28402
#define ATTITUDE_FROTATION_ETR_L2		0	//25195//0//12826

/*=================================================算法 速度PID控制器 姿态PID控制器 系列参数*/

/*--------------电机速度闭环控制PID控制器参数*/

#define MOTOR_SPEED_PID_KP					1.500		//速度PID控制器比例权重
#define MOTOR_SPEED_PID_KI					0.080		//速度PID控制器比例权重
#define MOTOR_SPEED_PID_KD					0.000		//速度PID控制器比例权重

#define MOTOR_OUTPUT_MAX						900			//电机输出最高值限制 MAX 1000

/*-------------直线运行姿态调整控制器系列参数*/

/*------------直线调整*/
#define ATTITUDE_STRAIGHT_KP 				0.2		//直线比例控制器动态调整参数 比例参数取值目的：限制修正值域到速度百分比
#define ATTITUDE_STRAIGHT_INE_MAX		1000		//直线误差修正最大偏移值 

/*------------自转调整*/
#define	ATTITUDE_ROTATION_KP_VALE		0.1	//自转速度比例控制器权重
#define ATTITUDE_ROTATION_OUT_MAX		650			//姿态调整原地自转控制器最高输出限制
#define ATTITUDE_ROTATION_FAST_VB		1.0			//快速调整时的补偿 1.0为不补偿 按照基础值比例增加

/*----------------高速运动控制器部分*/

/*-------------高速转弯*/
#define ATTITUDE_FROTATION_ETR_ET		3000		//允许误差范围
#define ATTITUDE_FROTATION_X_KP			0.05 		//控制器比例参数
#define ATTITUDE_FROTATION_CRP			2.81	//旋转半径比
#define ATTITUDE_FROTATION_TPOS			0.81		//转弯前的距离

/*-------------高速直行*/
#define ATTITUDE_STRAIGHT_FAST_KP		0.024		//高速直行时的位移闭环控制的速度比例
#define ATTITUDE_STRAIGHT_FAST_SET	650			//高速直行时的速度偏置 作用：消除由于末期比例结果过小而无法达到位移
#define ATTITUDE_STRAIGHT_SPD_LTD   1400		//高速直行的最高速度限制

/*=================================================策略 地图扫描策略 与 终止条件策略*/

/*-----------------------------------地图扫描策略*/
#define ERGODICMAP_SEARCHOR_PLAN		2				//地图搜寻者计划 0 正方向优先 1负方向优先

/*-----------------------------------扫描终止条件配置*/
#define ERGODICMAP_STOP_ALLCPLT			0			//确认地图无死点


/*-----------------------------------终点坐标的配置*/

#define ERGODICMAP_TARGET_POS_1			[0].x = 7,[0].y = 7		//允许最多4个目标点 坐标参考参照里程计坐标系
#define ERGODICMAP_TARGET_POS_2		  [1].x = 7,[1].y = 8
#define ERGODICMAP_TARGET_POS_3			[2].x = 8,[2].y = 7
#define ERGODICMAP_TARGET_POS_4		  [3].x = 8,[3].y = 8


#endif /*__PARAMETER_H*/

