#ifndef _filter_H
#define _filter_H

#include "headfile.h"
#include "common.h"



//滤波程序
//void Complementary_filter(float angle_m,float gyro_m);
void kalman_filter(float angle_m,float gyro_m);

//别人写的卡尔曼滤波函数
//可用于参考

//typedef struct 
//{
//	float x[4];
//	float y[4];
//	float z[4];
//}xyz_f_4_t;
//
//typedef struct 
//{
//	float x[2][2];
//	float y[2][2];
//	float z[2][2];
//}xyz_f_2_2_t;
//
//typedef struct
//{
//	//输入
//	float dt;		//采样时间 dt = 1/frequent
//					//实际情况为调用时间有不确定性，需要做时间计算函数实时校正
//	
//	//输出
//	float Angel;	//{0.0,0.0,0.0};	//最优估计的角度	是最终角度结果
//	float Gyro_x;	//{0.0,0.0,0.0};	//最优估计角速度
//	
//	//固定参量
//	float Q_Angle;	//{0.001,0.001,0.001};	//陀螺仪噪声协方差	0.001是经验值
//	float Q_Gyro;		//{0.003,0.003,0.003};	//陀螺仪漂移噪声协方差	是mpu6050的经验值
//	float R_Angle;	//{0.5,0.5,0.5};	//是加速度计噪声的协方差	
//	
//	char C_0;		//{1,1,1};	//H矩阵的一个观测参数 是常数
//	
//	//中间量
//	float Q_Bias;		//{0,0,0};		//陀螺仪飘移预估值
//	float Angle_err;	//{0,0,0};		//计算中间值 Angle 观测值-预估值
//	
//	float PCt_0,			//{0,0,0},	//计算中间值
//		  PCt_1,			//{0,0,0},
//		  E,				//{0,0,0};	
//		  t_0,			//{0,0,0},	//t:计算中间变量
//		  t_1;			//{0,0,0};	
//	float K_0,			//{0,0,0},	//K:卡尔曼增益
//		  K_1;			//{0,0,0},
//
//			
//	float Pdot[4];		//{0,0,0,0};	//计算P矩阵的中间矩阵
//	float PP[2][2];		//{{{1,0},{0,1}},{{1,0},{0,1}},{{1,0},{0,1}}};	//P矩阵，X（Angle）的协方差
//	
//}KALMAN_STRUCT;
//
//void Kanman_Filter(KALMAN_STRUCT * kalman,float Gyro,float Accel,uint32 dt);
//void Kanman_Init(KALMAN_STRUCT * kalman);

#endif