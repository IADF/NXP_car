#ifndef __BALANCE_H__
#define __BALANCE_H__
#include "common.h"
#include "headfile.h"

#define Zero_Angle 2.5f	// ��ɫ���
//#define Zero_Angle 22.0f	// ��ɫ���

//typedef struct
//{
//	float X;			 //����ϵ��
//	float Y;			 //����ϵ��
//	float Z;		 	 //΢��ϵ��

//}S_FLOAT_XYZ ; 

//typedef struct
//{
//	int16 X;			 //����ϵ��
//	int16 Y;			 //����ϵ��
//	int16 Z;		 	 //΢��ϵ��

//}S_INT16_XYZ ; 

//typedef struct
//{
//	int32 X;			 //����ϵ��
//	int32 Y;			 //����ϵ��
//	int32 Z;		 	 //΢��ϵ��

//}S_INT32_XYZ ; 

//extern S_FLOAT_XYZ 
//	GYRO_Real,		// ������ת���������
//	ACC_Real,		// ���ٶȼ�ת���������
//	Attitude_Angle,	// ��ǰ�Ƕ� 
//	Last_Angle,		// �ϴνǶ�
//	Target_Angle;	// Ŀ��Ƕ�
//	

//extern S_INT16_XYZ
//	GYRO,			// ������ԭʼ����
//	GYRO_Offset,	// ��������Ʈ
//	GYRO_Last,		// �������ϴ�����
//	ACC, 			// ���ٶȼ�����
//	ACC_Offset,		// ���ٶȼ���Ʈ
//	ACC_Last;		// ���ٶȼ��ϴ�����
//extern S_INT32_XYZ
//	Tar_Ang_Vel,	// Ŀ����ٶ�
//	Tar_Ang_Vel_Last;	// �ϴ�Ŀ����ٶ�

//extern int32 
//	Speed_Now,		// ��ǰʵ���ٶ�
//	Speed_Min,		// ������С�ٶ�
//	Speed_Set, 		// Ŀ���趨�ٶ�
//	Theory_Duty,
//	Direct_Last,
//	Speed_Diff,		// ��ǰʵ���ٶ�
//	Vel_Set,		// Ŀ��ת����ٶ�
//	Direct_Parameter,
//	Radius;

extern uint8 Point;
extern int32 Difference;
float  Turn_Out_Filter(float turn_out);
extern char Speed_Flag, Angle_Flag, Ang_Velocity_Flag, DMP_Flag,MPU_Flag,JS_Flag;
//void MPU6050_GetData(S_INT16_XYZ *GYRO, S_INT16_XYZ *ACC);
void Balance_Control(void);


#endif
