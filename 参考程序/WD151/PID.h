#ifndef __PID_H__
#define __PID_H__
#include "common.h"
#include "headfile.h"
#include "filter.h"
#define KP 0
#define KI 1
#define KD 2
#define KT 3
#define KB 4
#define KF 5

typedef struct PID
{
	float SumError;	//����ۼ�	
	int32 LastError;	//Error[-1]
	int32 PrevError;	//Error[-2]	
	int32 LastData;	//Speed[-1]
	int32 Dis_Err;//΢����
	float Dis_Error_History[5];//��ʷ΢����
	Butter_BufferData Control_Device_LPF_Buffer;//��������ͨ�����������
} PID;

extern PID Speed_PID, Angle_PID, Ang_gyro_PID, Turn_gyro_PID, Turn_PID, Distance_PID;	//�����PID�����ṹ��
extern float Speed[4], Angle[4], Ang_gyro[5], Turn_gyro[5], Turn[5][4];

// PID������ʼ��
void PID_Parameter_Init(PID *sptr);


// λ��ʽ��̬PID����
int32 PlacePID_Control(PID *sprt, float *PID, float NowPiont, float SetPoint);

// λ��ʽPID����
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point);

// ����ʽPID����
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point);

#endif
