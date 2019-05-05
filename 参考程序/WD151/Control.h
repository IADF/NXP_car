#ifndef _Control_h
#define _Control_h

#include "headfile.h"
#include "common.h"
#include "filter.h"
//�������˿�
#define MOTOR_FTM   FTM2
#define MOTOR1_PWM  FTM2_CH0_PIN	// PTC1
#define MOTOR2_PWM  FTM2_CH1_PIN	// PTC2
#define MOTOR3_PWM  FTM2_CH2_PIN	// PTC3
#define MOTOR4_PWM  FTM2_CH3_PIN // PTC4
#define MOTOR_HZ    (20*1000)	//����ģʽ�£�Ƶ��Ӧ���� 30~100��
								//����ģʽ�£�Ƶ��Ӧ���� 20k ����
#define MOTOR_MAX   950
extern float battery;
extern int32 MOTOR_Duty_Left, MOTOR_Duty_Right;
extern volatile float  Distance;
extern int MOTOR_Speed_Left, MOTOR_Speed_Right;
extern int32 MOTOR_Left_Acc, MOTOR_Right_Acc;
extern uint8 Run_Flag;
extern uint8 Stop_Flag;
extern uint8 Run_Stop;
extern uint8 Auto_Run;
extern char Left_Crazy, Right_Crazy;
extern char Mode_Set;
extern int32  Speed_Limit_Max,Speed_Limit_Min;
void Get_Angle(void);
extern float angle,Gyro_Balance;

extern uint8 Fres;
void Speed_Calculate(void);
extern uint8 Run_OK;
typedef struct
{
	float X;			 //����ϵ��
	float Y;			 //����ϵ��
	float Z;		 	 //΢��ϵ��

}S_FLOAT_XYZ ; 

typedef struct
{
	int16 X;			 //����ϵ��
	int16 Y;			 //����ϵ��
	int16 Z;		 	 //΢��ϵ��

}S_INT16_XYZ ; 

typedef struct
{
	int32 X;			 //����ϵ��
	int32 Y;			 //����ϵ��
	int32 Z;		 	 //΢��ϵ��

}S_INT32_XYZ ; 

extern S_FLOAT_XYZ 
	GYRO_Real,		// ������ת���������
	ACC_Real,		// ���ٶȼ�ת���������
	Attitude_Angle,	// ��ǰ�Ƕ� 
	Last_Angle,		// �ϴνǶ�
	Target_Angle;	// Ŀ��Ƕ�
	

extern S_INT16_XYZ
	GYRO,			// ������ԭʼ����
	GYRO_Offset,	// ��������Ʈ
	GYRO_Last,		// �������ϴ�����
	ACC, 			// ���ٶȼ�����
	ACC_Offset,		// ���ٶȼ���Ʈ
	ACC_Last;		// ���ٶȼ��ϴ�����
extern S_INT32_XYZ
	Tar_Ang_Vel,	// Ŀ����ٶ�
	Tar_Ang_Vel_Last;	// �ϴ�Ŀ����ٶ�

extern int32 
	Speed_Now,		// ��ǰʵ���ٶ�
	Speed_Min,		// ������С�ٶ�
	Speed_Set, 		// Ŀ���趨�ٶ�
	Theory_Duty,
	Direct_Last,
	Speed_Diff,		// ��ǰʵ���ٶ�
	Vel_Set,		// Ŀ��ת����ٶ�
	Direct_Parameter,
	Radius;

/*********** �������� ************/
void Speed_Measure(void);	//����ٶȲ���
void Start_Control(void);	//�����߼����ͣ������
void Speed_Control(void);
void MOTOR_Control(int32 LDuty, int32 RDuty);	// �������
int32 range_protect(int32 duty, int32 min, int32 max); //�޷�����
void Diff_speedcontrol(void);
void WCZ_Data_Calc(u8 dT_ms);
void Speed_cal(void);
float Slope_Calculate(uint8 begin,uint8 end,float *p);
void BAT_CHECK(void);
void Balance_Init(void);
#endif
