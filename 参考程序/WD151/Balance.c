#include "Balance.h"
#include "MPU6050.h"
#include "ANO_Data_Transfer.h"
#include "System.h"
#include "MyBalance.h"
#include "ADRC.h"
#include "PID.h"
#include "Control.h"
#include "Search.h"
#include "EM.h"
#include "show.h"

/********************************************************/
/********************* ����ƽ����� *********************/
// Ƶ�ʿ����ڶ�ʱ��������
void Balance_Control(void)
{
//	MPU6050_GetData(&GYRO, &ACC);	// ��ȡ����������
//	Data_steepest();
	Speed_Calculate();
	Radius = -PlacePID_Control(&Turn_PID, Turn[Fres], Middle_Err, 0);	
//	IMU_update(0.005f,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data); //��̬����
		/* ���ٶȻ���Ϊ���ڻ�����ֱ�� */
	//ADRC_Control(&ADRC_GYRO_Controller,Tar_Ang_Vel.Y,GYRO_Real.Y*10);
	//Theory_Duty = ADRC_GYRO_Controller.u;
	Theory_Duty += -PID_Increase(&Ang_gyro_PID, Ang_gyro, (int32)(GYRO_Real.Y*10), 0);	// ����ֱ��PWM
	Theory_Duty = range_protect(Theory_Duty, -950, 950);
						/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min
	
//	ADRC_Control(&ADRC_I_TURN_Controller,0,GYRO_Real.Z*10);
//	Direct_Parameter = -ADRC_I_TURN_Controller.u;
	Direct_Parameter = -PID_Realize(&Turn_gyro_PID, Turn_gyro, (int32)(GYRO_Real.Z*100), 0);	// ת�������Ҹ�
	Direct_Parameter = range_protect(Direct_Parameter, -1200, 1200);
	//Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;	// �����ϴν��ٶȻ����
	MOTOR_Duty_Left  = Theory_Duty - Direct_Last;	// ���ҵ������ת��ϵ����������
	MOTOR_Duty_Right = Theory_Duty + Direct_Last;	
	
	if (Run_Flag)
	{
		MOTOR_Control(MOTOR_Duty_Right, MOTOR_Duty_Left);	// �������ҵ��
	}
	else
	{
		if (Stop_Flag)
		{
			if (Speed_Now > 80)
			{
				MOTOR_Control(550, 550);
			}
			else
			{
				MOTOR_Control(0, 0);
			}
		}
		else
		{
			MOTOR_Control(0, 0);
		}
	}	
	if (Angle_Flag)		// ֱ���ǶȻ�	10ms
	{
		Angle_Flag = 0;
		Speed_Measure();// ��ȡ��ǰ�ٶ�
				
		Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, imu_data.pit*100, Target_Angle.Y);	/* �ǶȻ��ӵ����ٶȻ��ϴ������� */
		Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// ע��������
	}
	if (Speed_Flag)		// �ٶȻ�	50ms
	{
		Speed_Flag = 0;
		Target_Angle.Y = PID_Realize(&Speed_PID, Speed, Speed_Now,0);/* �ٶȻ��ӵ��ǶȻ��ϴ������� */
		Target_Angle.Y -= 2000;
		Target_Angle.Y = range_protect(Target_Angle.Y, -2700, 1500);	// -44 22

	}
}

/* ��ʼ���õ���һЩ���� */


