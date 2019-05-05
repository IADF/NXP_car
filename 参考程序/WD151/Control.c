#include "control.h"
#include "mymath.h"
#include "Balance.h"
#include "ADRC.h"
#include "MPU6050.h"
#include "EM.h"
/******************************************************************************************************************
2018��ȫ����ѧ�������ֱ����ܳ�����
�ɶ���Ϣ���̴�ѧ   ����WD��
����QQ��97354734
�ļ�����:�ٶȼ��
*******************************************************************************************************************/
uint8 Run_OK = 0;
uint8 Run_Flag = 0;
uint8 Stop_Flag = 0;
uint8 Run_Stop = 1;
uint8 Auto_Run = 0;
uint8 Fast_run = 0;
S_FLOAT_XYZ 
	GYRO_Real,		// ������ת���������
	ACC_Real,		// ���ٶȼ�ת���������
	Attitude_Angle,	// ��ǰ�Ƕ�
	Last_Angle,		// �ϴνǶ�
	Target_Angle;	// Ŀ��Ƕ�
	

S_INT16_XYZ
	GYRO,			// ������ԭʼ����
	GYRO_Offset,	// ��������Ʈ
	GYRO_Last,		// �������ϴ�����
	ACC, 			// ���ٶȼ�����
	ACC_Offset,		// ���ٶȼ���Ʈ
	ACC_Last;		// ���ٶȼ��ϴ�����
	
S_INT32_XYZ
	Tar_Ang_Vel,	// Ŀ����ٶ�
	Tar_Ang_Vel_Last;	// �ϴ�Ŀ����ٶ�

int32 
	Speed_Now = 0,	// ��ǰʵ���ٶ�
	Speed_Min = 0,	// ������С�ٶ�
	Speed_Last = 0,	// ������С�ٶ�
	Speed_Set = 0, 	// Ŀ���趨�ٶ�
	Speed_Diff = 0, 	// Ŀ���趨�ٶ�
	Theory_Duty = 0,// ����ֱ��ռ�ձ�
	Vel_Set = 0,	// Ŀ��ת����ٶ�
	Direct_Parameter = 0,// ת��ϵ��
	Direct_Last = 0,
	Radius = 0;		// Ŀ��ת��뾶����

uint8 Point = 80;
int32 Difference = 0;
int16 Diff_speed = 0;
_sensor_st sensor;
	  			/* ���ֱ�־λ���Ŷ�ʱ���н���ʱ����� */
char Speed_Flag, Angle_Flag, Ang_Velocity_Flag;
extern _fix_inte_filter_st wcz_spe_fus;
char Left_Crazy = 0;	// �����ת
char Right_Crazy = 0;	// �����ת
int32 MOTOR_Duty_Left  = 0;
int32 MOTOR_Duty_Right = 0;
int MOTOR_Speed_Left = 0;
int MOTOR_Speed_Right = 0; 
int32 MOTOR_Speed_Left_Last = 0;
int32 MOTOR_Speed_Right_Last = 0;
int32 MOTOR_Left_Acc = 0;
int32 MOTOR_Right_Acc = 0;
volatile float  Distance = 0;
int32  Speed_Limit_Max =1200 ,Speed_Limit_Min = -800;

/******* ����ٶȲ��� ********/
uint16 temp1,temp2;
void Speed_Measure(void)
{
	static int32 Speed_Last = 0;
	static int32 Crazy_Count = 0;
	
	temp1 = ftm_count_get(ftm0);
	temp2 = ftm_count_get(ftm1);
	//����������
	ftm_count_clean(ftm0);
	ftm_count_clean(ftm1);
	//���ݷ����ź��ж����������跽���ź��Ǹߵ�ƽʱΪ��ת
	if(gpio_get(C5))    MOTOR_Speed_Right = -temp1;//�ٶ�ȡ�� �õ�����ת��
	else                MOTOR_Speed_Right = temp1;             
	if(gpio_get(H5))    MOTOR_Speed_Left 	= temp2;//�ٶ�ȡ�� �õ�����ת��
	else                MOTOR_Speed_Left 	= -temp2;
//	/******* �ҵ���ٶ���ؿ��� ********/

	MOTOR_Right_Acc = MOTOR_Speed_Right - MOTOR_Speed_Right_Last;	// ������ٶ�
	if (MOTOR_Right_Acc > 100)
	{
		Right_Crazy = 1;	// ��ת
	}
	if (MOTOR_Speed_Right > Speed_Set + 200)
	{
		Right_Crazy = 2;	// ��ת
	}
	if (MOTOR_Speed_Right < -350)
	{
		Right_Crazy = -1;	// ��ת
	}
	
	if (Right_Crazy)
	{
		if (MOTOR_Right_Acc <= 100)
		{
			if (MOTOR_Speed_Right < Speed_Set + 200 && MOTOR_Speed_Right > 0)
			{
				Right_Crazy = 0;
			}
		}
	}
	
	if (!Right_Crazy)
	{
		MOTOR_Speed_Right = MOTOR_Speed_Right*0.9 + MOTOR_Speed_Right_Last*0.1;
		MOTOR_Speed_Right_Last = MOTOR_Speed_Right;	// ���������ٶ�
	}
	else
	{
		MOTOR_Speed_Right = MOTOR_Speed_Right*0.5 + MOTOR_Speed_Right_Last*0.5;
		MOTOR_Speed_Right_Last = MOTOR_Speed_Right;	// ���������ٶ�
	}
	/******* �ҵ���ٶ���ؿ��ƽ��� ********/
	
	/******* �����ٶ���ؿ��� ********/
	
	MOTOR_Left_Acc = MOTOR_Speed_Left - MOTOR_Speed_Left_Last;	// ������ٶ�
	if (MOTOR_Left_Acc > 100)
	{
		Left_Crazy = 1;
	}
	if (MOTOR_Speed_Left > Speed_Set + 200)
	{
		Left_Crazy = 2;
	}
	if (MOTOR_Speed_Left < -350)
	{
		Left_Crazy = -1;
	}
	
	if (Left_Crazy)
	{
		if (MOTOR_Left_Acc <= 100)
		{
			if (MOTOR_Speed_Left < Speed_Set + 200 && MOTOR_Speed_Left > 0)
			{
				Left_Crazy = 0;
			}
		}
	}
	
	if (!Left_Crazy)
	{
		MOTOR_Speed_Left = 0.9*MOTOR_Speed_Left + 0.1*MOTOR_Speed_Left_Last;	// ��ͨ�˲�
		MOTOR_Speed_Left_Last = MOTOR_Speed_Left;	// ���������ٶ�
	}
	else
	{
		MOTOR_Speed_Left = 0.5*MOTOR_Speed_Left + 0.5*MOTOR_Speed_Left_Last;	// ��ͨ�˲�
		MOTOR_Speed_Left_Last = MOTOR_Speed_Left;	// ���������ٶ�
	}

	
	
	/******* �����ٶ���ؿ��ƽ��� ********/
	
	
	if ((Left_Crazy && Right_Crazy) || (Left_Crazy && MOTOR_Speed_Right < 10) || (Right_Crazy && MOTOR_Speed_Left < 10))
	{
		Crazy_Count++;
		if (Crazy_Count >= 200)
		{
			Crazy_Count = 0;
			Run_Flag = 0;
		}
	}
	else
	{
		Right_Crazy = 0;
	}
	
	/******* �����ת���⴦�� ********/
	if ((Left_Crazy > 0) && (Right_Crazy > 0))
	{
		Speed_Now = Speed_Last;			// ���߶���ת��ʹ���ϴ��ٶ���Ϊ��ǰʵ���ٶ�
	}
	else if (Left_Crazy)
	{
		if (MOTOR_Speed_Right > Speed_Set)
		{
			Speed_Now = Speed_Last;
		}
		else
		{
			Speed_Now = MOTOR_Speed_Right;	// ������ת��ʹ���ϴ��ٶ���Ϊ��ǰʵ���ٶ�
		}
	}
	else if (Right_Crazy)
	{
		if (MOTOR_Speed_Left > Speed_Set)
		{
			Speed_Now = Speed_Last;
		}
		else
		{
			Speed_Now = MOTOR_Speed_Left;	// �ҵ����ת��ʹ���ϴ��ٶ���Ϊ��ǰʵ���ٶ�
		}
	}
	else
	{
		Speed_Now = (MOTOR_Speed_Left + MOTOR_Speed_Right) / 2;	// ����ȡƽ�����㳵��ʵ���ٶ�
	}
	
	Speed_Now = Speed_Now *0.8 + Speed_Last * 0.2;
	Speed_Last = Speed_Now;
	Fhan_ADRC(&ADRC_SPEED_MIN_Controller,Speed_Now);  //�Ե�ǰ�ٶȽ���TD���� 
	Speed_Min = range_protect(ADRC_SPEED_MIN_Controller.x1,150,160);   //�޷�Խ��ת��Խ��  ��Խ����

}
_fix_inte_filter_st wcz_spe_fus;

s32 wcz_ref_speed;
float wcz_acc_deadzone;	
static s32 wcz_acc;
#define N_TIMES 5

void WCZ_Data_Calc(u8 dT_ms)//�����Ǽ��ٶȻ����ٶȺͱ������ٶ��ں�
{

	wcz_ref_speed  = Speed_Now;
	wcz_acc        = -(s32)imu_data.w_acc.x/10;
	wcz_acc_deadzone = LIMIT(5 *(0.996f - imu_data.x_vec.x *imu_data.x_vec.x),0,1) *10;
	
	
	wcz_spe_fus.fix_kp = 0.2f;
	wcz_spe_fus.in_est_d = my_deadzone(wcz_acc,0,wcz_acc_deadzone);
	wcz_spe_fus.in_obs = wcz_ref_speed;
	wcz_spe_fus.e_limit = 100;
	fix_inte_filter(dT_ms*1e-3f,&wcz_spe_fus);

}
float battery=0;
void BAT_CHECK(void)
{
	battery = adc_once(ADC0_SE4,ADC_12bit)*3.3*12*10/2/4096;
	
}
void Speed_Control(void)
{
		switch(Mode_Set)
		{
		case 0:		Speed_Set = 0;// Ĭ�ϳ�ʼ�ٶ�
				 	break;
		case 1:		Speed_Set = 100;
				 	break;
		case 2:		Speed_Set = 120;	
				 	break;
		case 3:		Speed_Set = 150;
				 	break;
		case 4:		Speed_Set = 180;
				 	break;
		case 5:		Speed_Set = 210;
				 	break;
		case 6:		Speed_Set = 250;
					break;
		default:	Speed_Set = 0;
					break;
		}
}

void MOTOR_Control(int32 LDuty, int32 RDuty)
{
	if (LDuty > 0)
	{
		LDuty = range_protect(LDuty, -MOTOR_MAX, MOTOR_MAX);	// �޷�����
		ftm_pwm_duty(ftm2,ftm_ch2,LDuty);	  	// ռ�ձ����990������
		ftm_pwm_duty(ftm2,ftm_ch3,0);	// ռ�ձ����990������
	}
	else
	{
		LDuty = range_protect(LDuty, -MOTOR_MAX, MOTOR_MAX);// �޷�����
		ftm_pwm_duty(ftm2,ftm_ch2,0);	  	// ռ�ձ����990������
		ftm_pwm_duty(ftm2,ftm_ch3,-LDuty);	// ռ�ձ����990������
	}
	
	if (RDuty > 0)
	{
		RDuty = range_protect(RDuty, -MOTOR_MAX, MOTOR_MAX);	// �޷�����
		ftm_pwm_duty(ftm2,ftm_ch0,0);	  	// ռ�ձ����990������
		ftm_pwm_duty(ftm2,ftm_ch1,RDuty);	// ռ�ձ����990������
	}
	else
	{
		RDuty = range_protect(RDuty, -MOTOR_MAX, MOTOR_MAX);// �޷�����
		ftm_pwm_duty(ftm2,ftm_ch0,-RDuty);	  	// ռ�ձ����990������
		ftm_pwm_duty(ftm2,ftm_ch1,0);	// ռ�ձ����990������
	}
}

void Start_Control(void)
{
	if (Run_OK)
	{									
		if (Run_Stop)	// ����Ļ�����Ƿ�ͣ����Ĭ��ͣ��
		{
			if (gpio_get(I5)==0)
			{
						
				Run_state = 4;
				Stop_Flag=1;
				Run_OK = 0;			//��������ɹ���־λ
			}
		}			
	}
}
uint8 Fres = 0;	// ǰհ
void Speed_Calculate(void)//�ɵ�ǰ�ٶȾ�������PD��ȡֵ
{
	if (ADRC_SPEED_MIN_Controller.x1 < 110)
	{
		Fres = 0;
	}
	else if (ADRC_SPEED_MIN_Controller.x1 < 130)
	{
		Fres = 1;
	}
	else if (ADRC_SPEED_MIN_Controller.x1 < 150)
	{
		Fres = 2;
	}
	else if (ADRC_SPEED_MIN_Controller.x1 < 170)
	{
		Fres = 3;
	}
	else if (ADRC_SPEED_MIN_Controller.x1 >= 200)
	{
		Fres = 4;
	}
}
/******** �޷����� *********/
int32 range_protect(int32 duty, int32 min, int32 max)//�޷�����
{
	if (duty >= max)
	{
		return max;
	}
	if (duty <= min)
	{
		return min;
	}
	else
	{
		return duty;
	}
}
/* ��ʼ���õ���һЩ���� */
void Balance_Init(void)
{
	Attitude_Angle.Y = 0;
	Target_Angle.Y = 0;
	Tar_Ang_Vel.Y = 0;
	Tar_Ang_Vel.Z = 0;
	Speed_Min=100;
	Distance = 0;
	Ring_state = 0;
	Ring_Flag = 0;
	Run_OK = 0;
	Run_state = 0;
}

