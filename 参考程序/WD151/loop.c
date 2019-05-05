#include "loop.h"
#include "MPU6050.h"
#include "ADRC.h"
#include "Balance.h"
#include "led.h"
#include "Control.h"
#include "filter.h"
#include "ANO_Data_Transfer.h"
#include "show.h"
#include "PID.h"
#include "My_I2C.h"
#include "EM.h"
#include "Search.h"
#include "ADRC.h"
/******************************************************************************************************************
2018��ȫ����ѧ�������ֱ����ܳ�����
�ɶ���Ϣ���̴�ѧ   ����WD��
����QQ��97354734
�ļ�����:�������
*******************************************************************************************************************/
volatile uint32_t sysTickUptime = 0;
float time_up[12],time_sum1 = 0;

static void Duty_2ms(void)
{
	L_AD_Sample();
	MPU6050_GetData(&GYRO, &ACC);	// ��ȡ����������
	Data_steepest();              //ԭʼ�����ݶ��½��˲�
}

static void Duty_4ms(void)
{
	
	Theory_Duty += -PID_Increase(&Ang_gyro_PID, Ang_gyro, (int32)(GYRO_Real.Y*10),(int32)Tar_Ang_Vel.Y);	// ֱ���ڻ����ٶȻ�
	Theory_Duty = range_protect(Theory_Duty, -950, 950);                                        // ֱ��PWM�޷�

	Direct_Parameter = PID_Realize(&Turn_gyro_PID, Turn_gyro, (int32)(GYRO_Real.Z*100),(int32)Radius*Speed_Min);	// ת�������Ҹ�(int32)Radius*Speed_Min
	Direct_Parameter = range_protect(Direct_Parameter, -1100, 1100);

	Direct_Last = Direct_Last*0.3 + Direct_Parameter*0.7;	// �����ϴν��ٶȻ����
	MOTOR_Duty_Left  = (Theory_Duty - Direct_Last);	// ���ҵ������ת��ϵ����������
	MOTOR_Duty_Right = (Theory_Duty + Direct_Last);	
	
	if (Run_Flag&&sysTickUptime>5000)//�����ȴ������˲���ʼ��5����������
		MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);	// �������ҵ��

	else
	{
		if (Stop_Flag)
		{
			if (Speed_Now > 60) MOTOR_Control(-850, -850);
			else                MOTOR_Control(0, 0);
		}
		else
			MOTOR_Control(0,0);
	}	
	
}

static void Duty_8ms(void)
{
	
	IMU_update(0.008f,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data); //��̬����
	Speed_Measure();                                                     //�ٶȲ���
	Distance += Speed_Now/55.0f;                     
	Speed_Calculate();                                                  //���ݵ�ǰ�ٶ��ж�ת��PID����
	Radius = -PlacePID_Control(&Turn_PID, Turn[Fres], Middle_Err, 0);  	//ת���⻷PID
	Road_Find();
	Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle,imu_data.pit*100, ADRC_SPEED_Controller.x1);	/* �ǶȻ��ӵ����ٶȻ��ϴ������� *///ADRC_SPEED_Controller.x1
	Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y,-1500, 1500);	// ע��������
	//ANO_DT_Data_Exchange();//�������ݵ�������λ��   ���Ϻ󽫶Կ������ڲ���Ӱ��  ����ʱ�ɼ��Ϸ���鿴����
}

static void Duty_40ms(void)    
{

	Target_Angle.Y = -PID_Realize(&Speed_PID, Speed, ADRC_SPEED_MIN_Controller.x1,Speed_Set);/* �ٶȻ��ӵ��ǶȻ��ϴ������� ADRC_SPEED_MIN_Controller.x1 Speed_Now*/
	Target_Angle.Y +=1400;        //����Ϊ14��
	if (ABS(Middle_Err>30)||sysTickUptime<5500)//�����������ǰ��������
		Target_Angle.Y = -500;
	else
		Target_Angle.Y = range_protect(Target_Angle.Y,Speed_Limit_Min, Speed_Limit_Max);	//  Speed_Limit_Min, Speed_Limit_Max

	Fhan_ADRC(&ADRC_SPEED_Controller,Target_Angle.Y); //���ٶȻ������������TD

}
static void Duty_100ms(void) 
{
	gpio_turn(I0);
	if (OLED_Refresh)
	OLED_Draw_UI();
	Check_BottonPress();
	BAT_CHECK();
}

//ϵͳ�������ã�������ִͬ�����ڵġ��̡߳�
static sched_task_t sched_tasks[] = 
{
	{Duty_100ms ,  100, 0},
	{Duty_40ms  ,   40, 0},
	{Duty_8ms   ,    8, 0},
	{Duty_4ms   ,    4, 0},
	{Duty_2ms   ,    2, 0}

};
//�������鳤�ȣ��ж��߳�����
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

//��������ŵ�main������while(1)�У���ͣ�ж��Ƿ����߳�Ӧ��ִ��
void Loop_Run(void)
{
	uint8_t index = 0;
	
	//ѭ���ж������̣߳��Ƿ�Ӧ��ִ��
	for(index=0;index < TASK_NUM;index++)
	{
		//��ȡϵͳ��ǰʱ�䣬��λMS --
		uint32_t tnow = sysTickUptime;
		//�����жϣ������ǰʱ���ȥ��һ��ִ�е�ʱ�䣬���ڵ��ڸ��̵߳�ִ�����ڣ���ִ���߳�
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			//�����̵߳�ִ��ʱ�䣬������һ���ж�
			sched_tasks[index].last_run = tnow;
			//ִ���̺߳�����ʹ�õ��Ǻ���ָ��
			sched_tasks[index].task_func();
		}
	}
}


	

