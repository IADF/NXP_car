#include "Search.h"
#include "mymath.h"
#include "EM.h"
#include "control.h"
#include "PID.h"
#include "MPU6050.h"
/******************************************************************************************************************
2018��ȫ����ѧ�������ֱ����ܳ�����
�ɶ���Ϣ���̴�ѧ   ����WD��
����QQ��97354734
�ļ�����:��·���ͼ��
*******************************************************************************************************************/
uint8 speed_low=170;
uint8 speed_mid=210;
uint8 speed_high=220;
uint16 speed_higher=230;
float Middle_Err;

uint8 RoadType;
u8 Dis_Start=0;
u8 Road_state = 0;

#define Normal     0
#define Ready_Str  1
#define Long_Str   2
#define Ramp   3

void Road_Find(void)//��·���ͼ��   ֱ�����µ�
{
	switch (Road_state)
	{
		case Normal:
			Speed_Set = speed_higher;
			Speed_Limit_Max =500 ;
			Speed_Limit_Min = -600;
			if (ABS(Middle_Err)<=10.0f)
			{
				Road_state = Ready_Str;
				Dis_Start = Distance;
			}
			else if (ABS(imu_data.w_acc.z)>8000)//z����������������ֵ����һ����ֵ
			{
				Road_state = Ramp;
			}
			else 
			{
				Road_state = Normal;
			}
			break;
		case Ready_Str:
			Speed_Set = speed_mid;
			Speed_Limit_Max =500 ;
			Speed_Limit_Min = -600;
			if (ABS(Middle_Err)<=10.0f)
			{
				
				if (Distance-Dis_Start > 200.0f)
				{
					Road_state = Long_Str;
					
				}
				else 
				Road_state = Ready_Str;
			}
			else 
				Road_state = Normal;
			break;
			
		case Long_Str:
			Speed_Set = speed_low;
			Speed_Limit_Max =600 ;
			Speed_Limit_Min = -500;
			if (ABS(Middle_Err)>10.0f)
			{
				Road_state = Normal;
				Dis_Start = 0;
			} 
			else Road_state = Long_Str;
			break;
		case Ramp:
			Speed_Set = speed_low;
			Speed_Limit_Max =900 ;
			Speed_Limit_Min = -200;
			Radius = 0;
			if (ABS(imu_data.w_acc.z)<8000)
			{
				Road_state = Normal;
			}
			break;
		default:
			break;
	}

}


