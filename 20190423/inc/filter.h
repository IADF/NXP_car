#ifndef _filter_H
#define _filter_H

#include "headfile.h"
#include "common.h"



//�˲�����
//void Complementary_filter(float angle_m,float gyro_m);
void kalman_filter(float angle_m,float gyro_m);

//����д�Ŀ������˲�����
//�����ڲο�

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
//	//����
//	float dt;		//����ʱ�� dt = 1/frequent
//					//ʵ�����Ϊ����ʱ���в�ȷ���ԣ���Ҫ��ʱ����㺯��ʵʱУ��
//	
//	//���
//	float Angel;	//{0.0,0.0,0.0};	//���Ź��ƵĽǶ�	�����սǶȽ��
//	float Gyro_x;	//{0.0,0.0,0.0};	//���Ź��ƽ��ٶ�
//	
//	//�̶�����
//	float Q_Angle;	//{0.001,0.001,0.001};	//����������Э����	0.001�Ǿ���ֵ
//	float Q_Gyro;		//{0.003,0.003,0.003};	//������Ư������Э����	��mpu6050�ľ���ֵ
//	float R_Angle;	//{0.5,0.5,0.5};	//�Ǽ��ٶȼ�������Э����	
//	
//	char C_0;		//{1,1,1};	//H�����һ���۲���� �ǳ���
//	
//	//�м���
//	float Q_Bias;		//{0,0,0};		//������Ʈ��Ԥ��ֵ
//	float Angle_err;	//{0,0,0};		//�����м�ֵ Angle �۲�ֵ-Ԥ��ֵ
//	
//	float PCt_0,			//{0,0,0},	//�����м�ֵ
//		  PCt_1,			//{0,0,0},
//		  E,				//{0,0,0};	
//		  t_0,			//{0,0,0},	//t:�����м����
//		  t_1;			//{0,0,0};	
//	float K_0,			//{0,0,0},	//K:����������
//		  K_1;			//{0,0,0},
//
//			
//	float Pdot[4];		//{0,0,0,0};	//����P������м����
//	float PP[2][2];		//{{{1,0},{0,1}},{{1,0},{0,1}},{{1,0},{0,1}}};	//P����X��Angle����Э����
//	
//}KALMAN_STRUCT;
//
//void Kanman_Filter(KALMAN_STRUCT * kalman,float Gyro,float Accel,uint32 dt);
//void Kanman_Init(KALMAN_STRUCT * kalman);

#endif