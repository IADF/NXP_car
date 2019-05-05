#ifndef __MY_PID_H
#define __MY_PID_H

#include "System.h"
/*=====================================================================================================================
						   PID�������趨����
=====================================================================================================================*/
#define INNER_INTEGRAL 1.0 		//�ڻ�����ϵ��
#define INTEGRAL_LIMIT_EN 1		//���������޷�
/*=====================================================================================================================
						
=====================================================================================================================*/
typedef struct
{
	float kp;			 //����ϵ��
	float ki;			 //����ϵ��
	float kd;		 	 //΢��ϵ��
	float k_pre_d; //previous_d ΢������
	float k_ff;		 //ǰ�� 
	

}__attribute__((packed)) PID_arg_t;

typedef struct
{
	float err;
	float err_old;
	float feedback_old;
	float feedback_d;
	float err_d;
	float err_i;
	float ff;
	float pre_d;

}__attribute__((packed)) PID_val_t;


void PID_calculate( float T,            //����
					float in_ff,				//ǰ��
					float expect,				//����ֵ���趨ֵ��
					float feedback,			//����ֵ
					PID_arg_t *pid_arg, //PID�����ṹ��
					PID_val_t *pid_val,	//PID���ݽṹ��
					float inte_lim,			//integration limit�������޷�,0���޷�
					int *out  );			//���

#endif

