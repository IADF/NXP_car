
#include "MY_PID.h"

void PID_calculate( float T,            //���ڣ���λ���룩
					float in_ff,				//ǰ��ֵ
					float expect,				//����ֵ���趨ֵ��
					float feedback,			//����ֵ����
					PID_arg_t *pid_arg, //PID�����ṹ��
					PID_val_t *pid_val,	//PID���ݽṹ��
					float inte_lim,			//integration limit�������޷�
					int *out  )				//���
{
	pid_val->err = pid_arg->kp *(expect - feedback);
	pid_val->err_d = pid_arg->kd *(pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);
	pid_val->feedback_d = - ( pid_arg->k_pre_d ) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
	pid_val->err_i += (pid_arg->ki *pid_val->err + INNER_INTEGRAL *pid_val->feedback_d)*T;
	
	if(INTEGRAL_LIMIT_EN)
	{
		pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	}
	
	*out = pid_arg->k_ff *in_ff + pid_val->err + pid_val->err_d + pid_val->feedback_d + pid_val->err_i;
	
	pid_val->feedback_old = feedback;
	pid_val->err_old = pid_val->err;
}




