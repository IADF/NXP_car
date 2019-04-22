#include "headfile.h"
#include "filter.h"

//�������˲������뺯��  
float angle, angle_dot;//�ǶȺͽ��ٶ�  
float angle_0, angle_dot_0;//�ɼ����ĽǶȺͽ��ٶ� 

//float dt=20*0.001;
//ע�⣺dt��ȡֵΪkalman�˲�������ʱ��  
//һ��Ϊ�����м����  
float P[2][2] = {{ 1, 0 },
                 { 0, 1 }}; 
float Pdot[4] ={ 0,0,0,0}; 
//���Գ����Ĳ���//noir
//const float Q_angle=0.16, Q_gyro=0.88; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�  
//const float R_angle=0.005 ,C_0 = 1;//R_angle�ǽǶȵ�Э����
//����ֱ�����Ĳ���
//const float Q_angle=1, Q_gyro=0.3; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�  
//const float R_angle=0.01 ,C_0 = 1;//R_angle�ǽǶȵ�Э����
float q_bias=0, angle_err=0, PCt_0=0, PCt_1=0, E=0, K_0=0, K_1=0, t_0=0, t_1=0;

const float Q_angle=0.16, Q_gyro=1; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�  
const float R_angle=0.005 ,C_0 = 1;//R_angle�ǽǶȵ�Э����

//float dt=0.002;//ע�⣺dt��ȡֵΪkalman�˲�������ʱ���뻥���˲���ͬʹ��
//ʱ����ͨ���˲�ʱ�����ó�
float dt=0.006;

//�����˲�����
float angle1,angle1_dot;//�ⲿ���õı���
float bias_cf;


extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;

//kalman�˲�����
//angle_dot���ⲿ��Ҫ���õı�����//�رտ��������ٶ��˲�
//angle ���ⲿ��Ҫ���õı�����
void kalman_filter(float angle_m,float gyro_m)
{
  angle+=(gyro_m-q_bias) * dt;//�������

  Pdot[0]=Q_angle - P[0][1] - P[1][0];// Pk-' ����������Э�����΢�� 

  Pdot[1]=- P[1][1];
  Pdot[2]=- P[1][1];
  Pdot[3]=Q_gyro; 
  P[0][0] += Pdot[0] * dt;// Pk- ����������Э����΢�ֵĻ��� = ����������Э����
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt; 
  P[1][1] += Pdot[3] * dt;
  angle_err = angle_m - angle;//zk�������  
  
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;  
  K_0 = PCt_0 / E;//���㿨��������
  K_1 = PCt_1 / E;
  t_0 = PCt_0; 
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;//����������Э���� 
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; //���ŽǶ� //������� 
  q_bias += K_1 * angle_err;//������� 

  angle_dot = gyro_m-q_bias;//���Ž��ٶ�  //���ֵ��������ƣ���΢�� = ���ٶ�
}




////һ�׻����˲�
//void Complementary_filter(float angle_m,float gyro_m)
//{
//    bias_cf*=0.998;
//    bias_cf+=gyro_m*0.002;
//    angle1_dot=gyro_m-bias_cf;
//    angle1=(angle1+angle1_dot*dt)*0.65+angle_m*0.35;
//
//}

//����д�Ŀ������˲�
//void Kanman_Init(KALMAN_STRUCT * kalman)
//{
//	int i;
//	
//	//���
//	(*kalman).Angel = 0.0;	//���Ź��ƵĽǶ�	�����սǶȽ��
//	(*kalman).Gyro_x = 0.0;	//���Ź��ƽ��ٶ�
//	
//	//�̶�����//�ο��ɶ���Ϣ���̴�ѧ�Ŀ���������
//	(*kalman).Q_Angle = 0.001;		//{0.001,0.001,0.001};	//����������Э����	0.001�Ǿ���ֵ
//	(*kalman).Q_Gyro = 0.001;		//{0.003,0.003,0.003};	//������Ư������Э����	��mpu6050�ľ���ֵ
//	(*kalman).R_Angle = 10;		//{0.5,0.5,0.5};	//�Ǽ��ٶȼ�������Э����	
//	
//	(*kalman).C_0 = 1;		//{1,1,1};	//H�����һ���۲���� �ǳ���
//	
//	//�м���
//	(*kalman).Q_Bias = 0;		//{0,0,0};		//������Ʈ��Ԥ��ֵ
//	(*kalman).Angle_err = 0;	//{0,0,0};		//�����м�ֵ Angle �۲�ֵ-Ԥ��ֵ
//	
//	(*kalman).PCt_0 = 0;			//{0,0,0},	//�����м�ֵ
//	(*kalman).PCt_1 = 0;			//{0,0,0},
//	(*kalman).E     = 0;			//{0,0,0};
//	(*kalman).t_0   = 0;			//{0,0,0},	//t:�����м����
//	(*kalman).t_1   = 0;			//{0,0,0},
//	
//	(*kalman).K_0 = 0;			//{0,0,0},	//K:����������
//	(*kalman).K_1 = 0;			//{0,0,0},
//	
//	for(i = 0;i < 4;i++)	//{0,0,0,0}	//����P������м����
//	{
//		(*kalman).Pdot[i] = 0;
//	}
//	
//	(*kalman).PP[0][0] = 1;
//	(*kalman).PP[0][1] = 0;
//	(*kalman).PP[1][0] = 0;
//	(*kalman).PP[1][1] = 1;
//}
//
//void Kanman_Filter(KALMAN_STRUCT * kalman,float Gyro,float Accel,uint32 dt)	//Gyro�����ǵĲ���ֵ  |  Accel���ٶȼƵĽǶȼ�  |  dt��ʱ�俼����С�� �� ��С�ķֶȱ�ʾ
//{
//	float dt_f;
//	
//	//��dt�����λ��ms��u32�ͱ������ֵת��Ϊfloat�͵�����Ϊ��λ��ֵ
//	dt_f = (float)dt;
//	dt_f = dt_f / 1000;
//	
//	//x��ָ��ǰ��y��ָ���������ϵ   Ҫ�㸩����
//	//��ô�����Ӧ����y��Ľ��ٶȣ�Gyro����y�����Ǽ��ٶȼƹ���ֵ
//	//����ϵ������������
//	
//	
//	//�ǶȲ���ģ�ͷ��� �Ƕȹ���ֵ=��һ�����нǶ�+�����ٶ�-��һ�ε�������Ʈ��*dt_f
//	//��Ư����˵����Ϊÿ�ζ�����ͬ��Q_bias=Q_bias
//	//���ƽǶ�
//	(*kalman).Angel += (Gyro - (*kalman).Q_Bias) * dt_f;
//	
//	//�������ģ�͵ķ���
//	(*kalman).Pdot[0] = (*kalman).Q_Angle - (*kalman).PP[0][1] - (*kalman).PP[1][0];
//	(*kalman).Pdot[1] = -(*kalman).PP[1][1];
//	(*kalman).Pdot[2] = -(*kalman).PP[1][1];
//	(*kalman).Pdot[3] = (*kalman).Q_Gyro;
//	
//	(*kalman).PP[0][0] += (*kalman).Pdot[0] * dt_f;
//	(*kalman).PP[0][1] += (*kalman).Pdot[1] * dt_f;
//	(*kalman).PP[1][0] += (*kalman).Pdot[2] * dt_f;
//	(*kalman).PP[1][1] += (*kalman).Pdot[3] * dt_f;
//	
//	//���㿨��������
//	(*kalman).PCt_0 = (*kalman).C_0 * (*kalman).PP[0][0];	//����˷����м����
//	(*kalman).PCt_1 = (*kalman).C_0 * (*kalman).PP[0][1];	//C_0=1
//	(*kalman).E = (*kalman).R_Angle + (*kalman).C_0 * (*kalman).PCt_0;	//��ĸ
//	(*kalman).K_0 = (*kalman).PCt_0 / (*kalman).E;	//���������棬������һ����Angle�ģ�һ����Q_bias��
//	(*kalman).K_1 = (*kalman).PCt_1 / (*kalman).E;
//	
//	//�������ŽǶȡ�������Ʈ
//	(*kalman).Angle_err = Accel - (*kalman).Angel;
//	(*kalman).Angel += (*kalman).K_0 * (*kalman).Angle_err;	//�������ŵĽǶ�
//	(*kalman).Q_Bias += (*kalman).K_1 * (*kalman).Angle_err;	//�������ŵ���Ʈ
//	
//	(*kalman).Gyro_x = Gyro -(*kalman).Q_Bias;	//��������Ž��ٶ�
//	
//	//���¹���ģ�͵ķ���
//	(*kalman).t_0 = (*kalman).PCt_0;	//��������м�������൱��a
//	(*kalman).t_1 = (*kalman).C_0 * (*kalman).PP[0][1];	//��������м�������൱��b
//	
//	(*kalman).PP[0][0] -= (*kalman).K_0 * (*kalman).t_0;
//	(*kalman).PP[0][1] -= (*kalman).K_0 * (*kalman).t_1;
//	(*kalman).PP[1][0] -= (*kalman).K_1 * (*kalman).t_0;
//	(*kalman).PP[1][1] -= (*kalman).K_1 * (*kalman).t_1;
//}
//
