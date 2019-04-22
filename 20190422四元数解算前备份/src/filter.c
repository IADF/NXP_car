#include "headfile.h"
#include "filter.h"

//卡尔曼滤波参数与函数  
float angle, angle_dot;//角度和角速度  
float angle_0, angle_dot_0;//采集来的角度和角速度 

//float dt=20*0.001;
//注意：dt的取值为kalman滤波器采样时间  
//一下为运算中间变量  
float P[2][2] = {{ 1, 0 },
                 { 0, 1 }}; 
float Pdot[4] ={ 0,0,0,0}; 
//调试出来的参数//noir
//const float Q_angle=0.16, Q_gyro=0.88; //角度数据置信度,角速度数据置信度  
//const float R_angle=0.005 ,C_0 = 1;//R_angle是角度的协方差
//龙邱直立车的参数
//const float Q_angle=1, Q_gyro=0.3; //角度数据置信度,角速度数据置信度  
//const float R_angle=0.01 ,C_0 = 1;//R_angle是角度的协方差
float q_bias=0, angle_err=0, PCt_0=0, PCt_1=0, E=0, K_0=0, K_1=0, t_0=0, t_1=0;

const float Q_angle=0.16, Q_gyro=1; //角度数据置信度,角速度数据置信度  
const float R_angle=0.005 ,C_0 = 1;//R_angle是角度的协方差

//float dt=0.002;//注意：dt的取值为kalman滤波器采样时间与互补滤波共同使用
//时间是通过滤波时间计算得出
float dt=0.006;

//互补滤波参数
float angle1,angle1_dot;//外部引用的变量
float bias_cf;


extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;

//kalman滤波程序
//angle_dot（外部需要引用的变量）//关闭卡尔曼角速度滤波
//angle （外部需要引用的变量）
void kalman_filter(float angle_m,float gyro_m)
{
  angle+=(gyro_m-q_bias) * dt;//先验估计

  Pdot[0]=Q_angle - P[0][1] - P[1][0];// Pk-' 先验估计误差协方差的微分 

  Pdot[1]=- P[1][1];
  Pdot[2]=- P[1][1];
  Pdot[3]=Q_gyro; 
  P[0][0] += Pdot[0] * dt;// Pk- 先验估计误差协方差微分的积分 = 先验估计误差协方差
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt; 
  P[1][1] += Pdot[3] * dt;
  angle_err = angle_m - angle;//zk先验估计  
  
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;  
  K_0 = PCt_0 / E;//计算卡尔曼增益
  K_1 = PCt_1 / E;
  t_0 = PCt_0; 
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;//后验估计误差协方差 
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; //最优角度 //后验估计 
  q_bias += K_1 * angle_err;//后验估计 

  angle_dot = gyro_m-q_bias;//最优角速度  //输出值（后验估计）的微分 = 角速度
}




////一阶互补滤波
//void Complementary_filter(float angle_m,float gyro_m)
//{
//    bias_cf*=0.998;
//    bias_cf+=gyro_m*0.002;
//    angle1_dot=gyro_m-bias_cf;
//    angle1=(angle1+angle1_dot*dt)*0.65+angle_m*0.35;
//
//}

//别人写的卡尔曼滤波
//void Kanman_Init(KALMAN_STRUCT * kalman)
//{
//	int i;
//	
//	//输出
//	(*kalman).Angel = 0.0;	//最优估计的角度	是最终角度结果
//	(*kalman).Gyro_x = 0.0;	//最优估计角速度
//	
//	//固定参量//参考成都信息工程大学的卡尔曼参数
//	(*kalman).Q_Angle = 0.001;		//{0.001,0.001,0.001};	//陀螺仪噪声协方差	0.001是经验值
//	(*kalman).Q_Gyro = 0.001;		//{0.003,0.003,0.003};	//陀螺仪漂移噪声协方差	是mpu6050的经验值
//	(*kalman).R_Angle = 10;		//{0.5,0.5,0.5};	//是加速度计噪声的协方差	
//	
//	(*kalman).C_0 = 1;		//{1,1,1};	//H矩阵的一个观测参数 是常数
//	
//	//中间量
//	(*kalman).Q_Bias = 0;		//{0,0,0};		//陀螺仪飘移预估值
//	(*kalman).Angle_err = 0;	//{0,0,0};		//计算中间值 Angle 观测值-预估值
//	
//	(*kalman).PCt_0 = 0;			//{0,0,0},	//计算中间值
//	(*kalman).PCt_1 = 0;			//{0,0,0},
//	(*kalman).E     = 0;			//{0,0,0};
//	(*kalman).t_0   = 0;			//{0,0,0},	//t:计算中间变量
//	(*kalman).t_1   = 0;			//{0,0,0},
//	
//	(*kalman).K_0 = 0;			//{0,0,0},	//K:卡尔曼增益
//	(*kalman).K_1 = 0;			//{0,0,0},
//	
//	for(i = 0;i < 4;i++)	//{0,0,0,0}	//计算P矩阵的中间矩阵
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
//void Kanman_Filter(KALMAN_STRUCT * kalman,float Gyro,float Accel,uint32 dt)	//Gyro陀螺仪的测量值  |  Accel加速度计的角度计  |  dt的时间考虑用小数 或 更小的分度表示
//{
//	float dt_f;
//	
//	//把dt这个单位是ms的u32型变量里的值转换为float型的以秒为单位的值
//	dt_f = (float)dt;
//	dt_f = dt_f / 1000;
//	
//	//x轴指向前，y轴指向左的坐标系   要算俯仰角
//	//那么输入的应该是y轴的角速度（Gyro）和y轴的倾角加速度计估计值
//	//坐标系情况大概是这样
//	
//	
//	//角度测量模型方程 角度估计值=上一次最有角度+（角速度-上一次的最优零飘）*dt_f
//	//就漂移来说，认为每次都是相同的Q_bias=Q_bias
//	//估计角度
//	(*kalman).Angel += (Gyro - (*kalman).Q_Bias) * dt_f;
//	
//	//计算估计模型的方差
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
//	//计算卡尔曼增益
//	(*kalman).PCt_0 = (*kalman).C_0 * (*kalman).PP[0][0];	//矩阵乘法的中间变量
//	(*kalman).PCt_1 = (*kalman).C_0 * (*kalman).PP[0][1];	//C_0=1
//	(*kalman).E = (*kalman).R_Angle + (*kalman).C_0 * (*kalman).PCt_0;	//分母
//	(*kalman).K_0 = (*kalman).PCt_0 / (*kalman).E;	//卡尔曼增益，两个，一个是Angle的，一个是Q_bias的
//	(*kalman).K_1 = (*kalman).PCt_1 / (*kalman).E;
//	
//	//计算最优角度、最优零飘
//	(*kalman).Angle_err = Accel - (*kalman).Angel;
//	(*kalman).Angel += (*kalman).K_0 * (*kalman).Angle_err;	//计算最优的角度
//	(*kalman).Q_Bias += (*kalman).K_1 * (*kalman).Angle_err;	//计算最优的零飘
//	
//	(*kalman).Gyro_x = Gyro -(*kalman).Q_Bias;	//计算得最优角速度
//	
//	//更新估计模型的方差
//	(*kalman).t_0 = (*kalman).PCt_0;	//矩阵计算中间变量，相当于a
//	(*kalman).t_1 = (*kalman).C_0 * (*kalman).PP[0][1];	//矩阵计算中间变量，相当于b
//	
//	(*kalman).PP[0][0] -= (*kalman).K_0 * (*kalman).t_0;
//	(*kalman).PP[0][1] -= (*kalman).K_0 * (*kalman).t_1;
//	(*kalman).PP[1][0] -= (*kalman).K_1 * (*kalman).t_0;
//	(*kalman).PP[1][1] -= (*kalman).K_1 * (*kalman).t_1;
//}
//
