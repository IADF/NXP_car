#include "control.h"
#include "headfile.h"


#define parameter_mode    10//0Ϊ����1Ϊ����//����3��4��ʼΪ��������6ms��//����5��Ԫ���������//8�ٶȽ����ȶ�//9�ٶ��е��ȶ��ԱȽ��ȣ�ż����ʧ��//10�ٶȽϿ죬���ȶ����ڵ�����

//ȥ����Ʈ���е����ı�

//���߳�����ر���
extern uint8 L_curve_flag;//�����־λ
extern uint8 R_curve_flag;//�����־λ

//��Ԫ��������ر���
extern _sensor_st sensor;
extern _imu_st imu_data;

//mpu6050��ر���
uint8 PIT_flag;
float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x=0,mpu_gyro_float_y=0,mpu_gyro_float_z=0,mpu_acc_float_x=0,mpu_acc_float_y=0,mpu_acc_float_z=0;//mpu6050�����ĽǶȣ�δ�˲���
float last_mpu_gyro_float_x,last_mpu_gyro_float_y,last_mpu_gyro_float_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern float angle,angle_dot,angle1,angle1_dot;
extern int mpu_gyro_x_offset,mpu_gyro_y_offset,mpu_gyro_z_offset;//��Ʈ
extern int mpu_acc_x_offset,mpu_acc_y_offset,mpu_acc_z_offset;

//�����������
uint8 str[20];

//���ٶȻ�
float mpu_gyro_integral_val;

//���߲ɼ�������ر���
extern float middle_Line_temp;
extern int middle_Line;
float theory_middle_line=39.5;//��������
uint8 lose_line_flag;
float middle_Line_err,middle_Line_last_err;
extern int8 prospect;

//������ر���
int middle_line_output_val,middle_line_output_old_val,middle_line_smoothness_output_val,middle_line_smoothness_output_old_val,direction_output_val,direction_output_last_val;
int direction_count;//ƽ���������
int speed_err;
float direction_gyro_err,direcion_gyro_last_err;

//����������
float left_inductance,middle_inductance,right_inductance;


//�ٶȻ���ر���
int left_encoder_val,right_encoder_val,left_encoder_old_val,right_encoder_old_val; //��������ֵ
float   left_speed_val,right_speed_val; //ͨ����������������ҳ��ֵ����ٶ�
float   middle_speed_old_val,middle_speed_val,middle_encoder_old_val,middle_encoder_val;//��ģ�ٶȣ�ͨ�����ҳ��ֵ����ٶȾ�ֵ�õ�
int speed_output_old_val,speed_smoothness_output_val,speed_smoothness_output_old_val;
//int speed_output_val;
int speed_integral_val,speed_last_err;
int speed_count;

//�����ת��־λ
u8 left_motor_crazy_flag,right_motor_crazy_flag;
u8 run_flag;//���б�־λ��0��ʾδ��ת��1��ʾ��ת,2�������ֹͣ���ת����//����תʱֹͣ�ٶȻ����У����ٶȻ��ȶ���

//���PWM��������
int motor_right_val,motor_left_val,motor_old_right_val,motor_old_left_val;
//���Ƶ��ת�ٵ�ѹ,PWM�ź�����20%
//ֱ���Ƕȿ���+15��~-15��
//��ؼܽӴ�����ʱ�Ƕ�Ϊ52.3
//���̽Ӵ�����ʱ�Ƕ�Ϊ0.52
//��ֱ̥��Ϊ62mm
//����תһȦ512�߱�����ʾ��ԼΪ1157
//#define static_up_angle_P  48.5//82.5

//��������ֱ���Ƕ�
#define angle_down_limit  -0//-0.52
#define angle_up_limit    -50.8   //-52.3

//PWM����//ת������
//ת�ٹ������׵������ص�ѹ����
#define PWM_limit   800 //��1000ʱΪ100%PWM����������ȫ�����ʱ�����ѹ�������׵������ص�ѹ����

#if  parameter_mode==8//�ٶȻ���PI���ƣ��Ƚ��ȶ����ٶ�180��ʵ���ȶ�150����
//ֱ��PID����
//���ٶȻ�PD��������
#define static_angle_speed_P   3.04//6.8//34//58//102//64
#define static_angle_speed_I   0.012
#define static_angle_speed_D   0//0.18//-0.84//-12
#define mpu_gyro_integral_limit 600//�����޷�����

//�ǶȻ�ͨ��PD��������
#define static_angle_D  0.0024//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  0.086//0.064//0.098//0.143//-48//-12//42//78//88//108//112//122//142
float balance_angle=-30.4;//34.8;//34.4//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//30.4ƽ��Ƕ�//ǰ��ʱ�Ƕ�����
#define angle_output_limit 1100

//�ٶȻ�ͨ��PI����//δ�Ӽ����޷����ٲ��ȶ������׹���//��P���ƣ����׹����𵴣��������ȶ���(�ٶȻζ��ϴ�)
#define speed_P  3.34//-0.98//171//4.8//32.4//-48.6
#define speed_I  0.32//0.24
#define speed_D  0//76
#define speed_destination 0//��ģ�����ٶ�-Ϊ��ط���
#define speed_curve_destination 0//��ģת��������ٶ�
#define speed_output_limit 1100
#define speed_up_limit 50
#define speed_integral_limit 500//��������
int speed_val_I=150;
int speed_val_P=150;
#define speed_target_val 150//Ŀ���ٶ�


float middle_line_P=158;//20.0;//����P����
float middle_line_I=0;
float middle_line_D=0;//����D���� 

int middle_line_PWM_limit=1500;
int direction_output_limit=180;//�ڻ��޷�����

//�����ڻ�
float direction_P=0.28;//0.8;
float direction_I=0;
float direction_D=-0.0;

#endif

#if  parameter_mode==9//�ٶȻ�ʹ��PI����
//ֱ��PID����
//���ٶȻ�PD��������
#define static_angle_speed_P   3.04//6.8//34//58//102//64
#define static_angle_speed_I   0.012
#define static_angle_speed_D   0//0.18//-0.84//-12
#define mpu_gyro_integral_limit 600//�����޷�����

//�ǶȻ�ͨ��PD��������
#define static_angle_D  0.0024//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  0.089//0.064//0.098//0.143//-48//-12//42//78//88//108//112//122//142
float balance_angle=-30.4;//34.8;//34.4//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//30.4ƽ��Ƕ�//ǰ��ʱ�Ƕ�����
#define angle_output_limit 1100

//�ٶȻ�ͨ��PI����//δ�Ӽ����޷����ٲ��ȶ������׹���//��P���ƣ����׹����𵴣��������ȶ���(�ٶȻζ��ϴ�)
#define speed_P  2.54//-0.98//171//4.8//32.4//-48.6
#define speed_I  0.274//0.24
#define speed_D  0//76

#define speed_destination 0//��ģ�����ٶ�-Ϊ��ط���
#define speed_curve_destination 0//��ģת��������ٶ�
#define speed_output_limit 1100
#define speed_up_limit 50
#define speed_integral_limit 300//��������
int speed_val_I=180;
int speed_val_P=180;
#define speed_target_val 180//Ŀ���ٶ�


float middle_line_P=158;//20.0;//����P����
float middle_line_I=0;
float middle_line_D=0;//����D���� 

int middle_line_PWM_limit=1500;
int direction_output_limit=180;//�ڻ��޷�����

//�����ڻ�
float direction_P=0.28;//0.8;
float direction_I=0;
float direction_D=-0.0;


#endif

#if  parameter_mode==10//�ٶȻ�ʹ��PI����
//ֱ��PID����
//���ٶȻ�PD��������
#define static_angle_speed_P   3.04//6.8//34//58//102//64
#define static_angle_speed_I   0.012
#define static_angle_speed_D   0//0.18//-0.84//-12
#define mpu_gyro_integral_limit 600//�����޷�����

//�ǶȻ�ͨ��PD��������
#define static_angle_D  0.0024//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  0.089//0.064//0.098//0.143//-48//-12//42//78//88//108//112//122//142
float balance_angle=-30.4;//34.8;//34.4//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//30.4ƽ��Ƕ�//ǰ��ʱ�Ƕ�����
#define angle_output_limit 1100

//�ٶȻ�ͨ��PI����//δ�Ӽ����޷����ٲ��ȶ������׹���//��P���ƣ����׹����𵴣��������ȶ���(�ٶȻζ��ϴ�)
#define speed_P  2.04//-0.98//171//4.8//32.4//-48.6
#define speed_I  0.278//0.24
#define speed_D  0//76

#define speed_destination 0//��ģ�����ٶ�-Ϊ��ط���
#define speed_curve_destination 0//��ģת��������ٶ�
#define speed_output_limit 1100
#define speed_up_limit 50
#define speed_integral_limit 300//��������
int speed_val_I=220;
int speed_val_P=220;
#define speed_target_val 220//Ŀ���ٶ�


float middle_line_P=158;//20.0;//����P����
float middle_line_I=0;
float middle_line_D=0;//����D���� 

int middle_line_PWM_limit=1500;
int direction_output_limit=180;//�ڻ��޷�����

//�����ڻ�
float direction_P=0.28;//0.8;
float direction_I=0;
float direction_D=-0.0;


#endif


float var[3];

int angle_speed_output_val;
//int angle_output_val;
float angle_last;
uint8 crazy_time;

uint8 ms_task,first_cycle_flag=0;

float angle_y_add,mpu_gyro_z_add;//mpu6050��ֵ�˲�����

//mpu6050��ر���
extern float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050�����ĽǶȣ�δ�˲���
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern float angle,angle_dot,angle1,angle1_dot;
//mpu6050��ֵ�˲�����

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//���ٶȻ��������ǶȻ�
//���ǶȻ����������ٶȻ�
#if 1//����PID���� by noir
#define speed_out_limit 0

uint8 ms_task;
float angle_output_val,speed_output_val;
int motor_val;
float angle_speed_last_val;

void PIT0_IRQHandler(void)//2ms�ǶȻ�
{
    PIT_FlAG_CLR(pit0);
  
    ms_task++;
  
    //2ms��ȡһ��mpu6050���о�ֵ�˲�
    //��ȡmpu6050������
    Get_AccData();
    Get_Gyro();
    Get_encoder();
//    Get_AdcData();
    
    mpu_normalization_point();//��һ��
    speed_measure();//��������ֵ����
//    way_judge();//�����ж�(δ���)
    
    if((ms_task%5)==0)//10ms�ٶȻ�
    {                
        middle_speed_val=middle_encoder_val-speed_val_P;
        
        speed_integral_val+=middle_speed_val;//����
        if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;//�����޷�
        else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
        
        speed_output_val=speed_P*(middle_speed_val)+speed_I*(speed_integral_val-speed_val_I)+speed_D*(middle_speed_val-middle_speed_old_val);//PID
      
        middle_speed_old_val=middle_speed_val;
        
//        //�ٶȻ������޷�
//        if(speed_output_val-speed_output_old_val>speed_up_limit) speed_output_val+=speed_up_limit;
//        else if(speed_output_val-speed_output_old_val<-speed_up_limit) speed_output_val-=speed_up_limit;
//          
//        speed_output_old_val=speed_output_val;
        
        if(speed_output_val>speed_output_limit) speed_output_val=speed_output_limit;//�ٶȻ�����޷�
        else if(speed_output_val<-speed_output_limit) speed_output_val=-speed_output_limit;
      
    }
    if((ms_task%3)==0)//6ms�ǶȻ�
    { 
       
        sensor.Acc_mmss.x=mpu_acc_float_x;
        sensor.Acc_mmss.y=mpu_acc_float_y;
        sensor.Acc_mmss.z=mpu_acc_float_z;

        sensor.Gyro_deg.x=mpu_gyro_float_x;
        sensor.Gyro_deg.y=mpu_gyro_float_y;
        sensor.Gyro_deg.z=mpu_gyro_float_z;
       
      
       IMU_update(1,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data);//��Ԫ������
//      ���ú��������ת��Ϊ�����������������ת��ϵ
//       angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180.0/3.1415926;//���ٶ�ͨ�����Ǻ�������
       kalman_filter(imu_data.pit,mpu_gyro_float_y);//imu_data.pit
       //��е���LED��ʾ�����жϺ���
       if(angle>balance_angle)
       {
          LED_Ctrl(LEDALL,OFF);
          LED_Ctrl(LED0,ON);
       }
       else if(angle<balance_angle)
       {
          LED_Ctrl(LEDALL,OFF);
          LED_Ctrl(LED1,ON);
       }
       
       if(R_curve_flag||L_curve_flag)//�����⣨�ı��е��㣩ת��ʱ�ᵼ�³�ģ������ͨ���ı��е������ƺ�������ͬʱ����ת����٣�//R_curve_flag||L_curve_flag((middle_Line-theory_middle_line)>8||(middle_Line-theory_middle_line)<-8
       {
         balance_angle=-31.4;
       }
       else 
       {
         balance_angle=-30.4;
       }
       
       LED_Ctrl(LED2,OFF);
       angle_output_val=static_angle_P*(100*(angle-balance_angle)-speed_output_val)+static_angle_D*((100*(angle-balance_angle)-speed_output_val)-angle_last);
       angle_last=100*(angle-balance_angle)-speed_output_val;
      
       if(angle_output_val>angle_output_limit) angle_output_val=angle_output_limit;
       else if(angle_output_val<-angle_output_limit) angle_output_val=-angle_output_limit;
       
       mpu_acc_float_z=0;
       mpu_acc_float_x=0;
    }
    if((ms_task%2)==0)//4ms����ƫ��򻷣��⻷��
    {
      //ת��
      find_middle_line();
     
      Image_Decompression(image_bin,image_dec[0]);
      dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
     
      middle_Line_err=(float)middle_Line-theory_middle_line;
      
      middle_line_parameter();//���ߵ�ǰ�ٶȸ���ת�򻷲���
      
      middle_line_output_val=(int)((middle_Line_err)*(float)middle_line_P+(float)middle_line_D*middle_Line_last_err+middle_line_I);
      
      middle_Line_last_err=middle_Line_err;
//      //�������Ʒ���PWM���� (ʹת��Բ��)        
//      middle_line_output_old_val=(int)((float)middle_line_output_old_val*0.7+(float)middle_line_output_val*0.3);
    
      //����(�⻷)�޷�
      if(middle_line_output_val>middle_line_PWM_limit)
         middle_line_output_val=middle_line_PWM_limit;
      else if(middle_line_output_val<-middle_line_PWM_limit)
         middle_line_output_val=-middle_line_PWM_limit;
    }
    //2ms���ٶȻ�  
    mpu_gyro_integral_val+=(mpu_gyro_float_y*10-angle_output_val);
    
    //���ٶȻ����޷�
    if(mpu_gyro_integral_val>mpu_gyro_integral_limit) mpu_gyro_integral_val=mpu_gyro_integral_limit;
    else if(mpu_gyro_integral_val<-mpu_gyro_integral_limit) mpu_gyro_integral_val=-mpu_gyro_integral_limit;

    
    motor_val=(int)(static_angle_speed_P*(mpu_gyro_float_y*10-angle_output_val)+static_angle_speed_I*(mpu_gyro_integral_val)+static_angle_speed_D*((mpu_gyro_float_y*10-angle_output_val)-angle_speed_last_val));//PI
    angle_speed_last_val=mpu_gyro_float_y*10-angle_output_val;
    
    //2ms������ٶȻ����ڻ���
    direction_gyro_err=mpu_gyro_float_z*100-middle_line_output_val;
    
    direction_output_val=(int)(direction_P*(direction_gyro_err)+direction_D*direcion_gyro_last_err);//����PID������Ϊ����������ƽ�����
    
    direction_output_last_val=(int)(0.3*direction_output_last_val+0.7*direction_output_val);//ת�������޷�
    
    if(direction_output_last_val>direction_output_limit) direction_output_last_val=direction_output_limit;
    else if(direction_output_last_val<-direction_output_limit) direction_output_last_val=-direction_output_limit;
    
    motor_right_val=motor_val+direction_output_last_val;
    motor_left_val=motor_val-direction_output_last_val;
    
    var[0]=middle_encoder_val;
    var[1]=angle;
    var[2]=middle_Line-theory_middle_line;
    
//    var[0]=middle_inductance;
//    var[1]=right_inductance;
//    var[2]=left_inductance;
    
    vcan_sendware((void*)var,sizeof(var));
    
    //��ֹС��δ��������ʱ����
    if(angle>angle_down_limit||angle<angle_up_limit||lose_line_flag||run_flag==0)//angle<=-75.7||
    {    
        motor_right_val=0;
        motor_left_val=0;
    }
    motor_control(motor_right_val,motor_left_val);
    
    if(ms_task>=60)//120ms����
    {
        ms_task=0;//��������
        if(run_flag==0&&first_cycle_flag==0)//��һ���������ڿ������˲���Ҫ�������ڣ����Ե�һ�����ڵ��������
        {
          first_cycle_flag=1;
          run_flag=1;
        }
    }
}

#endif


////�ǶȻ�PID�������
//void angle_output(void)
//{
//  angle_output_val=(int)(static_angle_P*(angle-balance_angle)+(angle-angle_last)*static_angle_D);
//  
//  angle_last=angle;
//}

////���ٶȻ�PID�������
//void angle_speed_output(void)
//{
//  angle_speed_output_val=(int)(static_angle_speed_P*(mpu_gyro_float_y-(float)0)+static_angle_speed_D*(mpu_gyro_float_y-last_mpu_gyro_float_y));
//  
//  last_mpu_gyro_float_y=mpu_gyro_float_y;
//}

void Get_encoder()
{
  //�����������
  left_encoder_val=-ftm_quad_get(ftm1);
  right_encoder_val=ftm_quad_get(ftm2);
  
//  middle_encoder_val=(left_encoder_val+right_encoder_val)/2;
  
  ftm_quad_clean(ftm1);
  ftm_quad_clean(ftm2);
}

void Get_AdcData(void)
{
    middle_inductance=adc_once(ADC1_SE13,ADC_12bit);
    right_inductance=adc_once(ADC1_SE12,ADC_12bit);
    left_inductance=adc_once(ADC1_SE10,ADC_12bit);
        
    //��һ��
    middle_inductance=middle_inductance*3.3/4096;
    left_inductance=left_inductance*3.3/4096;
    right_inductance=right_inductance*3.3/4096;
    
}

//�Ա��������д���
//�������תʱ�����ٶȻ��رյȴ����ȶ�
//�������תʱ��ʼ�����������������תһ��ʱ��رյ������ֹ��ģײ��//δ���
void speed_measure(void)////δ���
{
  //������ת���
  if(left_encoder_val>speed_target_val+50)//��ת
  {
    left_motor_crazy_flag=1;
  }
  else if(left_encoder_val<-10)//��ת
  {
    left_motor_crazy_flag=-1;
  }
  else if(left_encoder_val-left_encoder_old_val>30)//���ּ���
  {
    left_motor_crazy_flag=2;
  }
  else 
  {
    left_motor_crazy_flag=0;//�����������Χ��ת
  }

  //�ҵ����ת���
  if(right_encoder_val>speed_target_val+50)//��ת
  {
    right_motor_crazy_flag=1;
  }
  else if(right_encoder_val<-10)//��ת
  {
    right_motor_crazy_flag=-1;
  }
  else if(right_encoder_val-right_encoder_old_val>30)//���ּ���
  {
    right_motor_crazy_flag=2;
  }
  else 
  {
    right_motor_crazy_flag=0;//���������Χ��ת
  }
  
  if(right_motor_crazy_flag>0)//�ҵ���ٶȿ���
  {
    right_encoder_val=(int)(right_encoder_val*0.5+right_encoder_old_val*0.5);
    right_encoder_old_val=right_encoder_val;
  }
  else
  {
    right_encoder_val=(int)(right_encoder_val*0.9+right_encoder_old_val*0.1);
    right_encoder_old_val=right_encoder_val;
  }
  
  if(left_motor_crazy_flag>0)//�����ٶȿ���
  {
    left_encoder_val=(int)(left_encoder_val*0.5+left_encoder_old_val*0.5);
    left_encoder_old_val=left_encoder_val;
  }
  else 
  {
    left_encoder_val=(int)(left_encoder_val*0.9+left_encoder_old_val*0.1);
    left_encoder_old_val=left_encoder_val;
  }
  
  //�����ת����
  if(left_motor_crazy_flag>0&&right_motor_crazy_flag>0)//����ͬʱ�����ת����(��ȥ��ת)
  {
    middle_encoder_val=middle_encoder_old_val;
//    crazy_time++;
  }
  
  if(left_motor_crazy_flag>0)//������ת����
  {
    if(right_encoder_val>speed_target_val)
    {
      middle_encoder_val=middle_encoder_old_val;
//      crazy_time++;
    }
    else 
    {
      middle_encoder_val=right_encoder_val;//������ת���ҵ�����ٶ���Ϊ�����ٶ�
    }
  }
  
  if(right_motor_crazy_flag>0)
  {
    if(left_encoder_val>speed_target_val)
    {
      middle_encoder_val=middle_encoder_old_val;
//      crazy_time++;
    }
    else 
    {
      middle_encoder_val=left_encoder_val;
    }
  }
  
  if(right_motor_crazy_flag==0&&left_motor_crazy_flag==0)//���ҵ����������Χ����ת
  {  
    middle_encoder_val=(left_encoder_val+right_encoder_val)/2;
  
    middle_encoder_val=middle_encoder_val*0.8+middle_encoder_old_val*0.2;
    middle_encoder_old_val=middle_encoder_val;
  }
  
//  if(crazy_time>200)
//  {
//    run_flag=0;
//  }
}


//ɽ���ѹ����
//ѹ����ֵ��ͼ���ѹ���ռ� �� ʱ�� ��ѹ��
//srclen �Ƕ�ֵ��ͼ���ռ�ÿռ��С
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {255, 0}; //0 �� 1 �ֱ��Ӧ����ɫ
    //ע��ɽ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
    uint8 tmpsrc;
    while(srclen --)
    {
        tmpsrc = *src++;
        *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
    }
}

//������ƺ���
void motor_control(int motor_right_val,int motor_left_val)
{
     if(motor_right_val>=0)
    {
      if(motor_right_val>PWM_limit) motor_right_val=PWM_limit;
      ftm_pwm_duty(ftm3,ftm_ch1,motor_right_val+right_deal_val);
      ftm_pwm_duty(ftm3,ftm_ch0,0);
    }
    else if(motor_right_val<0)
    {
      if(motor_right_val<=-PWM_limit) motor_right_val=-PWM_limit;
      ftm_pwm_duty(ftm3,ftm_ch1,0);
      ftm_pwm_duty(ftm3,ftm_ch0,-motor_right_val+right_deal_val);
    }
    
    if(motor_left_val>=0)
    {
      if(motor_left_val>PWM_limit) motor_left_val=PWM_limit;
      ftm_pwm_duty(ftm3,ftm_ch3,motor_left_val+left_deal_val);
      ftm_pwm_duty(ftm3,ftm_ch2,0);
    }
    else if(motor_left_val<0)
    {
      if(motor_left_val<-PWM_limit) motor_left_val=-PWM_limit;
      ftm_pwm_duty(ftm3,ftm_ch2,-motor_left_val+left_deal_val);
      ftm_pwm_duty(ftm3,ftm_ch3,0);
    }  
}

////�����жϺ���
////�����������;����ٶȻ�Ŀ���ٶ�
//void way_judge(void)
//{
//  if((middle_Line-theory_middle_line)>8&&(middle_Line-theory_middle_line)<-8)//�����ж�
//  {
//    else if()
//  }
//  else if()
//  {
//  }
//  
//}

//���ݵ�ǰ�ٶȾ���ת�򻷲���
//���Ը����ٶȸı�ǰհ
void middle_line_parameter(void)
{
  if(middle_encoder_val<100)  
  {
      middle_line_P=124;//20.0;//����P����
      middle_line_I=0;
      middle_line_D=20;//����D���� 

      middle_line_PWM_limit=1500;
      direction_output_limit=120;//�ڻ��޷�����

      //�����ڻ�
      direction_P=0.28;//0.8;
      direction_I=0;
      direction_D=0.06;
      
      prospect=35;
  }
  else if(middle_encoder_val>=100&&middle_encoder_val<=140)
  {
      middle_line_P=168;//20.0;//����P����
      middle_line_I=0;
      middle_line_D=22;//����D���� 

      middle_line_PWM_limit=1500;
      direction_output_limit=160;//�ڻ��޷�����

      //�����ڻ�
      direction_P=0.28;//0.8;
      direction_I=0;
      direction_D=0.08;
      
      prospect=33;
  }
  else if(middle_encoder_val>=140&&middle_encoder_val<180)
  {
       middle_line_P=232;//20.0;//����P����
       middle_line_I=0;
       middle_line_D=24;//����D���� 

       middle_line_PWM_limit=1500;
       direction_output_limit=200;//�ڻ��޷�����

       //�����ڻ�
       direction_P=0.32;//0.8;
       direction_I=0;
       direction_D=0.10;
       
       prospect=29;
  }
  else if(middle_encoder_val>=180&&middle_encoder_val<200)
  {
       middle_line_P=256;//20.0;//����P����
       middle_line_I=0;
       middle_line_D=26;//����D���� 

       middle_line_PWM_limit=1500;
       direction_output_limit=220;//�ڻ��޷�����

       //�����ڻ�
       direction_P=0.32;//0.8;
       direction_I=0;
       direction_D=0.12;
       
       prospect=27;
  }
}


void mpu_normalization_point()//��һ��
{
#if offset_option==1//offset_option
    mpu_gyro_x-=mpu_gyro_x_offset;
    mpu_gyro_y-=mpu_gyro_y_offset;
    mpu_gyro_z-=mpu_gyro_z_offset;
#endif
  
//    �������ǵ��ٶȻ���
  mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
  mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;
  mpu_gyro_float_x=(float)mpu_gyro_x*0.0610361;//����
  
//  
  mpu_acc_float_x=(float)mpu_acc_x*0.2392615;
  mpu_acc_float_y=(float)mpu_acc_y*0.2392615;
  mpu_acc_float_z=(float)mpu_acc_z*0.2392615;

//  mpu_acc_float_x+=(float)mpu_acc_x*(float)0.2392615/(float)3.0;
//  mpu_acc_float_y+=(float)mpu_acc_y*(float)0.2392615/(float)3.0;
//  mpu_acc_float_z+=(float)mpu_acc_z*(float)0.2392615/(float)3.0;
}


//����PID����ƫ����Ϊ����������ʹ��ƽ�����
//����ƽ���������
//int direction_smoothness_output()
//{
//  if(direction_count>=2)
//    direction_count=0;
//  direction_count++;
//
//  
//  middle_line_smoothness_output_old_val=middle_line_smoothness_output_val;
//  
//  middle_line_smoothness_output_val=middle_line_smoothness_output_old_val+(int)((float)(middle_line_output_val-middle_line_smoothness_output_old_val)*(float)direction_count/2.0);//��������Ϊ12ms//ÿ2�������һ�ε������
//  
//  return middle_line_smoothness_output_val;
//}

//����PID�ٶȻ���Ϊ����������ʹ��ƽ�����
//�ٶȻ�ƽ���������
//int speed_smoothness_output()
//{
//  if(speed_count>=5)
//    speed_count=0;
//  speed_count++;
//  
//  speed_smoothness_output_old_val=speed_smoothness_output_val;
//  
//  speed_smoothness_output_val=speed_smoothness_output_old_val+(int)((float)(speed_output_val-speed_smoothness_output_old_val)*(float)speed_count/5.0);//�ٶȻ�����Ϊ4ms//ÿ2�������һ�ε������
//  
//  return speed_smoothness_output_val;
//}

//#if  parameter_mode==8//�ٶȭh��P����//�ǶȻ�����ٶȻ�����Ӳת�䵼�³��岻�ȶ���
////ֱ��PID����
////���ٶȻ�PD��������
//#define static_angle_speed_P   3.6//6.8//34//58//102//64
//#define static_angle_speed_I   0.0
//#define static_angle_speed_D  -0.4//-12
//#define mpu_gyro_integral_limit 10//�����޷�����
//
////�ǶȻ�ͨ��PD��������
//#define static_angle_D  0.014//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  0.128//0.064//0.098//0.143//-48//-12//42//78//88//108//112//122//142
//float balance_angle=-33.8;//34.8;//34.4//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//33.4ƽ��Ƕ�//ǰ��ʱ�Ƕ�����
//#define angle_output_limit 1100
//
////�ٶȻ�ͨ��PI����
//#define speed_P  -2.8//171//4.8//32.4//-48.6
//#define speed_I  -3.42
//#define speed_D  -0//76
//#define speed_destination 0//��ģ�����ٶ�-Ϊ��ط���
//#define speed_curve_destination 0//��ģת��������ٶ�
//#define speed_output_limit 1100
//#define speed_up_limit 1000
//#define speed_integral_limit 1000//��������
//int speed_val_I=70;
//int speed_val_P=70;
//#define speed_target_val 0//Ŀ���ٶ�
//
//
//float middle_line_P=20;//20.0;//����P����
//float middle_line_I=0;
//float middle_line_D=0;//����D���� 
//
//int middle_line_PWM_limit=900;
//int direction_output_limit=80;//�ڻ��޷�����
//
////�����ڻ�
//float direction_P=0.8;//0.8;
//float direction_I=0;
//float direction_D=0;
//
//#endif


////2ms����һ���ж�
////�ǶȻ�6ms�ٶȻ�12ms����8ms
////4ms����һ�ε������
////PID����ֱ��
////���ٶȻ�Ϊ�����ǶȻ���
//#if 0
//
//void PIT0_IRQHandler(void)
//{
//  PIT_FlAG_CLR(pit0);
//  
//  ms_task++;
//  
//  //2ms��ȡһ��mpu6050���о�ֵ�˲�
//  //��ȡmpu6050������
//  Get_AccData();
//  Get_Gyro();
////  mpu_acc_x-=mpu_acc_x_offset;
////  mpu_acc_y-=mpu_acc_y_offset;
////  mpu_acc_z-=mpu_acc_z_offset;
//  
//  //    �������ǵ��ٶȻ���
//  mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
//  mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;
//  mpu_gyro_float_x=(float)mpu_gyro_x*0.0610361;//����
// 
//  angle_speed_output();
//  
//  mpu_acc_float_x+=(float)mpu_acc_x*0.2392615/(float)3.0;
//  mpu_acc_float_y+=(float)mpu_acc_y*0.2392615/(float)3.0;
//  mpu_acc_float_z+=(float)mpu_acc_z*0.2392615/(float)3.0;
//   
//  
////  motor_right_val=(int)angle_output_val+angle_speed_output_val-speed_smoothness_output()+direction_smoothness_output();
////  motor_left_val=(int)angle_output_val+angle_speed_output_val-speed_smoothness_output()-direction_smoothness_output();
//    
////  motor_old_right_val=(int)((float)motor_right_val*0.9+motor_old_right_val*0.1);//��ͨ�˲�
////  motor_old_left_val=(int)((float)motor_left_val*0.9+motor_old_left_val*0.1);//��ͨ�˲�
//  
//  //��ֹС��δ��������ʱ����
//  if(angle>=angle_down_limit||angle<=angle_up_limit)//angle<=-75.7||
//  {    
//    motor_right_val=0;
//    motor_left_val=0;
//  }
//  motor_control(motor_right_val,motor_left_val);
//   
//  if((ms_task%3)==0)//mpu6050�˲�
//  {
//    sensor.Acc_mmss.x=mpu_acc_float_x;
//    sensor.Acc_mmss.y=mpu_acc_float_y;
//    sensor.Acc_mmss.z=mpu_acc_float_z;
//
//    sensor.Gyro_deg.x=mpu_gyro_float_x;
//    sensor.Gyro_deg.y=mpu_gyro_float_y;
//    sensor.Gyro_deg.z=mpu_gyro_float_z;
//
//    
//    //���ü��ٶȼƼ������(�Ƕ�ֵ)
//    //���ú��������ת��Ϊ�����������������ת��ϵ
//    angle_y_now=atan2(mpu_acc_float_x,mpu_acc_float_z)*(float)180/3.1415926;//���ٶ�ͨ�����Ǻ�������
//    IMU_update(1,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data);
//    kalman_filter(imu_data.pit,mpu_gyro_float_y);//imu_data.pit
////    sprintf((char*)str,"%f",angle);
////    OLED_P6x8Str(84,16,str);
//    angle_output();//�ǶȻ�
//    
//     //ת��
//     find_middle_line();
//     
//     Image_Decompression(image_bin,image_dec[0]);
//     dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
//     
//     direction_output_val=(int)(((float)middle_Line-theory_middle_line)*(float)direction_P+(float)direction_D*(float)mpu_gyro_z_add+direction_I);
//   
////�������Ʒ���PWM���� (ʹת��Բ��)        
//    direction_output_old_val=(int)((float)direction_output_old_val*0.8+(float)direction_output_val*0.2);
//    
//     //�����޷�
//     if(direction_output_val>direction_PWM_limit)
//       direction_output_val=direction_PWM_limit;
//     else if(direction_output_val<-direction_PWM_limit)
//       direction_output_val=-direction_PWM_limit;
//     
//    
//
//
//    //��λ���������ǲ��γ���    
//    var[0]=mpu_gyro_float_y;
//    var[1]=angle;
//    var[2]=imu_data.pit;
//    var[3]=angle_y_now;
//    
//    vcan_sendware((void*)var,sizeof(var));
//  
//    mpu_acc_float_x=0;
//    mpu_acc_float_y=0;
//    mpu_acc_float_z=0;
//  }
//
//  if(ms_task>=12)
//  {
//      Get_encoder();
//    
//    //�ٶȻ�//�����޷�������
//#if 1 
//      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
//      
//      OLED_Print_Num1(84,0,middle_encoder_val);
//            
//      speed_integral_val+=middle_encoder_val;
//      
//      //�����޷�//������
//      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
//      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
//      
//      //�����޷�
////      if(L_curve_flag||R_curve_flag) 
////      {
////        if(middle_encoder_val-speed_curve_destination>speed_up_limit) middle_encoder_val+=speed_up_limit;
////        else if(middle_encoder_val-speed_curve_destination<speed_up_limit) middle_encoder_val-=speed_up_limit;
////        else middle_encoder_val=speed_curve_destination;
////      }
////      else if(lose_line_flag==1)     //ȫ�����ٶȻ�Ϊ0  
////      {
////        if(middle_encoder_val-0>speed_up_limit) middle_encoder_val+=speed_up_limit;
////        else if(middle_encoder_val-0<speed_up_limit) middle_encoder_val-=speed_up_limit;
////        else middle_encoder_val=0;
////
////      }
////      else 
////      {
////        if(middle_encoder_val-speed_destination>speed_up_limit) middle_encoder_val+=speed_up_limit;
////        else if(middle_encoder_val-speed_destination<speed_up_limit) middle_encoder_val-=speed_up_limit;
////        else middle_encoder_val=speed_destination;
////      }
//      
////       speed_output_val=(int)(speed_P*(middle_encoder_val+600)+speed_I*(speed_integral_val+speed_destination));
//      
//      
////      if(speed_output_val>speed_output_limit) speed_output_val=speed_output_limit;
////      else if(speed_output_val<-speed_output_limit) speed_output_val=-speed_output_limit;
//      
//      right_encoder_val=0;
//      left_encoder_val=0;
//#endif  
//      ms_task=0;
//  }
//}
//#endif
//

////ֱ����
////2ms����һ���ж�
////�ǶȻ�6ms�ٶȻ�12ms����8ms
////4ms����һ�ε������
//#if 0
//uint8 ms_task;
//float angle_y_add,mpu_gyro_z_add;//mpu6050��ֵ�˲�����
//
////mpu6050��ر���
//float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050�����ĽǶȣ�δ�˲���
//extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
//extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
//extern float angle,angle_dot,angle1,angle1_dot;
////mpu6050��ֵ�˲�����
//
////float var[4];//��λ������
//
//void PIT0_IRQHandler(void)
//{
//  PIT_FlAG_CLR(pit0);
//  
//  ms_task++;
//  
//  //2ms��ȡһ��mpu6050���о�ֵ�˲�
//  //��ȡmpu6050������
//  Get_AccData();
//  Get_Gyro();
//  Get_encoder();
//  
//  //    �������ǵ��ٶȻ���
//  mpu_gyro_float_y+=(float)mpu_gyro_y*0.0610361/(float)3.0;
//  mpu_gyro_float_z+=(float)mpu_gyro_z*0.0610361/(float)3.0;
//  mpu_gyro_float_x+=(float)mpu_gyro_x*0.0610361/(float)3.0;//����
// 
//
//  
//  mpu_acc_float_x+=(float)mpu_acc_x*0.2392615/(float)3.0;
//  mpu_acc_float_y+=(float)mpu_acc_y*0.2392615/(float)3.0;
//  mpu_acc_float_z+=(float)mpu_acc_z*0.2392615/(float)3.0;
//   
//  
//  motor_right_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()+direction_smoothness_output();
//  motor_left_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()-direction_smoothness_output();
//    
////  motor_old_right_val=(int)((float)motor_right_val*0.9+motor_old_right_val*0.1);//��ͨ�˲�
////  motor_old_left_val=(int)((float)motor_left_val*0.9+motor_old_left_val*0.1);//��ͨ�˲�
//
//  
//  //��ֹС��δ��������ʱ����
//  if(angle>=angle_down_limit||angle<=angle_up_limit)//angle<=-75.7||
//  {    
//    motor_right_val=0;
//    motor_left_val=0;
//  }
//  motor_control(motor_right_val,motor_left_val);
//  if((ms_task%3)==0)//mpu6050�˲�
//  {
//    sensor.Gyro_deg.x=mpu_gyro_float_x;
//    sensor.Gyro_deg.y=mpu_gyro_float_y;
//    sensor.Gyro_deg.z=mpu_gyro_float_z;
//    
//    sensor.Acc_mmss.x=mpu_acc_float_x;
//    sensor.Acc_mmss.y=mpu_acc_float_y;
//    sensor.Acc_mmss.z=mpu_acc_float_z;
//    
//    //���ü��ٶȼƼ������(�Ƕ�ֵ)
//    //���ú��������ת��Ϊ�����������������ת��ϵ
//    angle_y_now=atan2(mpu_acc_float_x,mpu_acc_float_z)*(float)180/3.1415926;//���ٶ�ͨ�����Ǻ�������
//    IMU_update(1,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data);
//    kalman_filter(angle_y_now,mpu_gyro_float_y);//imu_data.pit
//    
//    //��е���LED��ʾ�����жϺ���
//    if(angle>balance_angle)
//    {
//      LED_Ctrl(LEDALL,OFF);
//      LED_Ctrl(LED0,ON);
//
//    }
//    else if(angle<balance_angle)
//    {
//      LED_Ctrl(LEDALL,OFF);
//      LED_Ctrl(LED1,ON);
//    }
//    
//         //�ٶȻ�
//#if 0 
//      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
//      
//      if(middle_encoder_val-middle_encoder_old_val>speed_output_limit) middle_encoder_old_val+=speed_output_limit;
//      else if(middle_encoder_val-middle_encoder_old_val<-speed_output_limit) middle_encoder_old_val-=speed_output_limit;
//      else middle_encoder_old_val=middle_encoder_val;
//      
//      speed_integral_val+=middle_encoder_old_val;
//      
//      //�����޷�
//      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
//      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
//      
//      speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
//      
//#endif
//
//    //��λ���������ǲ��γ���    
//    var[0]=mpu_gyro_float_y;
//    var[1]=angle;
//    var[2]=(float)imu_data.pit;
//    var[3]=angle_y_now;
//    
//    vcan_sendware((void*)var,sizeof(var));
//  
//    mpu_gyro_float_x=0;
//    mpu_gyro_float_y=0;
//    mpu_gyro_float_z=0;
//    mpu_acc_float_x=0;
//    mpu_acc_float_y=0;
//    mpu_acc_float_z=0;
//  }
//  if(ms_task%4==0)
//  {
//     find_middle_line();
//     
//     Image_Decompression(image_bin,image_dec[0]);
//     dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
//     mpu_gyro_z_add/=(float)4;
//     
//     direction_output_val=(int)(((float)middle_Line-theory_middle_line)*(float)direction_P+(float)direction_D*(float)mpu_gyro_z_add+direction_I);
//   
////�̶�ֵ���Ʒ���PWM����
//#if 0
//     if((direction_output_val-direction_output_old_val)>direction_up_limit) direction_output_old_val+=direction_up_limit;
//     if((direction_output_val-direction_output_old_val)>direction_up_limit) 
//     
//#endif 
//     
////�������Ʒ���PWM���� (ʹת��Բ��)    
//#if 1     
//    direction_output_old_val=(int)((float)direction_output_old_val*0.6+(float)direction_output_val*0.4);
//#endif 
//    
//    
//     //�����޷�
//     if(direction_output_val>direction_PWM_limit)
//       direction_output_val=direction_PWM_limit;
//     else if(direction_output_val<-direction_PWM_limit)
//       direction_output_val=-direction_PWM_limit;
//     
//     mpu_gyro_z_add=0;
//  }
//  if((ms_task%6)==0)
//  {
//       //�ٶȻ�
//#if 1 
//      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
//      
//      if(middle_encoder_val-middle_encoder_old_val>speed_output_limit) middle_encoder_old_val+=speed_output_limit;
//      else if(middle_encoder_val-middle_encoder_old_val<-speed_output_limit) middle_encoder_old_val-=speed_output_limit;
//      else middle_encoder_old_val=middle_encoder_val;
//      
//      speed_integral_val+=middle_encoder_old_val;
//      
//      
//      //�����޷�
//      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
//      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
//      
//      if(L_curve_flag||R_curve_flag) speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_curve_destination)+speed_I*speed_integral_val);
//      else if(lose_line_flag==1)     speed_output_val=(int)(speed_P*(middle_encoder_old_val-0)+speed_I*speed_integral_val);//ȫ�����ٶȻ�Ϊ0   
//      else speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
//#endif  
//  }
//  if(ms_task>=12)
//  {
//      ms_task=0;
//  }
//}
//
//#endif

//#if  parameter_mode==0
////ֱ��PID����
////���ٶȻ�PD��������
//#define static_angle_speed_P  46//34//58//102//64
//#define static_angle_speed_D  -0//-12
//
////�ǶȻ�ͨ��PD��������
//#define static_angle_D  0//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   46  
//#define right_deal_val  46
////#define static_down_angle_P  42
//#define static_angle_P  -28//-48//-12//42//78//88//108//112//122//142
//float balance_angle=-34.8;//34.8;//34.4//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4
//
////�ٶȻ�ͨ��PI����
//#define speed_P  8.2
//#define speed_I  0
//#define speed_D  0
//#define speed_destination 300//��ģ�����ٶ�-Ϊ��ط���
//#define speed_curve_destination 300//��ģת��������ٶ�
//#define speed_output_limit 200
//#define speed_up_limit 10
//#define speed_integral_limit 500//��������
//
//float direction_P=-18;//20.0;//����P����
//float direction_I=0;
//float direction_D=0;//����D���� 
//
//int direction_PWM_limit=60;
//
//#endif
//
//
////����5
//#if parameter_mode==5
////ֱ��PID����
//
////���ٶȻ�PD��������
//#define static_angle_speed_P  74//68//74//64
//#define static_angle_speed_D  -6//-4//-14
//
////�ǶȻ�ͨ��PD��������
//#define static_angle_D  -3//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  -32//-24//-24//52
////42//78//88//108//112//122//142
//float balance_angle=-32.8;//29.2//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//26.2~26.7
//
////�ٶȻ�ͨ��PI����
//#define speed_P  2.4
//#define speed_I  0
//#define speed_D  0
//#define speed_destination -80//80//��ģ�����ٶ� 
//#define speed_curve_destination -80//��ģת��������ٶ�
//#define speed_output_limit 5
//#define speed_up_limit 200
//#define speed_integral_limit 500//��������
//
//
////����
//float direction_P=-24;//����P����
//float direction_I=0;
//float direction_D=-0;//����D���� 
//
//#define direction_up_limit 0
//
//int direction_PWM_limit=120;
//#endif


//��ǰ�Ĳ���
//ֱ�����Ŀ�������Ϊ2ms�Ĳ���
////����1
//#if  parameter_mode==1
////ֱ��PID����
////�ǶȻ�ͨ��PD��������
//#define static_angle_D  -18.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  42//42//78//88//108//112//122//142
//float balance_angle=-31.8;//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4
//
////�ٶȻ�ͨ��PI����
//#define speed_P  0.4
//#define speed_I  0.002
//#define speed_D  0
//#define speed_destination 80//��ģ�����ٶ� 
//#define speed_output_limit 30
//#define speed_up_limit 5
//#define speed_integral_limit 400//��������
//
//float direction_P=20.0;//����P����
//float direction_D=0;//����D���� 
//
//int direction_PWM_limit;
//#endif
//
////����2
//#if parameter_mode==2
////ֱ��PID����
////�ǶȻ�ͨ��PD��������
//#define static_angle_D  -17.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  42//42//78//88//108//112//122//142
//float balance_angle=-31.8;//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4
//
////�ٶȻ�ͨ��PI����
//#define speed_P  0.4
//#define speed_I  0
//#define speed_D  0
//#define speed_destination 100//��ģ�����ٶ� 
//#define speed_output_limit 100
//#define speed_up_limit 5
//#define speed_integral_limit 50//��������
//
//float direction_P=20.0;//����P����
//float direction_D=0;//����D���� 
//
//int direction_PWM_limit;
//#endif


//ֱ����Ϊ6ms�Ŀ��Ʋ���
////�Ƚ��ȵ����ٶȺ�����
//#if parameter_mode==3
////ֱ��PID����
////�ǶȻ�ͨ��PD��������
//#define static_angle_D  -34//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  52//52
////42//78//88//108//112//122//142
//float balance_angle=-29.8;//29.2//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//26.2~26.7
//
////�ٶȻ�ͨ��PI����
//#define speed_P  4.2
//#define speed_I  0.000
//#define speed_D  0
//#define speed_destination 120//80//��ģ�����ٶ� 
//#define speed_curve_destination 60//ת��ʱ��ģ�������ٶ�
//#define speed_output_limit 200
//#define speed_up_limit 5
//#define speed_integral_limit 500//��������
//
//float direction_P=16;//����P����
//float direction_I=0;
//float direction_D=-1.2;//����D���� 
//
//int direction_PWM_limit=60;
//#endif
//
////����4
//#if parameter_mode==4
////ֱ��PID����
////�ǶȻ�ͨ��PD��������
//#define static_angle_D  -34//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  58//52
////42//78//88//108//112//122//142
//float balance_angle=-28.8;//29.2//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//26.2~26.7
//
////�ٶȻ�ͨ��PI����
//#define speed_P  1.4
//#define speed_I  0.004
//#define speed_D  0
//#define speed_destination 60//80//��ģ�����ٶ� 
//#define speed_curve_destination 60//��ģת��������ٶ�
//#define speed_output_limit 200
//#define speed_up_limit 10
//#define speed_integral_limit 500//��������
//
//
////����
//float direction_P=20;//����P����
//float direction_I=0;
//float direction_D=-1.8;//����D���� 
//
//#define direction_up_limit 10
//
//int direction_PWM_limit=60;
//#endif


//ת�ٻ���Ϊ���ٶȳ���//by noir
//  left_speed_val=(float)left_encoder_val*0.038042722890625;//����512�ߵı���������
//  right_speed_val=(float)right_encoder_val*0.038042722890625;//
  
//  middle_speed_val=(left_speed_val+right_speed_val)/2.0;

//����ʽ������
//      int16_t var[3];
//      //��Ҫ���ԣ�δ���м��Բ���by noir
//      //����ʽ������
//      if(gpio_get(A8)==0)
//      {
//        left_encoder_val=ftm_input_get(ftm1,ftm_ch1);
//      }
//      else
//      {
//        left_encoder_val=-ftm_input_get(ftm1,ftm_ch1);
//      }
//      if(gpio_get(A11)==0)
//      {
//        right_encoder_val=ftm_input_get(ftm2,ftm_ch0);
//      }
//      else
//      {
//        right_encoder_val=-ftm_input_get(ftm2,ftm_ch0);
//      }
//        
//      ftm_input_clean(ftm1);
//      ftm_input_clean(ftm2);


//�ٶȻ�����    
//      
////      if(middle_encoder_val-speed_destination>speed_up_limit)
////      {
////        speed_err=speed_err+speed_up_limit;
////      }
////      else if(middle_encoder_val-speed_destination<-speed_up_limit)
////      {
////        speed_err=speed_err-speed_up_limit;
////      }
////      else speed_err=middle_encoder_val-speed_destination;
//      
//      speed_last_err=speed_err;
//      
//      speed_integral_val+=speed_err;
//      
////      if(middle_encoder)
//      
//      speed_output_val=speed_err*speed_P+speed_integral_val*speed_I+speed_D*(speed_err-speed_last_err);
//      
//      speed_count=0;//�ٶȻ�ƽ������������
//       
//     
//      //�ٶȻ��޷�
//      if(speed_output_val>speed_output_limit)
//        speed_output_val=speed_output_limit;
//      else if(speed_output_val<-speed_output_limit)
//        speed_output_val=-speed_output_limit;
////      var[0]=middle_encoder_val;
////      var[1]=middle_encoder_integral_val;
////      var[2]=speed_output_val;
////      
////      vcan_sendware((void*)var,sizeof(var)); 
      
//�ο���PID����
// λ��ʽ��̬PID����
//int32 PlacePID_Control(PID *sprt, float *PID, float NowPiont, float SetPoint)
//{
//	//����Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
//	float iError;	//��ǰ���
//	int  Actual;	//���ó���ʵ�����ֵ
//	float Kp;		//��̬P
//	
//	iError = SetPoint - NowPiont;	//���㵱ǰ���
//	sprt->SumError += iError*0.01;
//	if (sprt->SumError >= PID[KT])
//	{
//		sprt->SumError = PID[KT];
//	}
//	else if (sprt->SumError <= PID[KT])
//	{
//		sprt->SumError = -PID[KT];
//	}
//	//���κ�����Ϊ�˴ﵽ ���Խ��  ��ӦԽ�� �ظ���Խ�� ���� KIֵ�����Ϊ0ʱ��P Ҳ����ֱ���ϵ�Pֵ
//	Kp = 1.0 * (iError*iError) / PID[KP] + PID[KI];	//Pֵ���ֵ�ɶ��κ�����ϵ���˴�P��I����PID���������Ƕ�̬PID������Ҫע�⣡����
//	
//	Actual = Kp * iError
//		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//����ȫ΢��  
//	sprt->LastError = iError;		//�����ϴ����
//
//	Actual = range_protect(Actual, -260, 260);
//
//	return Actual;
//}
//
//// λ��ʽPID����
//int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
//{
//	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
//	int32 Realize;	// ���ó���ʵ������
//
//	sptr->Dis_Err = Point - NowData;	// ���㵱ǰ���
//	sptr->SumError += PID[KI] * sptr->Dis_Err;	// ������
//	
//	if (sptr->SumError >= PID[KT])
//	{
//		sptr->SumError = PID[KT];
//	}
//	else if (sptr->SumError <= -PID[KT])
//	{
//		sptr->SumError = -PID[KT];
//	}
//	
//	Realize = PID[KP] * sptr->Dis_Err
//			+ sptr->SumError
//			+ PID[KD] *(sptr->Dis_Err  - sptr->LastError);
////			+ PID[KB] * ( NowData- sptr->LastData); //΢������
//	
//	sptr->PrevError = sptr->LastError;	// ����ǰ�����
//	sptr->LastError = sptr->Dis_Err;		  	// �����ϴ����
//	sptr->LastData  = NowData;			// �����ϴ�����
//
//	return Realize;	// ����ʵ��ֵ
//}
//
//// ����ʽPID�������
//int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point)
//{
//	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
//	int32 iError,	//��ǰ���
//		Increase;	//���ó���ʵ������
//
//	iError = Point - NowData;	// ���㵱ǰ���
//
//	Increase =  PID[KP] * (iError - sptr->LastError)
//			  + PID[KI] * iError
//			  + PID[KD] * (iError - 2 * sptr->LastError + sptr->PrevError);
//	
//	sptr->PrevError = sptr->LastError;	// ����ǰ�����
//	sptr->LastError = iError;		  	// �����ϴ����
//	sptr->LastData  = NowData;			// �����ϴ�����
//	
//	return Increase;	// ��������
//}

////2ms����һ�νǶȻ�����
////4ms����һ���ٶȻ�����
////8ms����һ�η��򻷿���
//#if 0
////ƽ����̬����
//void PIT0_IRQHandler(void)//ÿ���2ms����һ�ζ�ʱ���жϺ���
//{
////  int16_t var[3];  
//  PIT_FlAG_CLR(pit0);
//  float var[6];
//  
//  ms_task_flag++;
//   
//  if((ms_task_flag%2)==0)//4ms_task�ٶȻ�
//  {     
//      //�����������
//      left_encoder_val=ftm_quad_get(ftm1);
//      right_encoder_val=-ftm_quad_get(ftm2);
//      
//      ftm_quad_clean(ftm1);
//      ftm_quad_clean(ftm2);
// 
////�ο�����ѧУ���޷�����      
////��ֹ���ּ���Ӱ��ֱ��������
//#if 0
//      
//      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
//      middle_encoder_old_val*=0.7;
//      middle_encoder_old_val+=middle_encoder_val*0.3;
//      
//      speed_integral_val+=middle_encoder_old_val;
//        
//#endif
////�ο�Э����ǰʦ�ֵ��޷�����      
//#if 1
//      
//      
//      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
//      
//      if(middle_encoder_val-middle_encoder_old_val>speed_output_limit) middle_encoder_old_val+=speed_output_limit;
//      else if(middle_encoder_val-middle_encoder_old_val<-speed_output_limit) middle_encoder_old_val-=speed_output_limit;
//      else middle_encoder_old_val=middle_encoder_val;
//      
//      speed_integral_val+=middle_encoder_old_val;
//      
//#endif
//   
//      if(middle_encoder_val<80)
//      {
//        direction_P=20;
//        direction_PWM_limit=40;
//      }
//      else if(middle_encoder_val>=80)
//      {
//        direction_P=30;
//        direction_PWM_limit=50;
//      }
//      
//      //�����޷�
//      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
//      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
//      
//      speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
//   }
//
//    
//   if((ms_task_flag%4)==0)//8ms_task����
//   {
//     find_middle_line();
//     
//     Image_Decompression(image_bin,image_dec[0]);
//     dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
//     
//     direction_output_val=(int)(((float)middle_Line-theory_middle_line)*(float)direction_P);
//      
//     direction_count=0;//
//     
//     //�����޷�
//     if(direction_output_val>direction_PWM_limit)
//       direction_output_val=direction_PWM_limit;
//     else if(direction_output_val<-direction_PWM_limit)
//       direction_output_val=-direction_PWM_limit;
//     
//     ms_task_flag=0;
//   }
//  
///**********************
//ÿ�������ȡһ��mpu6050������ (2ms����)
//***********************/
//    //��ȡmpu6050������
//    Get_AccData();
//    Get_Gyro();
//    Get_encoder();
//    
////    �������ǵ��ٶȻ���
//    mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
//    mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;//����
//    
//    //�������ǵļ��ٶȽ��л���
//    mpu_acc_float_y=(float)mpu_acc_y*0.2392615;
//    
//    //���ü��ٶȼƼ������(�Ƕ�ֵ)
//    //���ú��������ת��Ϊ�����������������ת��ϵ
//    angle_z_now=atan2(mpu_acc_y,mpu_acc_x)*(float)180/3.1415926;
//    angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180/3.1415926;
//
//    kalman_filter(angle_y_now,mpu_gyro_float_y);
//    
////    //��λ���������ǲ��γ���    
////    var[0]=mpu_gyro_float_y;
////    var[1]=angle;
////    var[2]=angle_y_now;
////    var[3]=angle_dot;
////    var[4]=motor_right_val;
////    var[5]=motor_left_val;
////    
////    vcan_sendware((void*)var,sizeof(var));
//    
//    //���ҵ��PWM����
//     motor_right_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()+direction_smoothness_output();
//     motor_left_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()-direction_smoothness_output();
//     
////     var[1]=motor_left_val;
//     
////    vcan_sendware((void*)var,sizeof(var));
//      
//    //��е���LED��ʾ�����жϺ���
//    if(angle>balance_angle)
//    {
//      LED_Ctrl(LEDALL,OFF);
//      LED_Ctrl(LED0,ON);
//    }
//    else if(angle<balance_angle)
//    {
//      LED_Ctrl(LEDALL,OFF);
//      LED_Ctrl(LED1,ON);
//    }
//    
//    if(angle>=angle_down_limit||angle<=angle_up_limit)//angle<=-75.7||
//    {    
//      motor_right_val=0;
//      motor_left_val=0;
//    }
//    
//    motor_control(motor_right_val,motor_left_val);
//}
//#endif
//
//#if  parameter_mode==7
////ֱ��PID����
////���ٶȻ�PD��������
//#define static_angle_speed_P   4.4//6.8//34//58//102//64
//#define static_angle_speed_I   0.0
//#define static_angle_speed_D  -0//-12
//#define mpu_gyro_integral_limit 0//�����޷�����
//
////�ǶȻ�ͨ��PD��������
//#define static_angle_D  0//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  0.098//-48//-12//42//78//88//108//112//122//142
//float balance_angle=-34.4;//34.8;//34.4//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//33.4ƽ��Ƕ�//ǰ��ʱ�Ƕ�����
//#define angle_output_limit 300
//
////�ٶȻ�ͨ��PI����
//#define speed_P  -14
//#define speed_I  0.08
//#define speed_D  0
//#define speed_destination 0//��ģ�����ٶ�-Ϊ��ط���
//#define speed_curve_destination 0//��ģת��������ٶ�
//#define speed_output_limit 60
//#define speed_up_limit 1000
//#define speed_integral_limit 0//��������
//int speed_val_I=-1200;
//int speed_val_P=-800;
//#define speed_target_val 0//Ŀ���ٶ�
//
////
//float middle_line_P=-18;//20.0;//����P����
//float middle_line_I=0;
//float middle_line_D=0;//����D���� 
//
//int middle_line_PWM_limit=80;
//int direction_output_limit=160;//�ڻ��޷�����
//
////�����ڻ�
//float direction_P=0.8;//0.8;
//float direction_I=0;
//float direction_D=0;
//
//#endif

//#if  parameter_mode==9
////ֱ��PID����
////���ٶȻ�PD��������
//#define static_angle_speed_P   3.6//6.8//34//58//102//64
//#define static_angle_speed_I   0.0
//#define static_angle_speed_D  -0//-12
//#define mpu_gyro_integral_limit 10//�����޷�����
//
////�ǶȻ�ͨ��PD��������
//#define static_angle_D  0.006//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  0.128//0.098//0.143//-48//-12//42//78//88//108//112//122//142
//float balance_angle=-33.2;//34.8;//34.4//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//33.4ƽ��Ƕ�//ǰ��ʱ�Ƕ�����
//#define angle_output_limit 1100
//
////�ٶȻ�ͨ��PI����
//#define speed_P  -0.64//16//43.8//4.8//32.4//-48.6
//#define speed_I  0//12.4
//#define speed_D  -0.48//4
//#define speed_destination 0//��ģ�����ٶ�-Ϊ��ط���
//#define speed_curve_destination 0//��ģת��������ٶ�
//#define speed_output_limit 1100
//#define speed_up_limit 1000
//#define speed_integral_limit 500//��������
//int speed_val_I=10;
//int speed_val_P=80;
//#define speed_target_val 0//Ŀ���ٶ�
//
//
//float middle_line_P=20;//20.0;//����P����
//float middle_line_I=0;
//float middle_line_D=0;//����D���� 
//
//int middle_line_PWM_limit=900;
//int direction_output_limit=60;//�ڻ��޷�����
//
////�����ڻ�
//float direction_P=0.8;//0.8;
//float direction_I=0;
//float direction_D=0;
//
//#endif