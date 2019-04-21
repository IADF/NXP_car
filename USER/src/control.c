#include "control.h"

#define parameter_mode 3 //0Ϊ����1Ϊ����//����3��ʼΪ��������6ms��

//���߳�����ر���
extern uint8 L_curve_flag;//�����־λ
extern uint8 R_curve_flag;//�����־λ

//mpu6050��ر���
uint8 PIT_flag;
float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050�����ĽǶȣ�δ�˲���
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern float angle,angle_dot,angle1,angle1_dot;

//�����������
uint8 str[20];

//���߲ɼ�������ر���
extern float middle_Line_temp;
extern int middle_Line;
float theory_middle_line=39.5;//��������

//������ر���
extern int direction_output_val,direction_output_old_val,direction_smoothness_output_val,direction_smoothness_output_old_val;
extern int direction_count;//ƽ���������
int speed_err;

//�ٶȻ���ر���
int left_encoder_val,right_encoder_val,middle_encoder_val; //��������ֵ
float   left_speed_val,right_speed_val; //ͨ����������������ҳ��ֵ����ٶ�
float   middle_speed_val,middle_encoder_old_val;//��ģ�ٶȣ�ͨ�����ҳ��ֵ����ٶȾ�ֵ�õ�
int speed_output_val,speed_output_old_val,speed_smoothness_output_val,speed_smoothness_output_old_val;
int speed_integral_val,speed_last_err;
int speed_count;

//���PWM��������
int motor_right_val,motor_left_val;
//���Ƶ��ת�ٵ�ѹ,PWM�ź�����20%
//ֱ���Ƕȿ���+15��~-15��
//��ؼܽӴ�����ʱ�Ƕ�Ϊ52.3
//���̽Ӵ�����ʱ�Ƕ�Ϊ0.52
//��ֱ̥��Ϊ62mm
//����תһȦ512�߱�����ʾ��ԼΪ1157
//#define static_up_angle_P  48.5//82.5

//��������ֱ���Ƕ�
#define angle_down_limit  -1//-0.52
#define angle_up_limit    -52   //-52.3

//PWM����//ת������
//ת�ٹ������׵������ص�ѹ����
#define PWM_limit   700 //��1000ʱΪ100%PWM����������ȫ�����ʱ�����ѹ�������׵������ص�ѹ����

#if  parameter_mode==0
//ֱ��PID����
//�ǶȻ�ͨ��PD��������
#define static_angle_D  0//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  52//42//78//88//108//112//122//142
float balance_angle=-31.9;//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4

//�ٶȻ�ͨ��PI����
#define speed_P  0
#define speed_I  0.0
#define speed_D  0
#define speed_destination 0//��ģ�����ٶ�
#define speed_output_limit 0
#define speed_up_limit 0
#define speed_integral_limit 0//��������

float direction_P=0;//20.0;//����P����
float direction_D=0;//����D���� 

int direction_PWM_limit;

#endif

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

//�Ƚ��ȵ����ٶȺ�����
#if parameter_mode==3
//ֱ��PID����
//�ǶȻ�ͨ��PD��������
#define static_angle_D  -34//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  52//52
//42//78//88//108//112//122//142
float balance_angle=-29.8;//29.2//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//26.2~26.7

//�ٶȻ�ͨ��PI����
#define speed_P  3.2
#define speed_I  0.004
#define speed_D  0
#define speed_destination 120//80//��ģ�����ٶ� 
#define speed_curve_destination 60//ת��ʱ��ģ�������ٶ�
#define speed_output_limit 200
#define speed_up_limit 10
#define speed_integral_limit 500//��������

float direction_P=16;//����P����
float direction_I=0;
float direction_D=-1.2;//����D���� 

int direction_PWM_limit=60;
#endif

//����4
#if parameter_mode==4
//ֱ��PID����
//�ǶȻ�ͨ��PD��������
#define static_angle_D  -34//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  52//52
//42//78//88//108//112//122//142
float balance_angle=-29.8;//29.2//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//26.2~26.7

//�ٶȻ�ͨ��PI����
#define speed_P  2.8
#define speed_I  0.004
#define speed_D  0
#define speed_destination 60//80//��ģ�����ٶ� 
#define speed_curve_destination 60//��ģת��������ٶ�
#define speed_output_limit 200
#define speed_up_limit 10
#define speed_integral_limit 500//��������


//����
float direction_P=20;//����P����
float direction_I=0;
float direction_D=-1.8;//����D���� 

#define direction_up_limit 10

int direction_PWM_limit=60;
#endif

//����5
#if parameter_mode==5
//ֱ��PID����
//�ǶȻ�ͨ��PD��������
#define static_angle_D  -31//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  68//52
//42//78//88//108//112//122//142
float balance_angle=-29.8;//29.2//-20.7;//-37.4;//��е���Ƕ� 44.8~47//39//35-38��Լ36//35.4//26.2~26.7

//�ٶȻ�ͨ��PI����
#define speed_P  4.2
#define speed_I  0.04
#define speed_D  0
#define speed_destination 160//80//��ģ�����ٶ� 
#define speed_output_limit 300
#define speed_up_limit 10
#define speed_integral_limit 500//��������


//����
float direction_P=14;//����P����
float direction_I=0;
float direction_D=-1.8;//����D���� 

#define direction_up_limit 0

int direction_PWM_limit=80;
#endif

//float var[4];

uint8 ms_task_flag;//��ͬʱ�������־λ


//2ms����һ�νǶȻ�����
//4ms����һ���ٶȻ�����
//8ms����һ�η��򻷿���
#if 0
//ƽ����̬����
void PIT0_IRQHandler(void)//ÿ���2ms����һ�ζ�ʱ���жϺ���
{
//  int16_t var[3];  
  PIT_FlAG_CLR(pit0);
  float var[6];
  
  ms_task_flag++;
   
  if((ms_task_flag%2)==0)//4ms_task�ٶȻ�
  {     
      //�����������
      left_encoder_val=ftm_quad_get(ftm1);
      right_encoder_val=-ftm_quad_get(ftm2);
      
      ftm_quad_clean(ftm1);
      ftm_quad_clean(ftm2);
 
//�ο�����ѧУ���޷�����      
//��ֹ���ּ���Ӱ��ֱ��������
#if 0
      
      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
      middle_encoder_old_val*=0.7;
      middle_encoder_old_val+=middle_encoder_val*0.3;
      
      speed_integral_val+=middle_encoder_old_val;
        
#endif
//�ο�Э����ǰʦ�ֵ��޷�����      
#if 1
      
      
      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
      
      if(middle_encoder_val-middle_encoder_old_val>speed_output_limit) middle_encoder_old_val+=speed_output_limit;
      else if(middle_encoder_val-middle_encoder_old_val<-speed_output_limit) middle_encoder_old_val-=speed_output_limit;
      else middle_encoder_old_val=middle_encoder_val;
      
      speed_integral_val+=middle_encoder_old_val;
      
#endif
   
      if(middle_encoder_val<80)
      {
        direction_P=20;
        direction_PWM_limit=40;
      }
      else if(middle_encoder_val>=80)
      {
        direction_P=30;
        direction_PWM_limit=50;
      }
      
      //�����޷�
      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
      
      speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
   }

    
   if((ms_task_flag%4)==0)//8ms_task����
   {
     find_middle_line();
     
     Image_Decompression(image_bin,image_dec[0]);
     dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
     
     direction_output_val=(int)(((float)middle_Line-theory_middle_line)*(float)direction_P);
      
     direction_count=0;//
     
     //�����޷�
     if(direction_output_val>direction_PWM_limit)
       direction_output_val=direction_PWM_limit;
     else if(direction_output_val<-direction_PWM_limit)
       direction_output_val=-direction_PWM_limit;
     
     ms_task_flag=0;
   }
  
/**********************
ÿ�������ȡһ��mpu6050������ (2ms����)
***********************/
    //��ȡmpu6050������
    Get_AccData();
    Get_Gyro();
    Get_encoder();
    
//    �������ǵ��ٶȻ���
    mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
    mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;//����
    
    //�������ǵļ��ٶȽ��л���
    mpu_acc_float_y=(float)mpu_acc_y*0.2392615;
    
    //���ü��ٶȼƼ������(�Ƕ�ֵ)
    //���ú��������ת��Ϊ�����������������ת��ϵ
    angle_z_now=atan2(mpu_acc_y,mpu_acc_x)*(float)180/3.1415926;
    angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180/3.1415926;

    kalman_filter(angle_y_now,-mpu_gyro_float_y);
    
    //��λ���������ǲ��γ���    
    var[0]=mpu_gyro_float_y;
    var[1]=angle;
    var[2]=angle_y_now;
    var[3]=angle_dot;
    var[4]=motor_right_val;
    var[5]=motor_left_val;
    
    vcan_sendware((void*)var,sizeof(var));
    
    //���ҵ��PWM����
     motor_right_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()+direction_smoothness_output();
     motor_left_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()-direction_smoothness_output();
     
//     var[1]=motor_left_val;
     
//    vcan_sendware((void*)var,sizeof(var));
      
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
    
    if(angle>=angle_down_limit||angle<=angle_up_limit)//angle<=-75.7||
    {    
      motor_right_val=0;
      motor_left_val=0;
    }
    
    motor_control(motor_right_val,motor_left_val);
}
#endif

//2ms����һ���ж�
//�ǶȻ�6ms�ٶȻ�12ms����8ms
//4ms����һ�ε������
#if 1
uint8 ms_task;
float angle_y_add,mpu_gyro_z_add;//mpu6050��ֵ�˲�����

//mpu6050��ر���
float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050�����ĽǶȣ�δ�˲���
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern float angle,angle_dot,angle1,angle1_dot;
//mpu6050��ֵ�˲�����

//float var[4];//��λ������

void PIT0_IRQHandler(void)
{
  PIT_FlAG_CLR(pit0);
  
  ms_task++;
  
  //2ms��ȡһ��mpu6050���о�ֵ�˲�
  //��ȡmpu6050������
  Get_AccData();
  Get_Gyro();
  Get_encoder();
  
  //    �������ǵ��ٶȻ���
  mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
  mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;
//  mpu_gyro_float_x=(float)mpu_gyro_x*0.0610361;//����
  mpu_gyro_z_add+=mpu_gyro_float_z;
  
  //���ü��ٶȼƼ������(�Ƕ�ֵ)
  //���ú��������ת��Ϊ�����������������ת��ϵ
  angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180/3.1415926;
  
  angle_y_add+=angle_y_now;
  
  motor_right_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()+direction_smoothness_output();
  motor_left_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()-direction_smoothness_output();
    
  //��ֹС��δ��������ʱ����
  if(angle>=angle_down_limit||angle<=angle_up_limit)//angle<=-75.7||
  {    
    motor_right_val=0;
    motor_left_val=0;
  }
  motor_control(motor_right_val,motor_left_val);
  if((ms_task%3)==0)//mpu6050�˲�
  {
    angle_y_add/=(float)3;//��ֵ�˲�
    kalman_filter(angle_y_add,mpu_gyro_float_y);
    
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
    
         //�ٶȻ�
#if 0 
      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
      
      if(middle_encoder_val-middle_encoder_old_val>speed_output_limit) middle_encoder_old_val+=speed_output_limit;
      else if(middle_encoder_val-middle_encoder_old_val<-speed_output_limit) middle_encoder_old_val-=speed_output_limit;
      else middle_encoder_old_val=middle_encoder_val;
      
      speed_integral_val+=middle_encoder_old_val;
      
      //�����޷�
      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
      
      speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
      
#endif

//    //��λ���������ǲ��γ���    
//    var[0]=mpu_gyro_float_y;
//    var[1]=angle;
//    var[2]=angle_y_add;
//    var[3]=angle_dot;
//    
//    vcan_sendware((void*)var,sizeof(var));
  
    angle_y_add=0;
  }
  if(ms_task%4==0)
  {
     find_middle_line();
     
     Image_Decompression(image_bin,image_dec[0]);
     dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
     mpu_gyro_z_add/=(float)4;
     
     direction_output_val=(int)(((float)middle_Line-theory_middle_line)*(float)direction_P+(float)direction_D*(float)mpu_gyro_z_add+direction_I);
   
//�̶�ֵ���Ʒ���PWM����
#if 0
     if((direction_output_val-direction_output_old_val)>direction_up_limit) direction_output_old_val+=direction_up_limit;
     if((direction_output_val-direction_output_old_val)>direction_up_limit) 
     
#endif 
     
//�������Ʒ���PWM���� (ʹת��Բ��)    
#if 1     
    direction_output_old_val=(int)((float)direction_output_old_val*0.6+(float)direction_output_val*0.4);
#endif 
    
    
     //�����޷�
     if(direction_output_val>direction_PWM_limit)
       direction_output_val=direction_PWM_limit;
     else if(direction_output_val<-direction_PWM_limit)
       direction_output_val=-direction_PWM_limit;
     
     mpu_gyro_z_add=0;
  }
  if((ms_task%6)==0)
  {
       //�ٶȻ�
#if 1 
      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
      
      if(middle_encoder_val-middle_encoder_old_val>speed_output_limit) middle_encoder_old_val+=speed_output_limit;
      else if(middle_encoder_val-middle_encoder_old_val<-speed_output_limit) middle_encoder_old_val-=speed_output_limit;
      else middle_encoder_old_val=middle_encoder_val;
      
      speed_integral_val+=middle_encoder_old_val;
      
      
      //�����޷�
      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
      
      if(L_curve_flag||R_curve_flag) speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_curve_destination)+speed_I*speed_integral_val);
      else speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
#endif  
  }
  if(ms_task>=12)
  {
      ms_task=0;
  }
}

#endif



//�ٶȻ�ƽ���������
int speed_smoothness_output()
{
  if(speed_count>=3)
    speed_count=0;
  speed_count++;
  
  speed_smoothness_output_old_val=speed_smoothness_output_val;
  
  speed_smoothness_output_val=speed_smoothness_output_old_val+(int)((float)(speed_output_val-speed_smoothness_output_old_val)*(float)speed_count/3.0);//�ٶȻ�����Ϊ4ms//ÿ2�������һ�ε������
  
  return speed_smoothness_output_val;
}


//����ƽ���������
int direction_smoothness_output()
{
  if(direction_count>=4)
    direction_count=0;
  direction_count++;

  
  direction_smoothness_output_old_val=direction_smoothness_output_val;
  
  direction_smoothness_output_val=direction_smoothness_output_old_val+(int)((float)(direction_output_val-direction_smoothness_output_old_val)*(float)direction_count/4.0);//��������Ϊ12ms//ÿ2�������һ�ε������
  
  return direction_smoothness_output_val;
}

void Get_encoder()
{
  //�����������
  left_encoder_val=ftm_quad_get(ftm1);
  right_encoder_val=-ftm_quad_get(ftm2);
      
  ftm_quad_clean(ftm1);
  ftm_quad_clean(ftm2);
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
      ftm_pwm_duty(ftm3,ftm_ch0,-motor_right_val-right_deal_val);
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
      ftm_pwm_duty(ftm3,ftm_ch2,-motor_left_val-left_deal_val);
      ftm_pwm_duty(ftm3,ftm_ch3,0);
    }  
}





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
