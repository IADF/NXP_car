#include "control.h"

#define parameter_mode 3 //0为测试1为参数//参数3开始为控制周期6ms的

//中线程序相关变量
extern uint8 L_curve_flag;//右弯标志位
extern uint8 R_curve_flag;//左弯标志位

//mpu6050相关变量
uint8 PIT_flag;
float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050读出的角度（未滤波）
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern float angle,angle_dot,angle1,angle1_dot;

//串口输出变量
uint8 str[20];

//中线采集程序相关变量
extern float middle_Line_temp;
extern int middle_Line;
float theory_middle_line=39.5;//理论中线

//方向环相关变量
extern int direction_output_val,direction_output_old_val,direction_smoothness_output_val,direction_smoothness_output_old_val;
extern int direction_count;//平滑输出变量
int speed_err;

//速度环相关变量
int left_encoder_val,right_encoder_val,middle_encoder_val; //编码器数值
float   left_speed_val,right_speed_val; //通过编码器换算成左右车轮的线速度
float   middle_speed_val,middle_encoder_old_val;//车模速度，通过左右车轮的线速度均值得到
int speed_output_val,speed_output_old_val,speed_smoothness_output_val,speed_smoothness_output_old_val;
int speed_integral_val,speed_last_err;
int speed_count;

//电机PWM给定变量
int motor_right_val,motor_left_val;
//限制电机转速电压,PWM信号限制20%
//直立角度控制+15°~-15°
//电池架接触地面时角度为52.3
//车盘接触地面时角度为0.52
//车胎直径为62mm
//车轮转一圈512线编码器示数约为1157
//#define static_up_angle_P  48.5//82.5

//上下限制直立角度
#define angle_down_limit  -1//-0.52
#define angle_up_limit    -52   //-52.3

//PWM限制//转速限制
//转速过快容易导致主控电压不足
#define PWM_limit   700 //给1000时为100%PWM输出，当电机全速输出时电机分压过大容易导致主控电压不足

#if  parameter_mode==0
//直立PID参数
//角度环通过PD参数整定
#define static_angle_D  0//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  52//42//78//88//108//112//122//142
float balance_angle=-31.9;//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4

//速度环通过PI控制
#define speed_P  0
#define speed_I  0.0
#define speed_D  0
#define speed_destination 0//车模期望速度
#define speed_output_limit 0
#define speed_up_limit 0
#define speed_integral_limit 0//积分上限

float direction_P=0;//20.0;//方向环P参数
float direction_D=0;//方向环D参数 

int direction_PWM_limit;

#endif

//直立环的控制周期为2ms的参数
////参数1
//#if  parameter_mode==1
////直立PID参数
////角度环通过PD参数整定
//#define static_angle_D  -18.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  42//42//78//88//108//112//122//142
//float balance_angle=-31.8;//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4
//
////速度环通过PI控制
//#define speed_P  0.4
//#define speed_I  0.002
//#define speed_D  0
//#define speed_destination 80//车模期望速度 
//#define speed_output_limit 30
//#define speed_up_limit 5
//#define speed_integral_limit 400//积分上限
//
//float direction_P=20.0;//方向环P参数
//float direction_D=0;//方向环D参数 
//
//int direction_PWM_limit;
//#endif
//
////参数2
//#if parameter_mode==2
////直立PID参数
////角度环通过PD参数整定
//#define static_angle_D  -17.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  42//42//78//88//108//112//122//142
//float balance_angle=-31.8;//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4
//
////速度环通过PI控制
//#define speed_P  0.4
//#define speed_I  0
//#define speed_D  0
//#define speed_destination 100//车模期望速度 
//#define speed_output_limit 100
//#define speed_up_limit 5
//#define speed_integral_limit 50//积分上限
//
//float direction_P=20.0;//方向环P参数
//float direction_D=0;//方向环D参数 
//
//int direction_PWM_limit;
//#endif

//比较稳但是速度很慢，
#if parameter_mode==3
//直立PID参数
//角度环通过PD参数整定
#define static_angle_D  -34//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  52//52
//42//78//88//108//112//122//142
float balance_angle=-29.8;//29.2//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//26.2~26.7

//速度环通过PI控制
#define speed_P  3.2
#define speed_I  0.004
#define speed_D  0
#define speed_destination 120//80//车模期望速度 
#define speed_curve_destination 60//转向时车模的期望速度
#define speed_output_limit 200
#define speed_up_limit 10
#define speed_integral_limit 500//积分上限

float direction_P=16;//方向环P参数
float direction_I=0;
float direction_D=-1.2;//方向环D参数 

int direction_PWM_limit=60;
#endif

//参数4
#if parameter_mode==4
//直立PID参数
//角度环通过PD参数整定
#define static_angle_D  -34//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  52//52
//42//78//88//108//112//122//142
float balance_angle=-29.8;//29.2//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//26.2~26.7

//速度环通过PI控制
#define speed_P  2.8
#define speed_I  0.004
#define speed_D  0
#define speed_destination 60//80//车模期望速度 
#define speed_curve_destination 60//车模转弯的期望速度
#define speed_output_limit 200
#define speed_up_limit 10
#define speed_integral_limit 500//积分上限


//方向环
float direction_P=20;//方向环P参数
float direction_I=0;
float direction_D=-1.8;//方向环D参数 

#define direction_up_limit 10

int direction_PWM_limit=60;
#endif

//参数5
#if parameter_mode==5
//直立PID参数
//角度环通过PD参数整定
#define static_angle_D  -31//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  68//52
//42//78//88//108//112//122//142
float balance_angle=-29.8;//29.2//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//26.2~26.7

//速度环通过PI控制
#define speed_P  4.2
#define speed_I  0.04
#define speed_D  0
#define speed_destination 160//80//车模期望速度 
#define speed_output_limit 300
#define speed_up_limit 10
#define speed_integral_limit 500//积分上限


//方向环
float direction_P=14;//方向环P参数
float direction_I=0;
float direction_D=-1.8;//方向环D参数 

#define direction_up_limit 0

int direction_PWM_limit=80;
#endif

//float var[4];

uint8 ms_task_flag;//不同时刻任务标志位


//2ms进行一次角度环控制
//4ms进行一次速度环控制
//8ms进行一次方向环控制
#if 0
//平衡姿态控制
void PIT0_IRQHandler(void)//每间隔2ms进入一次定时器中断函数
{
//  int16_t var[3];  
  PIT_FlAG_CLR(pit0);
  float var[6];
  
  ms_task_flag++;
   
  if((ms_task_flag%2)==0)//4ms_task速度环
  {     
      //正交解码程序
      left_encoder_val=ftm_quad_get(ftm1);
      right_encoder_val=-ftm_quad_get(ftm2);
      
      ftm_quad_clean(ftm1);
      ftm_quad_clean(ftm2);
 
//参考其他学校的限幅程序      
//防止过分加速影响直立环程序
#if 0
      
      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
      middle_encoder_old_val*=0.7;
      middle_encoder_old_val+=middle_encoder_val*0.3;
      
      speed_integral_val+=middle_encoder_old_val;
        
#endif
//参考协会以前师兄的限幅程序      
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
      
      //积分限幅
      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
      
      speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
   }

    
   if((ms_task_flag%4)==0)//8ms_task方向环
   {
     find_middle_line();
     
     Image_Decompression(image_bin,image_dec[0]);
     dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
     
     direction_output_val=(int)(((float)middle_Line-theory_middle_line)*(float)direction_P);
      
     direction_count=0;//
     
     //方向环限幅
     if(direction_output_val>direction_PWM_limit)
       direction_output_val=direction_PWM_limit;
     else if(direction_output_val<-direction_PWM_limit)
       direction_output_val=-direction_PWM_limit;
     
     ms_task_flag=0;
   }
  
/**********************
每两毫秒获取一次mpu6050的数据 (2ms任务)
***********************/
    //获取mpu6050的数据
    Get_AccData();
    Get_Gyro();
    Get_encoder();
    
//    将陀螺仪的速度换算
    mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
    mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;//方向环
    
    //将陀螺仪的加速度进行换算
    mpu_acc_float_y=(float)mpu_acc_y*0.2392615;
    
    //利用加速度计计算倾角(角度值)
    //利用函数将倾角转换为符合右手螺旋定则的转动系
    angle_z_now=atan2(mpu_acc_y,mpu_acc_x)*(float)180/3.1415926;
    angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180/3.1415926;

    kalman_filter(angle_y_now,-mpu_gyro_float_y);
    
    //上位机看陀螺仪波形程序    
    var[0]=mpu_gyro_float_y;
    var[1]=angle;
    var[2]=angle_y_now;
    var[3]=angle_dot;
    var[4]=motor_right_val;
    var[5]=motor_left_val;
    
    vcan_sendware((void*)var,sizeof(var));
    
    //左右电机PWM计算
     motor_right_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()+direction_smoothness_output();
     motor_left_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()-direction_smoothness_output();
     
//     var[1]=motor_left_val;
     
//    vcan_sendware((void*)var,sizeof(var));
      
    //机械零点LED显示函数判断函数
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

//2ms触发一次中断
//角度环6ms速度环12ms方向环8ms
//4ms进行一次电机控制
#if 1
uint8 ms_task;
float angle_y_add,mpu_gyro_z_add;//mpu6050均值滤波程序

//mpu6050相关变量
float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050读出的角度（未滤波）
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern float angle,angle_dot,angle1,angle1_dot;
//mpu6050均值滤波变量

//float var[4];//上位机变量

void PIT0_IRQHandler(void)
{
  PIT_FlAG_CLR(pit0);
  
  ms_task++;
  
  //2ms获取一次mpu6050进行均值滤波
  //获取mpu6050的数据
  Get_AccData();
  Get_Gyro();
  Get_encoder();
  
  //    将陀螺仪的速度换算
  mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
  mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;
//  mpu_gyro_float_x=(float)mpu_gyro_x*0.0610361;//方向环
  mpu_gyro_z_add+=mpu_gyro_float_z;
  
  //利用加速度计计算倾角(角度值)
  //利用函数将倾角转换为符合右手螺旋定则的转动系
  angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180/3.1415926;
  
  angle_y_add+=angle_y_now;
  
  motor_right_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()+direction_smoothness_output();
  motor_left_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()-direction_smoothness_output();
    
  //防止小车未正常自立时疯跑
  if(angle>=angle_down_limit||angle<=angle_up_limit)//angle<=-75.7||
  {    
    motor_right_val=0;
    motor_left_val=0;
  }
  motor_control(motor_right_val,motor_left_val);
  if((ms_task%3)==0)//mpu6050滤波
  {
    angle_y_add/=(float)3;//均值滤波
    kalman_filter(angle_y_add,mpu_gyro_float_y);
    
    //机械零点LED显示函数判断函数
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
    
         //速度环
#if 0 
      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
      
      if(middle_encoder_val-middle_encoder_old_val>speed_output_limit) middle_encoder_old_val+=speed_output_limit;
      else if(middle_encoder_val-middle_encoder_old_val<-speed_output_limit) middle_encoder_old_val-=speed_output_limit;
      else middle_encoder_old_val=middle_encoder_val;
      
      speed_integral_val+=middle_encoder_old_val;
      
      //积分限幅
      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
      
      speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
      
#endif

//    //上位机看陀螺仪波形程序    
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
   
//固定值限制方向环PWM增长
#if 0
     if((direction_output_val-direction_output_old_val)>direction_up_limit) direction_output_old_val+=direction_up_limit;
     if((direction_output_val-direction_output_old_val)>direction_up_limit) 
     
#endif 
     
//比例限制方向PWM增长 (使转向圆滑)    
#if 1     
    direction_output_old_val=(int)((float)direction_output_old_val*0.6+(float)direction_output_val*0.4);
#endif 
    
    
     //方向环限幅
     if(direction_output_val>direction_PWM_limit)
       direction_output_val=direction_PWM_limit;
     else if(direction_output_val<-direction_PWM_limit)
       direction_output_val=-direction_PWM_limit;
     
     mpu_gyro_z_add=0;
  }
  if((ms_task%6)==0)
  {
       //速度环
#if 1 
      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
      
      if(middle_encoder_val-middle_encoder_old_val>speed_output_limit) middle_encoder_old_val+=speed_output_limit;
      else if(middle_encoder_val-middle_encoder_old_val<-speed_output_limit) middle_encoder_old_val-=speed_output_limit;
      else middle_encoder_old_val=middle_encoder_val;
      
      speed_integral_val+=middle_encoder_old_val;
      
      
      //积分限幅
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



//速度环平滑输出程序
int speed_smoothness_output()
{
  if(speed_count>=3)
    speed_count=0;
  speed_count++;
  
  speed_smoothness_output_old_val=speed_smoothness_output_val;
  
  speed_smoothness_output_val=speed_smoothness_output_old_val+(int)((float)(speed_output_val-speed_smoothness_output_old_val)*(float)speed_count/3.0);//速度环周期为4ms//每2毫秒进行一次电机控制
  
  return speed_smoothness_output_val;
}


//方向环平滑输出程序
int direction_smoothness_output()
{
  if(direction_count>=4)
    direction_count=0;
  direction_count++;

  
  direction_smoothness_output_old_val=direction_smoothness_output_val;
  
  direction_smoothness_output_val=direction_smoothness_output_old_val+(int)((float)(direction_output_val-direction_smoothness_output_old_val)*(float)direction_count/4.0);//方向环周期为12ms//每2毫秒进行一次电机控制
  
  return direction_smoothness_output_val;
}

void Get_encoder()
{
  //正交解码程序
  left_encoder_val=ftm_quad_get(ftm1);
  right_encoder_val=-ftm_quad_get(ftm2);
      
  ftm_quad_clean(ftm1);
  ftm_quad_clean(ftm2);
}

//山外解压函数
//压缩二值化图像解压（空间 换 时间 解压）
//srclen 是二值化图像的占用空间大小
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {255, 0}; //0 和 1 分别对应的颜色
    //注：山外的摄像头 0 表示 白色，1表示 黑色
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

//电机控制函数
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





//转速换算为线速度程序//by noir
//  left_speed_val=(float)left_encoder_val*0.038042722890625;//按照512线的编码器计算
//  right_speed_val=(float)right_encoder_val*0.038042722890625;//
  
//  middle_speed_val=(left_speed_val+right_speed_val)/2.0;

//增量式编码器
//      int16_t var[3];
//      //需要测试，未进行极性测试by noir
//      //增量式编码器
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


//速度环程序    
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
//      speed_count=0;//速度环平滑参数数清零
//       
//     
//      //速度环限幅
//      if(speed_output_val>speed_output_limit)
//        speed_output_val=speed_output_limit;
//      else if(speed_output_val<-speed_output_limit)
//        speed_output_val=-speed_output_limit;
////      var[0]=middle_encoder_val;
////      var[1]=middle_encoder_integral_val;
////      var[2]=speed_output_val;
////      
////      vcan_sendware((void*)var,sizeof(var)); 
      
//参考的PID程序
// 位置式动态PID控制
//int32 PlacePID_Control(PID *sprt, float *PID, float NowPiont, float SetPoint)
//{
//	//定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
//	float iError;	//当前误差
//	int  Actual;	//最后得出的实际输出值
//	float Kp;		//动态P
//	
//	iError = SetPoint - NowPiont;	//计算当前误差
//	sprt->SumError += iError*0.01;
//	if (sprt->SumError >= PID[KT])
//	{
//		sprt->SumError = PID[KT];
//	}
//	else if (sprt->SumError <= PID[KT])
//	{
//		sprt->SumError = -PID[KT];
//	}
//	//二次函数是为了达到 误差越大  反应越快 回复力越大 其中 KI值是误差为0时的P 也就是直道上的P值
//	Kp = 1.0 * (iError*iError) / PID[KP] + PID[KI];	//P值与差值成二次函数关系，此处P和I不是PID参数，而是动态PID参数，要注意！！！
//	
//	Actual = Kp * iError
//		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//不完全微分  
//	sprt->LastError = iError;		//更新上次误差
//
//	Actual = range_protect(Actual, -260, 260);
//
//	return Actual;
//}
//
//// 位置式PID控制
//int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
//{
//	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
//	int32 Realize;	// 最后得出的实际增量
//
//	sptr->Dis_Err = Point - NowData;	// 计算当前误差
//	sptr->SumError += PID[KI] * sptr->Dis_Err;	// 误差积分
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
////			+ PID[KB] * ( NowData- sptr->LastData); //微分先行
//	
//	sptr->PrevError = sptr->LastError;	// 更新前次误差
//	sptr->LastError = sptr->Dis_Err;		  	// 更新上次误差
//	sptr->LastData  = NowData;			// 更新上次数据
//
//	return Realize;	// 返回实际值
//}
//
//// 增量式PID电机控制
//int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point)
//{
//	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
//	int32 iError,	//当前误差
//		Increase;	//最后得出的实际增量
//
//	iError = Point - NowData;	// 计算当前误差
//
//	Increase =  PID[KP] * (iError - sptr->LastError)
//			  + PID[KI] * iError
//			  + PID[KD] * (iError - 2 * sptr->LastError + sptr->PrevError);
//	
//	sptr->PrevError = sptr->LastError;	// 更新前次误差
//	sptr->LastError = iError;		  	// 更新上次误差
//	sptr->LastData  = NowData;			// 更新上次数据
//	
//	return Increase;	// 返回增量
//}
