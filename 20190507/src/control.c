#include "control.h"
#include "headfile.h"


#define parameter_mode 8 //0为测试1为参数//参数3，4开始为控制周期6ms的//参数5四元数解算参数

//去除零飘后机械零点会改变

//中线程序相关变量
extern uint8 L_curve_flag;//右弯标志位
extern uint8 R_curve_flag;//左弯标志位

//四元数解算相关变量
extern _sensor_st sensor;
extern _imu_st imu_data;

//mpu6050相关变量
uint8 PIT_flag;
float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x=0,mpu_gyro_float_y=0,mpu_gyro_float_z=0,mpu_acc_float_x=0,mpu_acc_float_y=0,mpu_acc_float_z=0;//mpu6050读出的角度（未滤波）
float last_mpu_gyro_float_x,last_mpu_gyro_float_y,last_mpu_gyro_float_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern float angle,angle_dot,angle1,angle1_dot;
extern int mpu_gyro_x_offset,mpu_gyro_y_offset,mpu_gyro_z_offset;//零飘
extern int mpu_acc_x_offset,mpu_acc_y_offset,mpu_acc_z_offset;

//串口输出变量
uint8 str[20];

//角速度环
float mpu_gyro_integral_val;

//中线采集程序相关变量
extern float middle_Line_temp;
extern int middle_Line;
float theory_middle_line=39.5;//理论中线
uint8 lose_line_flag;

//方向环相关变量
int middle_line_output_val,middle_line_output_old_val,middle_line_smoothness_output_val,middle_line_smoothness_output_old_val,direction_output_val,direction_output_last_val;
int direction_count;//平滑输出变量
int speed_err;

//速度环相关变量
int left_encoder_val,right_encoder_val,left_encoder_old_val,right_encoder_old_val; //编码器数值
float   left_speed_val,right_speed_val; //通过编码器换算成左右车轮的线速度
float   middle_speed_val,middle_encoder_old_val,middle_encoder_val;//车模速度，通过左右车轮的线速度均值得到
int speed_output_old_val,speed_smoothness_output_val,speed_smoothness_output_old_val;
//int speed_output_val;
int speed_integral_val,speed_last_err;
int speed_count;

//电机疯转标志位
u8 left_motor_crazy_flag,right_motor_crazy_flag;
u8 run_flag;//运行标志位（0表示未疯转，1表示疯转,2冲出赛道停止电机转动）//当疯转时停止速度环运行，待速度环稳定后

//电机PWM给定变量
int motor_right_val,motor_left_val,motor_old_right_val,motor_old_left_val;
//限制电机转速电压,PWM信号限制20%
//直立角度控制+15°~-15°
//电池架接触地面时角度为52.3
//车盘接触地面时角度为0.52
//车胎直径为62mm
//车轮转一圈512线编码器示数约为1157
//#define static_up_angle_P  48.5//82.5

//上下限制直立角度
#define angle_down_limit  -1//-0.52
#define angle_up_limit    -53.5   //-52.3

//PWM限制//转速限制
//转速过快容易导致主控电压不足
#define PWM_limit   800 //给1000时为100%PWM输出，当电机全速输出时电机分压过大容易导致主控电压不足

#if  parameter_mode==8//速度環純P控制//角度环与角速度环不够硬转弯导致车体不稳定
//直立PID参数
//角速度环PD参数整定
#define static_angle_speed_P   3.8//6.8//34//58//102//64
#define static_angle_speed_I   0.0
#define static_angle_speed_D   0.24//-0.84//-12
#define mpu_gyro_integral_limit 1100//积分限幅参数

//角度环通过PD参数整定
#define static_angle_D  0.0008//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  0.104//0.064//0.098//0.143//-48//-12//42//78//88//108//112//122//142
float balance_angle=-30.4;//34.8;//34.4//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//33.4平衡角度//前进时角度增加
#define angle_output_limit 1100

//速度环通过PI控制//未加加速限幅加速不稳定，容易过冲//纯P控制（容易过冲震荡）（不够稳定）(速度晃动较大)
#define speed_P  1.8//-0.98//171//4.8//32.4//-48.6
#define speed_I  -0.018
#define speed_D  0//76
#define speed_destination 0//车模期望速度-为电池方向
#define speed_curve_destination 0//车模转弯的期望速度
#define speed_output_limit 1100
#define speed_up_limit 50
#define speed_integral_limit 1000//积分上限
int speed_val_I=180;
int speed_val_P=180;
#define speed_target_val 180//目标速度


float middle_line_P=18;//20.0;//方向环P参数
float middle_line_I=0;
float middle_line_D=0;//方向环D参数 

int middle_line_PWM_limit=900;
int direction_output_limit=80;//内环限幅参数

//方向环内环
float direction_P=1.4;//0.8;
float direction_I=0;
float direction_D=0;

#endif

#if  parameter_mode==10//速度环使用PI控制
//直立PID参数
//角速度环PD参数整定
#define static_angle_speed_P   3.9//3.8//6.8//34//58//102//64
#define static_angle_speed_I   -0.0
#define static_angle_speed_D  -0.0//-12
#define mpu_gyro_integral_limit 1100//积分限幅参数

//角度环通过PD参数整定
#define static_angle_D  -0.0//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
#define left_deal_val   0  
#define right_deal_val  0
//#define static_down_angle_P  42
#define static_angle_P  0.0//0.064//0.098//0.143//-48//-12//42//78//88//108//112//122//142
float balance_angle=-33.8;//34.8;//34.4//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//33.4平衡角度//前进时角度增加
#define angle_output_limit 1100

//速度环通过PI控制
#define speed_P  -0//171//4.8//32.4//-48.6
#define speed_I  -0
#define speed_D  -0//76
#define speed_destination 0//车模期望速度-为电池方向
#define speed_curve_destination 0//车模转弯的期望速度
#define speed_output_limit 1100
#define speed_up_limit 1000
#define speed_integral_limit 1000//积分上限
int speed_val_I=0;
int speed_val_P=0;
#define speed_target_val 0//目标速度


float middle_line_P=0;//20.0;//方向环P参数
float middle_line_I=0;
float middle_line_D=0;//方向环D参数 

int middle_line_PWM_limit=900;
int direction_output_limit=80;//内环限幅参数

//方向环内环
float direction_P=0.0;//0.8;
float direction_I=0;
float direction_D=0;

#endif


float var[2];

int angle_speed_output_val;
//int angle_output_val;
float angle_last;

uint8 ms_task;
float angle_y_add,mpu_gyro_z_add;//mpu6050均值滤波程序

//mpu6050相关变量
extern float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050读出的角度（未滤波）
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern float angle,angle_dot,angle1,angle1_dot;
//mpu6050均值滤波变量

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//将速度环串联进角度环
//将角度环串联进角速度环
#if 1//串级PID程序 by noir
#define speed_out_limit 0

uint8 ms_task;
float angle_output_val,speed_output_val;
int motor_val;
float angle_speed_last_val;

void PIT0_IRQHandler(void)//2ms角度环
{
    PIT_FlAG_CLR(pit0);
  
    ms_task++;
  
    //2ms获取一次mpu6050进行均值滤波
    //获取mpu6050的数据
    Get_AccData();
    Get_Gyro();
    Get_encoder();
    
    mpu_normalization_point();//归一化
    speed_measure();//编码器数值处理
    
    if((ms_task%5)==0)//10ms速度环
    {                
        middle_speed_val=middle_encoder_val-speed_val_P;
        
        speed_integral_val+=middle_speed_val;//积分
        if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;//积分限幅
        else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
        
        speed_output_val=speed_P*(middle_speed_val)+speed_I*(speed_integral_val-speed_val_I)+speed_D*(middle_speed_val-middle_encoder_old_val);//PID
      
        middle_encoder_old_val=middle_speed_val;
        
//        //速度环加速限幅
//        if(speed_output_val-speed_output_old_val>speed_up_limit) speed_output_val+=speed_up_limit;
//        else if(speed_output_val-speed_output_old_val<-speed_up_limit) speed_output_val-=speed_up_limit;
//          
//        speed_output_old_val=speed_output_val;
        
        if(speed_output_val>speed_output_limit) speed_output_val=speed_output_limit;//速度环输出限幅
        else if(speed_output_val<-speed_output_limit) speed_output_val=-speed_output_limit;
      
    }
    if((ms_task%3)==0)//6ms角度环
    { 
       
        sensor.Acc_mmss.x=mpu_acc_float_x;
        sensor.Acc_mmss.y=mpu_acc_float_y;
        sensor.Acc_mmss.z=mpu_acc_float_z;

        sensor.Gyro_deg.x=mpu_gyro_float_x;
        sensor.Gyro_deg.y=mpu_gyro_float_y;
        sensor.Gyro_deg.z=mpu_gyro_float_z;
       
      
       IMU_update(1,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data);//四元数解算
//      利用函数将倾角转换为符合右手螺旋定则的转动系
//       angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180.0/3.1415926;//加速度通过三角函数解算
       kalman_filter(imu_data.pit,mpu_gyro_float_y);//imu_data.pit
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
       
       if((middle_Line-theory_middle_line)>6||(middle_Line-theory_middle_line)<-6)//急弯检测（改变机械零点）转向时会导致车模后仰（通过改变机械零点抑制后仰）（同时抑制转弯减速）//R_curve_flag||L_curve_flag
       {
         balance_angle=-31.8;
       }
       else 
       {
         balance_angle=-30.4;
       }
       
       if(run_flag)
       {
          LED_Ctrl(LED2,ON);
          angle_output_val=static_angle_P*(100*(angle-balance_angle))+static_angle_D*((100*(angle-balance_angle))-angle_last);
          angle_last=100*(angle-balance_angle);
       }
       else 
       {
          LED_Ctrl(LED2,OFF);
          angle_output_val=static_angle_P*(100*(angle-balance_angle)-speed_output_val)+static_angle_D*((100*(angle-balance_angle)-speed_output_val)-angle_last);
          angle_last=100*(angle-balance_angle)-speed_output_val;
       }
      
       if(angle_output_val>angle_output_limit) angle_output_val=angle_output_limit;
       else if(angle_output_val<-angle_output_limit) angle_output_val=-angle_output_limit;
       
       mpu_acc_float_z=0;
       mpu_acc_float_x=0;
    }
    if((ms_task%2)==0)//4ms中线偏差方向环（外环）
    {
      //转向环
      find_middle_line();
     
      Image_Decompression(image_bin,image_dec[0]);
      dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
     
      middle_line_output_val=(int)(((float)middle_Line-theory_middle_line)*(float)middle_line_P+(float)middle_line_D*mpu_acc_float_z+middle_line_I);
   
//      //比例限制方向PWM增长 (使转向圆滑)        
//      middle_line_output_old_val=(int)((float)middle_line_output_old_val*0.7+(float)middle_line_output_val*0.3);
    
      //方向环(外环)限幅
      if(middle_line_output_val>middle_line_PWM_limit)
         middle_line_output_val=middle_line_PWM_limit;
      else if(middle_line_output_val<-middle_line_PWM_limit)
         middle_line_output_val=-middle_line_PWM_limit;
    }
    //2ms角速度环  
    mpu_gyro_integral_val+=(mpu_gyro_float_y*10-angle_output_val);
    
    //角速度积分限幅
    if(mpu_gyro_integral_val>mpu_gyro_integral_limit) mpu_gyro_integral_val=mpu_gyro_integral_limit;
    else if(mpu_gyro_integral_val<-mpu_gyro_integral_limit) mpu_gyro_integral_val=-mpu_gyro_integral_limit;

    
    motor_val=(int)(static_angle_speed_P*(mpu_gyro_float_y*10-angle_output_val)+static_angle_speed_I*(mpu_gyro_integral_val)+static_angle_speed_D*((mpu_gyro_float_y*10-angle_output_val)-angle_speed_last_val));//PI
    angle_speed_last_val=mpu_gyro_float_y*10-angle_output_val;
    
    //2ms方向角速度环（内环）
    direction_output_val=(int)(direction_P*(mpu_gyro_float_z*10-middle_line_output_val));//串级PID中线量为输入量无需平滑输出
    
    direction_output_last_val=(int)(0.7*direction_output_last_val+0.3*direction_output_val);//转向上升限幅
    
    if(direction_output_last_val>direction_output_limit) direction_output_last_val=direction_output_limit;
    else if(direction_output_last_val<-direction_output_limit) direction_output_last_val=-direction_output_limit;
    
    motor_right_val=motor_val+direction_output_last_val;
    motor_left_val=motor_val-direction_output_last_val;
    
    var[0]=middle_encoder_val;
    var[1]=angle;
    
    vcan_sendware((void*)var,sizeof(var));
    
    //防止小车未正常自立时疯跑
    if(angle>angle_down_limit||angle<angle_up_limit||lose_line_flag)//angle<=-75.7||
    {    
        motor_right_val=0;
        motor_left_val=0;
    }
    motor_control(motor_right_val,motor_left_val);
    
    if(ms_task>=60)//120ms任务
    {
      ms_task=0;//计数归零
    }
}

#endif


////角度环PID输出函数
//void angle_output(void)
//{
//  angle_output_val=(int)(static_angle_P*(angle-balance_angle)+(angle-angle_last)*static_angle_D);
//  
//  angle_last=angle;
//}

////角速度环PID输出函数
//void angle_speed_output(void)
//{
//  angle_speed_output_val=(int)(static_angle_speed_P*(mpu_gyro_float_y-(float)0)+static_angle_speed_D*(mpu_gyro_float_y-last_mpu_gyro_float_y));
//  
//  last_mpu_gyro_float_y=mpu_gyro_float_y;
//}


void Get_encoder()
{
  //正交解码程序
  left_encoder_val=-ftm_quad_get(ftm1);
  right_encoder_val=ftm_quad_get(ftm2);
  
//  middle_encoder_val=(left_encoder_val+right_encoder_val)/2;
  
  ftm_quad_clean(ftm1);
  ftm_quad_clean(ftm2);
}

//对编码器进行处理
//当电机疯转时，将速度环关闭等待车稳定
void speed_measure(void)////未完成
{
  //左电机疯转检测
  if(left_encoder_val>speed_target_val+50)//疯转
  {
    left_motor_crazy_flag=1;
  }
  else if(left_encoder_val<-10)//倒转
  {
    left_motor_crazy_flag=-1;
  }
  else if(left_encoder_val-left_encoder_old_val>30)//过分加速
  {
    left_motor_crazy_flag=2;
  }
  else 
  {
    left_motor_crazy_flag=0;//电机在正常范围运转
  }

  //右电机疯转检测
  if(right_encoder_val>speed_target_val+50)//疯转
  {
    right_motor_crazy_flag=1;
  }
  else if(right_encoder_val<-10)//倒转
  {
    right_motor_crazy_flag=-1;
  }
  else if(right_encoder_val-right_encoder_old_val>30)//过分加速
  {
    right_motor_crazy_flag=2;
  }
  else 
  {
    right_motor_crazy_flag=0;//电机正常范围运转
  }
  
  if(right_motor_crazy_flag)//右电机速度控制
  {
    right_encoder_val=(int)(right_encoder_val*0.9+right_encoder_old_val*0.1);
    right_encoder_old_val=right_encoder_val;
  }
  else
  {
    right_speed_val=(int)(right_encoder_val*0.5+right_encoder_old_val*0.5);
    right_encoder_old_val=right_encoder_val;
  }
  
  if(left_motor_crazy_flag)//左电机速度控制
  {
    left_speed_val=(int)(left_encoder_val*0.9+left_encoder_old_val*0.1);
    left_encoder_old_val=left_encoder_val;
  }
  else 
  {
    left_speed_val=(int)(left_encoder_val*0.5+left_encoder_old_val*0.5);
    left_encoder_old_val=left_encoder_val;
  }
  
  //电机疯转处理
  if(left_motor_crazy_flag>0&&right_motor_crazy_flag>0)//左右同时电机疯转处理(除去倒转)
  {
    middle_encoder_val=middle_encoder_old_val;
    run_flag=1;
  }
  
  if(left_motor_crazy_flag>0)//左电机疯转处理
  {
    if(right_encoder_val>speed_target_val)
    {
      middle_encoder_val=middle_encoder_old_val;
      run_flag=1;
    }
    else 
    {
      middle_encoder_val=right_encoder_val;//左电机疯转以右电机的速度作为车体速度
      run_flag=0;
    }
  }
  
  if(right_motor_crazy_flag>0)
  {
    if(left_encoder_val>speed_target_val)
    {
      middle_encoder_val=middle_encoder_old_val;
      run_flag=1;
    }
    else 
    {
      middle_encoder_val=left_encoder_val;
      run_flag=0;
    }
  }
  
  if(right_motor_crazy_flag==0&&left_motor_crazy_flag==0)//左右电机在正常范围内运转
  {  
    middle_encoder_val=(int)((left_speed_val+right_speed_val)/2);
  
    middle_encoder_val=middle_encoder_val*0.8+middle_encoder_old_val*0.2;
    middle_encoder_old_val=middle_encoder_val;
    run_flag=0;
  }
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

//

void mpu_normalization_point()//归一化
{
#if offset_option==1//offset_option
    mpu_gyro_x-=mpu_gyro_x_offset;
    mpu_gyro_y-=mpu_gyro_y_offset;
    mpu_gyro_z-=mpu_gyro_z_offset;
#endif
  
//    将陀螺仪的速度换算
  mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
  mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;
  mpu_gyro_float_x=(float)mpu_gyro_x*0.0610361;//方向环
  
//  
  mpu_acc_float_x=(float)mpu_acc_x*0.2392615;
  mpu_acc_float_y=(float)mpu_acc_y*0.2392615;
  mpu_acc_float_z=(float)mpu_acc_z*0.2392615;

//  mpu_acc_float_x+=(float)mpu_acc_x*(float)0.2392615/(float)3.0;
//  mpu_acc_float_y+=(float)mpu_acc_y*(float)0.2392615/(float)3.0;
//  mpu_acc_float_z+=(float)mpu_acc_z*(float)0.2392615/(float)3.0;
}


//串级PID中线偏差作为输入量不能使用平滑输出
//方向环平滑输出程序
//int direction_smoothness_output()
//{
//  if(direction_count>=2)
//    direction_count=0;
//  direction_count++;
//
//  
//  middle_line_smoothness_output_old_val=middle_line_smoothness_output_val;
//  
//  middle_line_smoothness_output_val=middle_line_smoothness_output_old_val+(int)((float)(middle_line_output_val-middle_line_smoothness_output_old_val)*(float)direction_count/2.0);//方向环周期为12ms//每2毫秒进行一次电机控制
//  
//  return middle_line_smoothness_output_val;
//}

//串级PID速度环作为输入量不能使用平滑输出
//速度环平滑输出程序
//int speed_smoothness_output()
//{
//  if(speed_count>=5)
//    speed_count=0;
//  speed_count++;
//  
//  speed_smoothness_output_old_val=speed_smoothness_output_val;
//  
//  speed_smoothness_output_val=speed_smoothness_output_old_val+(int)((float)(speed_output_val-speed_smoothness_output_old_val)*(float)speed_count/5.0);//速度环周期为4ms//每2毫秒进行一次电机控制
//  
//  return speed_smoothness_output_val;
//}

//#if  parameter_mode==8//速度環純P控制//角度环与角速度环不够硬转弯导致车体不稳定，
////直立PID参数
////角速度环PD参数整定
//#define static_angle_speed_P   3.6//6.8//34//58//102//64
//#define static_angle_speed_I   0.0
//#define static_angle_speed_D  -0.4//-12
//#define mpu_gyro_integral_limit 10//积分限幅参数
//
////角度环通过PD参数整定
//#define static_angle_D  0.014//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  0.128//0.064//0.098//0.143//-48//-12//42//78//88//108//112//122//142
//float balance_angle=-33.8;//34.8;//34.4//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//33.4平衡角度//前进时角度增加
//#define angle_output_limit 1100
//
////速度环通过PI控制
//#define speed_P  -2.8//171//4.8//32.4//-48.6
//#define speed_I  -3.42
//#define speed_D  -0//76
//#define speed_destination 0//车模期望速度-为电池方向
//#define speed_curve_destination 0//车模转弯的期望速度
//#define speed_output_limit 1100
//#define speed_up_limit 1000
//#define speed_integral_limit 1000//积分上限
//int speed_val_I=70;
//int speed_val_P=70;
//#define speed_target_val 0//目标速度
//
//
//float middle_line_P=20;//20.0;//方向环P参数
//float middle_line_I=0;
//float middle_line_D=0;//方向环D参数 
//
//int middle_line_PWM_limit=900;
//int direction_output_limit=80;//内环限幅参数
//
////方向环内环
//float direction_P=0.8;//0.8;
//float direction_I=0;
//float direction_D=0;
//
//#endif


////2ms触发一次中断
////角度环6ms速度环12ms方向环8ms
////4ms进行一次电机控制
////PID控制直立
////角速度环为主，角度环辅
//#if 0
//
//void PIT0_IRQHandler(void)
//{
//  PIT_FlAG_CLR(pit0);
//  
//  ms_task++;
//  
//  //2ms获取一次mpu6050进行均值滤波
//  //获取mpu6050的数据
//  Get_AccData();
//  Get_Gyro();
////  mpu_acc_x-=mpu_acc_x_offset;
////  mpu_acc_y-=mpu_acc_y_offset;
////  mpu_acc_z-=mpu_acc_z_offset;
//  
//  //    将陀螺仪的速度换算
//  mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
//  mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;
//  mpu_gyro_float_x=(float)mpu_gyro_x*0.0610361;//方向环
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
////  motor_old_right_val=(int)((float)motor_right_val*0.9+motor_old_right_val*0.1);//高通滤波
////  motor_old_left_val=(int)((float)motor_left_val*0.9+motor_old_left_val*0.1);//高通滤波
//  
//  //防止小车未正常自立时疯跑
//  if(angle>=angle_down_limit||angle<=angle_up_limit)//angle<=-75.7||
//  {    
//    motor_right_val=0;
//    motor_left_val=0;
//  }
//  motor_control(motor_right_val,motor_left_val);
//   
//  if((ms_task%3)==0)//mpu6050滤波
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
//    //利用加速度计计算倾角(角度值)
//    //利用函数将倾角转换为符合右手螺旋定则的转动系
//    angle_y_now=atan2(mpu_acc_float_x,mpu_acc_float_z)*(float)180/3.1415926;//加速度通过三角函数解算
//    IMU_update(1,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data);
//    kalman_filter(imu_data.pit,mpu_gyro_float_y);//imu_data.pit
////    sprintf((char*)str,"%f",angle);
////    OLED_P6x8Str(84,16,str);
//    angle_output();//角度环
//    
//     //转向环
//     find_middle_line();
//     
//     Image_Decompression(image_bin,image_dec[0]);
//     dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
//     
//     direction_output_val=(int)(((float)middle_Line-theory_middle_line)*(float)direction_P+(float)direction_D*(float)mpu_gyro_z_add+direction_I);
//   
////比例限制方向PWM增长 (使转向圆滑)        
//    direction_output_old_val=(int)((float)direction_output_old_val*0.8+(float)direction_output_val*0.2);
//    
//     //方向环限幅
//     if(direction_output_val>direction_PWM_limit)
//       direction_output_val=direction_PWM_limit;
//     else if(direction_output_val<-direction_PWM_limit)
//       direction_output_val=-direction_PWM_limit;
//     
//    
//
//
//    //上位机看陀螺仪波形程序    
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
//    //速度环//加速限幅有问题
//#if 1 
//      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
//      
//      OLED_Print_Num1(84,0,middle_encoder_val);
//            
//      speed_integral_val+=middle_encoder_val;
//      
//      //积分限幅//有问题
//      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
//      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
//      
//      //加速限幅
////      if(L_curve_flag||R_curve_flag) 
////      {
////        if(middle_encoder_val-speed_curve_destination>speed_up_limit) middle_encoder_val+=speed_up_limit;
////        else if(middle_encoder_val-speed_curve_destination<speed_up_limit) middle_encoder_val-=speed_up_limit;
////        else middle_encoder_val=speed_curve_destination;
////      }
////      else if(lose_line_flag==1)     //全丢线速度环为0  
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

////直立环
////2ms触发一次中断
////角度环6ms速度环12ms方向环8ms
////4ms进行一次电机控制
//#if 0
//uint8 ms_task;
//float angle_y_add,mpu_gyro_z_add;//mpu6050均值滤波程序
//
////mpu6050相关变量
//float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050读出的角度（未滤波）
//extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
//extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
//extern float angle,angle_dot,angle1,angle1_dot;
////mpu6050均值滤波变量
//
////float var[4];//上位机变量
//
//void PIT0_IRQHandler(void)
//{
//  PIT_FlAG_CLR(pit0);
//  
//  ms_task++;
//  
//  //2ms获取一次mpu6050进行均值滤波
//  //获取mpu6050的数据
//  Get_AccData();
//  Get_Gyro();
//  Get_encoder();
//  
//  //    将陀螺仪的速度换算
//  mpu_gyro_float_y+=(float)mpu_gyro_y*0.0610361/(float)3.0;
//  mpu_gyro_float_z+=(float)mpu_gyro_z*0.0610361/(float)3.0;
//  mpu_gyro_float_x+=(float)mpu_gyro_x*0.0610361/(float)3.0;//方向环
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
////  motor_old_right_val=(int)((float)motor_right_val*0.9+motor_old_right_val*0.1);//高通滤波
////  motor_old_left_val=(int)((float)motor_left_val*0.9+motor_old_left_val*0.1);//高通滤波
//
//  
//  //防止小车未正常自立时疯跑
//  if(angle>=angle_down_limit||angle<=angle_up_limit)//angle<=-75.7||
//  {    
//    motor_right_val=0;
//    motor_left_val=0;
//  }
//  motor_control(motor_right_val,motor_left_val);
//  if((ms_task%3)==0)//mpu6050滤波
//  {
//    sensor.Gyro_deg.x=mpu_gyro_float_x;
//    sensor.Gyro_deg.y=mpu_gyro_float_y;
//    sensor.Gyro_deg.z=mpu_gyro_float_z;
//    
//    sensor.Acc_mmss.x=mpu_acc_float_x;
//    sensor.Acc_mmss.y=mpu_acc_float_y;
//    sensor.Acc_mmss.z=mpu_acc_float_z;
//    
//    //利用加速度计计算倾角(角度值)
//    //利用函数将倾角转换为符合右手螺旋定则的转动系
//    angle_y_now=atan2(mpu_acc_float_x,mpu_acc_float_z)*(float)180/3.1415926;//加速度通过三角函数解算
//    IMU_update(1,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data);
//    kalman_filter(angle_y_now,mpu_gyro_float_y);//imu_data.pit
//    
//    //机械零点LED显示函数判断函数
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
//         //速度环
//#if 0 
//      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
//      
//      if(middle_encoder_val-middle_encoder_old_val>speed_output_limit) middle_encoder_old_val+=speed_output_limit;
//      else if(middle_encoder_val-middle_encoder_old_val<-speed_output_limit) middle_encoder_old_val-=speed_output_limit;
//      else middle_encoder_old_val=middle_encoder_val;
//      
//      speed_integral_val+=middle_encoder_old_val;
//      
//      //积分限幅
//      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
//      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
//      
//      speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
//      
//#endif
//
//    //上位机看陀螺仪波形程序    
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
////固定值限制方向环PWM增长
//#if 0
//     if((direction_output_val-direction_output_old_val)>direction_up_limit) direction_output_old_val+=direction_up_limit;
//     if((direction_output_val-direction_output_old_val)>direction_up_limit) 
//     
//#endif 
//     
////比例限制方向PWM增长 (使转向圆滑)    
//#if 1     
//    direction_output_old_val=(int)((float)direction_output_old_val*0.6+(float)direction_output_val*0.4);
//#endif 
//    
//    
//     //方向环限幅
//     if(direction_output_val>direction_PWM_limit)
//       direction_output_val=direction_PWM_limit;
//     else if(direction_output_val<-direction_PWM_limit)
//       direction_output_val=-direction_PWM_limit;
//     
//     mpu_gyro_z_add=0;
//  }
//  if((ms_task%6)==0)
//  {
//       //速度环
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
//      //积分限幅
//      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
//      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
//      
//      if(L_curve_flag||R_curve_flag) speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_curve_destination)+speed_I*speed_integral_val);
//      else if(lose_line_flag==1)     speed_output_val=(int)(speed_P*(middle_encoder_old_val-0)+speed_I*speed_integral_val);//全丢线速度环为0   
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
////直立PID参数
////角速度环PD参数整定
//#define static_angle_speed_P  46//34//58//102//64
//#define static_angle_speed_D  -0//-12
//
////角度环通过PD参数整定
//#define static_angle_D  0//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   46  
//#define right_deal_val  46
////#define static_down_angle_P  42
//#define static_angle_P  -28//-48//-12//42//78//88//108//112//122//142
//float balance_angle=-34.8;//34.8;//34.4//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4
//
////速度环通过PI控制
//#define speed_P  8.2
//#define speed_I  0
//#define speed_D  0
//#define speed_destination 300//车模期望速度-为电池方向
//#define speed_curve_destination 300//车模转弯的期望速度
//#define speed_output_limit 200
//#define speed_up_limit 10
//#define speed_integral_limit 500//积分上限
//
//float direction_P=-18;//20.0;//方向环P参数
//float direction_I=0;
//float direction_D=0;//方向环D参数 
//
//int direction_PWM_limit=60;
//
//#endif
//
//
////参数5
//#if parameter_mode==5
////直立PID参数
//
////角速度环PD参数整定
//#define static_angle_speed_P  74//68//74//64
//#define static_angle_speed_D  -6//-4//-14
//
////角度环通过PD参数整定
//#define static_angle_D  -3//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  -32//-24//-24//52
////42//78//88//108//112//122//142
//float balance_angle=-32.8;//29.2//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//26.2~26.7
//
////速度环通过PI控制
//#define speed_P  2.4
//#define speed_I  0
//#define speed_D  0
//#define speed_destination -80//80//车模期望速度 
//#define speed_curve_destination -80//车模转弯的期望速度
//#define speed_output_limit 5
//#define speed_up_limit 200
//#define speed_integral_limit 500//积分上限
//
//
////方向环
//float direction_P=-24;//方向环P参数
//float direction_I=0;
//float direction_D=-0;//方向环D参数 
//
//#define direction_up_limit 0
//
//int direction_PWM_limit=120;
//#endif


//以前的参数
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


//直立环为6ms的控制参数
////比较稳但是速度很慢，
//#if parameter_mode==3
////直立PID参数
////角度环通过PD参数整定
//#define static_angle_D  -34//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  52//52
////42//78//88//108//112//122//142
//float balance_angle=-29.8;//29.2//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//26.2~26.7
//
////速度环通过PI控制
//#define speed_P  4.2
//#define speed_I  0.000
//#define speed_D  0
//#define speed_destination 120//80//车模期望速度 
//#define speed_curve_destination 60//转向时车模的期望速度
//#define speed_output_limit 200
//#define speed_up_limit 5
//#define speed_integral_limit 500//积分上限
//
//float direction_P=16;//方向环P参数
//float direction_I=0;
//float direction_D=-1.2;//方向环D参数 
//
//int direction_PWM_limit=60;
//#endif
//
////参数4
//#if parameter_mode==4
////直立PID参数
////角度环通过PD参数整定
//#define static_angle_D  -34//-38//-64//48//-68.0//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  58//52
////42//78//88//108//112//122//142
//float balance_angle=-28.8;//29.2//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//26.2~26.7
//
////速度环通过PI控制
//#define speed_P  1.4
//#define speed_I  0.004
//#define speed_D  0
//#define speed_destination 60//80//车模期望速度 
//#define speed_curve_destination 60//车模转弯的期望速度
//#define speed_output_limit 200
//#define speed_up_limit 10
//#define speed_integral_limit 500//积分上限
//
//
////方向环
//float direction_P=20;//方向环P参数
//float direction_I=0;
//float direction_D=-1.8;//方向环D参数 
//
//#define direction_up_limit 10
//
//int direction_PWM_limit=60;
//#endif


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

////2ms进行一次角度环控制
////4ms进行一次速度环控制
////8ms进行一次方向环控制
//#if 0
////平衡姿态控制
//void PIT0_IRQHandler(void)//每间隔2ms进入一次定时器中断函数
//{
////  int16_t var[3];  
//  PIT_FlAG_CLR(pit0);
//  float var[6];
//  
//  ms_task_flag++;
//   
//  if((ms_task_flag%2)==0)//4ms_task速度环
//  {     
//      //正交解码程序
//      left_encoder_val=ftm_quad_get(ftm1);
//      right_encoder_val=-ftm_quad_get(ftm2);
//      
//      ftm_quad_clean(ftm1);
//      ftm_quad_clean(ftm2);
// 
////参考其他学校的限幅程序      
////防止过分加速影响直立环程序
//#if 0
//      
//      middle_encoder_val=(right_encoder_val+left_encoder_val)/2;
//      middle_encoder_old_val*=0.7;
//      middle_encoder_old_val+=middle_encoder_val*0.3;
//      
//      speed_integral_val+=middle_encoder_old_val;
//        
//#endif
////参考协会以前师兄的限幅程序      
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
//      //积分限幅
//      if(speed_integral_val>speed_integral_limit) speed_integral_val=speed_integral_limit;
//      else if(speed_integral_val<-speed_integral_limit) speed_integral_val=-speed_integral_limit;
//      
//      speed_output_val=(int)(speed_P*(middle_encoder_old_val-speed_destination)+speed_I*speed_integral_val);
//   }
//
//    
//   if((ms_task_flag%4)==0)//8ms_task方向环
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
//     //方向环限幅
//     if(direction_output_val>direction_PWM_limit)
//       direction_output_val=direction_PWM_limit;
//     else if(direction_output_val<-direction_PWM_limit)
//       direction_output_val=-direction_PWM_limit;
//     
//     ms_task_flag=0;
//   }
//  
///**********************
//每两毫秒获取一次mpu6050的数据 (2ms任务)
//***********************/
//    //获取mpu6050的数据
//    Get_AccData();
//    Get_Gyro();
//    Get_encoder();
//    
////    将陀螺仪的速度换算
//    mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
//    mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;//方向环
//    
//    //将陀螺仪的加速度进行换算
//    mpu_acc_float_y=(float)mpu_acc_y*0.2392615;
//    
//    //利用加速度计计算倾角(角度值)
//    //利用函数将倾角转换为符合右手螺旋定则的转动系
//    angle_z_now=atan2(mpu_acc_y,mpu_acc_x)*(float)180/3.1415926;
//    angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180/3.1415926;
//
//    kalman_filter(angle_y_now,mpu_gyro_float_y);
//    
////    //上位机看陀螺仪波形程序    
////    var[0]=mpu_gyro_float_y;
////    var[1]=angle;
////    var[2]=angle_y_now;
////    var[3]=angle_dot;
////    var[4]=motor_right_val;
////    var[5]=motor_left_val;
////    
////    vcan_sendware((void*)var,sizeof(var));
//    
//    //左右电机PWM计算
//     motor_right_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()+direction_smoothness_output();
//     motor_left_val=(int)((angle-balance_angle)*(float)static_angle_P+mpu_gyro_float_y*(float)static_angle_D)-speed_smoothness_output()-direction_smoothness_output();
//     
////     var[1]=motor_left_val;
//     
////    vcan_sendware((void*)var,sizeof(var));
//      
//    //机械零点LED显示函数判断函数
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
////直立PID参数
////角速度环PD参数整定
//#define static_angle_speed_P   4.4//6.8//34//58//102//64
//#define static_angle_speed_I   0.0
//#define static_angle_speed_D  -0//-12
//#define mpu_gyro_integral_limit 0//积分限幅参数
//
////角度环通过PD参数整定
//#define static_angle_D  0//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  0.098//-48//-12//42//78//88//108//112//122//142
//float balance_angle=-34.4;//34.8;//34.4//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//33.4平衡角度//前进时角度增加
//#define angle_output_limit 300
//
////速度环通过PI控制
//#define speed_P  -14
//#define speed_I  0.08
//#define speed_D  0
//#define speed_destination 0//车模期望速度-为电池方向
//#define speed_curve_destination 0//车模转弯的期望速度
//#define speed_output_limit 60
//#define speed_up_limit 1000
//#define speed_integral_limit 0//积分上限
//int speed_val_I=-1200;
//int speed_val_P=-800;
//#define speed_target_val 0//目标速度
//
////
//float middle_line_P=-18;//20.0;//方向环P参数
//float middle_line_I=0;
//float middle_line_D=0;//方向环D参数 
//
//int middle_line_PWM_limit=80;
//int direction_output_limit=160;//内环限幅参数
//
////方向环内环
//float direction_P=0.8;//0.8;
//float direction_I=0;
//float direction_D=0;
//
//#endif

//#if  parameter_mode==9
////直立PID参数
////角速度环PD参数整定
//#define static_angle_speed_P   3.6//6.8//34//58//102//64
//#define static_angle_speed_I   0.0
//#define static_angle_speed_D  -0//-12
//#define mpu_gyro_integral_limit 10//积分限幅参数
//
////角度环通过PD参数整定
//#define static_angle_D  0.006//-8.4//-12.0//-1.0//-0.5//-32.0//-1.0//14.2//14.2//11.2//19.2//9.1//18.4//31.6//-12.0
//#define left_deal_val   0  
//#define right_deal_val  0
////#define static_down_angle_P  42
//#define static_angle_P  0.128//0.098//0.143//-48//-12//42//78//88//108//112//122//142
//float balance_angle=-33.2;//34.8;//34.4//-20.7;//-37.4;//机械零点角度 44.8~47//39//35-38大约36//35.4//33.4平衡角度//前进时角度增加
//#define angle_output_limit 1100
//
////速度环通过PI控制
//#define speed_P  -0.64//16//43.8//4.8//32.4//-48.6
//#define speed_I  0//12.4
//#define speed_D  -0.48//4
//#define speed_destination 0//车模期望速度-为电池方向
//#define speed_curve_destination 0//车模转弯的期望速度
//#define speed_output_limit 1100
//#define speed_up_limit 1000
//#define speed_integral_limit 500//积分上限
//int speed_val_I=10;
//int speed_val_P=80;
//#define speed_target_val 0//目标速度
//
//
//float middle_line_P=20;//20.0;//方向环P参数
//float middle_line_I=0;
//float middle_line_D=0;//方向环D参数 
//
//int middle_line_PWM_limit=900;
//int direction_output_limit=60;//内环限幅参数
//
////方向环内环
//float direction_P=0.8;//0.8;
//float direction_I=0;
//float direction_D=0;
//
//#endif