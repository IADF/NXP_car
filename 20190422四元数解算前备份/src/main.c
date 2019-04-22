/********************************************
逐飞科技 总钻风-摄像头  历程
Designed by Fly Sir
软件版本:V1.1
最后更新:2016年5月3日
相关信息参考下列地址：
淘宝店：https://seekfree.taobao.com/
------------------------------------
软件版本： IAR 7.2 or MDK 5.17
目标核心： MK60DN512VLL10
 ============================================

分辨率是                188*120
摄像头参数设置可以到    SEEKFREE-->h_file-->SEEKFREE_MT9V032.h

总钻风-摄像头测试步骤：
1.下载程序到开发板
2.插上串口线或者USB转TTL
3.接好MT9V032模块接线
4.通电在TFT液晶上即可观看    
排线有问题容易影响摄像头采集
*********************************************/  
#include "headfile.h"

#define debug_mode 0//0正常程序，1mpu6050调试程序，2摄像头调试程序，3四元数解算调试

uint8 img[OV7725_W * OV7725_H];
//KALMAN_STRUCT atti_x,atti_y;//卡尔曼滤波相关参数结构体初始化函数

void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
int middle_encoder_integral_val;

int direction_output_val,direction_output_old_val,direction_smoothness_output_val,direction_smoothness_output_old_val;
int direction_count;//平滑输出变量


#if debug_mode==0//程序
int main(void)
{
    get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
//    DisableInterrupts;   //关闭所有中断
       
//    NVIC_SetPriorityGrouping(6);
    
    EnableInterrupts;
    
    LED_init();
    set_irq_priority(PORTC_IRQn,0);
    set_irq_priority(DMA0_IRQn,1);
    set_irq_priority(PIT0_IRQn,2);
    set_irq_priority(SysTick_IRQn,3);
    
    ov7725_init();//初始换摄像头  
    
    ftm_quad_init(ftm1);
    ftm_quad_init(ftm2);
    OLED_Init();
     
    uart_init (uart0, 115200);                          //初始换串口
    mpu_IIC_Init();     //初始化mpu6050软件i2c
    InitMPU6050();
    
    ftm_pwm_init(ftm3,ftm_ch0,10*1000,0);
    ftm_pwm_init(ftm3,ftm_ch1,10*1000,0);
    ftm_pwm_init(ftm3,ftm_ch2,10*1000,0);
    ftm_pwm_init(ftm3,ftm_ch3,10*1000,0);
    
    pit_init_ms(pit0,2);
    enable_irq(PIT0_IRQn);
    
    while(1)
    {

    }
}

#endif


//陀螺仪调试程序
#if debug_mode==1

int main(void)
{
  get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
  mpu_IIC_Init();     //初始化mpu6050软件i2c
  InitMPU6050();
  uart_init (uart0, 115200);                          //初始换串口
     ftm_pwm_init(ftm3,ftm_ch0,10*1000,0);
   ftm_pwm_init(ftm3,ftm_ch1,10*1000,0);
   ftm_pwm_init(ftm3,ftm_ch2,10*1000,0);
   ftm_pwm_init(ftm3,ftm_ch3,10*1000,0);
  float var[3];//虚拟示波器调用的变量
  float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050读出的角度（未滤波）
  extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
  extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
  extern float angle,angle_dot,angle1,angle1_dot;
  while(1)
  {
    
    Get_AccData();
    Get_Gyro();
    
//    将陀螺仪的速度换算
//    mpu_gyro_float_x=(float)mpu_gyro_x*0.0610361;
    mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
//    mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;

    
    //将陀螺仪的加速度进行换算
    mpu_acc_float_x=(float)mpu_acc_x*0.2392615;
    mpu_acc_float_y=(float)mpu_acc_y*0.2392615;
    mpu_acc_float_z=(float)mpu_acc_z*0.2392615;
    
//利用加速度计计算倾角(角度值)
//利用函数将倾角转换为符合右手螺旋定则的转动系
//    angle_z_now=atan2(mpu_acc_y,mpu_acc_x)*(float)180/3.1415926;
//    angle_x_now=atan2(mpu_acc_z,mpu_acc_y)*(float)180/3.1415926;
    angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180/3.1415926;
    
    kalman_filter(angle_y_now,mpu_gyro_float_y);
    
    var[0]=mpu_gyro_float_y;
    var[1]=angle;
    var[2]=angle_y_now;
    
    vcan_sendware((void*)var,sizeof(var));
  }
}

#endif

//摄像头调试程序
#if debug_mode==2

int main(void)
{
   get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
   
       DisableInterrupts;   //关闭所有中断
       
    NVIC_SetPriorityGrouping(6);

    
    EnableInterrupts;
   
    set_irq_priority(PORTC_IRQn,0);
    set_irq_priority(DMA0_IRQn,1);
   
   OLED_Init();
    
   ftm_pwm_init(ftm3,ftm_ch0,10*1000,0);
   ftm_pwm_init(ftm3,ftm_ch1,10*1000,0);
   ftm_pwm_init(ftm3,ftm_ch2,10*1000,0);
   ftm_pwm_init(ftm3,ftm_ch3,10*1000,0);
   
   ov7725_init();//初始换摄像头  

   
   while(1)
   {
     find_middle_line();
     Image_Decompression(image_bin,image_dec[0]);
     dis_bmp(OV7725_H,OV7725_W,image_dec[0],123);
   }
}
#endif

//四元数解算调试程序
#if debug_mode==3

//extern _imu_st imu_data;
//extern _sensor_st sensor;
float roll,pitch,yaw;


int main(void)
{
    get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
   
    DisableInterrupts;   //关闭所有中断
       
    NVIC_SetPriorityGrouping(6);

    
    EnableInterrupts;
    
    float var[4];
    
    ftm_pwm_init(ftm3,ftm_ch0,10*1000,0);
    ftm_pwm_init(ftm3,ftm_ch1,10*1000,0);
    ftm_pwm_init(ftm3,ftm_ch2,10*1000,0);
    ftm_pwm_init(ftm3,ftm_ch3,10*1000,0);
   
    uart_init (uart0, 115200);
    
    float angle_x_now,angle_y_now,angle_z_now,mpu_gyro_float_x,mpu_gyro_float_y,mpu_gyro_float_z,mpu_acc_float_x,mpu_acc_float_y,mpu_acc_float_z;//mpu6050读出的角度（未滤波）
    extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
    extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
    extern float angle,angle_dot,angle1,angle1_dot;
   
    mpu_IIC_Init();     //初始化mpu6050软件i2c
    InitMPU6050();
    OLED_Init();
    
   while(mpu_dmp_init());
   while(1)
   {
        
        mpu_dmp_get_data(&pitch,&roll,&yaw);
      	MPU_Get_Accelerometer();	//µÃµ½¼ÓËÙ¶È´«¸ÐÆ÷Êý¾Ý
	MPU_Get_Gyroscope();	//µÃµ½ÍÓÂÝÒÇÊý¾Ý
        
//    将陀螺仪的速度换算
//               mpu_gyro_float_x=(float)mpu_gyro_x*0.0610361;
        mpu_gyro_float_y=(float)mpu_gyro_y*0.0610361;
//               mpu_gyro_float_z=(float)mpu_gyro_z*0.0610361;

    
    //将陀螺仪的加速度进行换算
        mpu_acc_float_x=(float)mpu_acc_x*0.2392615;
        mpu_acc_float_y=(float)mpu_acc_y*0.2392615;
        mpu_acc_float_z=(float)mpu_acc_z*0.2392615;

//利用加速度计计算倾角(角度值)
//利用函数将倾角转换为符合右手螺旋定则的转动系
//    angle_z_now=atan2(mpu_acc_y,mpu_acc_x)*(float)180/3.1415926;
//    angle_x_now=atan2(mpu_acc_z,mpu_acc_y)*(float)180/3.1415926;
        angle_y_now=atan2(mpu_acc_x,mpu_acc_z)*(float)180/3.1415926;
        
        //上位机看陀螺仪波形程序    
        var[0]=mpu_gyro_float_y;
        var[1]=roll;
        var[2]=pitch;
        var[3]=angle_y_now;
    
        vcan_sendware((void*)var,sizeof(var));

   }
}
#endif

//      //停止摄像头采集
//      disable_irq(INTERRUPT_NUNBERS);
//      DMA_IRQ_DIS(OV7725_DMA_CH);
//      //重新开启摄像头采集
//      enable_irq(INTERRUPT_NUNBERS);
//      DMA_IRQ_EN(OV7725_DMA_CH);
      //      OV7725_get_img();
//      img_extract(img,image_bin,OV7725_SIZE);
//      vcan_sendimg(img,OV7725_W * OV7725_H);




//uint8  data1[8] = {1,2,3,4,5,6,7,8};
//uint16 data2 = 60000;
//uint32 data3 = 600051;
//
//uint8  data11[8];
//uint16 data22; 
//uint32 data33;
//
//int main(void)
//{
//	get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
//	
//	//相关的库函数在 MK60DN10_flash.c 里面
//	FLASH_Init();			//初始化flash模块
//	FLASH_EraseSector(10);	//擦除扇区
//	FLASH_WriteSector(10,(const uint8 *)data1,8,0);		
//	FLASH_WriteSector(10,(const uint8 *)&data2,8,8);
//	FLASH_WriteSector(10,(const uint8 *)&data3,8,16);
//    for(;;)
//	{
//		//读取保存的data1数组数据
//		data11[0] = flash_read(10,0,uint8);	
//		data11[1] = flash_read(10,1,uint8);
//		data11[2] = flash_read(10,2,uint8);
//		data11[3] = flash_read(10,3,uint8);
//        data11[4] = flash_read(10,4,uint8);
//        data11[5] = flash_read(10,5,uint8);
//        data11[6] = flash_read(10,6,uint8);
//        data11[7] = flash_read(10,7,uint8);
//		
//		//读取保存的data2数组数据
//		data22 = flash_read(10,8,uint16);
//		
//		//读取保存的data3变量数据
//		data33 = flash_read(10,16,uint32);
//	}
//}

//测试车轮一圈换算编码器数值
//    int16_t var[2];
//
//    get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
//
//
//    ftm_quad_init(ftm1);
//    ftm_quad_init(ftm2);
//    uart_init (uart0, 115200);                          //初始换串口
//    
//    ftm_pwm_init(ftm3,ftm_ch0,10*1000,0);
//    ftm_pwm_init(ftm3,ftm_ch1,10*1000,0);
//    ftm_pwm_init(ftm3,ftm_ch2,10*1000,0);
//    ftm_pwm_init(ftm3,ftm_ch3,10*1000,0);
//
//    ftm_quad_clean(ftm1);
//    ftm_quad_clean(ftm2);
//    while(1)
//    {
//      var[0]=ftm_quad_get(ftm1);
//      var[1]=ftm_quad_get(ftm2);
//    
//      vcan_sendware((void*)var,sizeof(var));
//    }
