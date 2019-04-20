#include "headfile.h"
#include "common.h"


//图像处理，提取中线
#define display_original_line                   1       //0关闭原始中线、理论中线，1打开原始中线、理论中线，
#define display_cross_supplement_line           0       //0关闭十字补线，1打开十字补线。
#define display_ring_supplement_line            0       //0关闭圆环补线，1打开圆环补线。
#define display_Obstacle_supplement_line        0       //0关闭障碍补线，1打开障碍补线。

#define picture_wide    (OV7725_W/8)
#define picture_high    OV7725_H

#define imgbuff  image_bin

int middle_Line;                      //中线值
int Line_wide;                        //线宽度

uint8  farthest_scan_Y_max;                //纵向扫描最远点的行21
uint8  farthest_scan_X_max;              //纵向扫描最远点的列  

uint8  farthest_scan_Y_max_R;
uint8  farthest_scan_X_max_R;

uint8  farthest_scan_Y_max_L;
uint8  farthest_scan_X_max_L;

uint8 starting_line;

int8 prospect=35;//29;//前瞻//为隔行扫描所以前瞻取值应为奇数

uint8 stop_line_check=0;                   //起跑线检测标志，0为关闭起跑线检测 1为开始起跑线检测

uint8 slow_down=0;                    //减速标记

uint8  curve_flag=0;                              //出环标记


int L,R;

float middle_Line_temp[60]={0};                       //3行中线的值
int R_shrink_flag[60]={44};
int L_shrink_flag[60]={36};

void find_middle_line()
{
    int Img_L[picture_high]={0};
    int Img_R[picture_high]={0};

    uint8 L_flag=2,R_flag=2;                               //扫描成功0，扫描失败1,未扫描2
    
    static float middle_Line_last;
    int i,j,k;                                          //行 列字节 列位
    int aa;
    
    uint8  L_convergence_flag=0;                           //右收敛标志
    uint8  R_convergence_flag=0;                           //左收敛标志
    
    
    uint8  lose_line_front_effective_line_flag=53 ;        //全丢线前标志位
    uint8  lose_line_back_effective_line_flag=53;          //全丢线后标志位
    
    uint8 lose_line_start=59;                              //全丢线开始标志位
    
    uint8  lose_line_flag=0;                               //全丢线标志位
    
    uint8  R_curve_flag=0;                               //右弯标志位         
    uint8  L_curve_flag=0;                               //左弯标志位
    
    //uint8  R_slow_downe=0;
   // uint8  L_slow_downe=0;

    uint8  R_lose_line_front_flag=0;                       //左丢线前标志
    uint8  R_lose_line_back_flag=0;                        //左丢线后标志
    
    uint8  L_lose_line_front_flag=0;                       //左丢线前标志
    uint8  L_lose_line_back_flag=0;                        //左丢线后标志
    
    uint8  R_mutation_shrink_flag=0;                       //右缩跳变点
    uint8  L_mutation_shrink_flag=0;                       //左缩跳变点
    
    
    uint8  R_mutation_wide_flag=0;                         //右宽跳变点
    uint8  L_mutation_wide_flag=0;                         //左宽跳变点
    
   // uint8  anticipation_ring_flag=0;
    // uint8  a_ring_flag=0;
    
    static uint8 Obstacle_avoidance_flag=0;                //避障标志位 0无障碍 1右 2左
    
    static uint8 ring_flag=1;                              //默认不进入圆环//0不进入圆环 1右 2左
    
    static uint8 cross_flag=0;                             //默认没进十字
      
    static uint8 L_ring_flag=0;                   
    static uint8 R_ring_flag=0;                    
    
   // static uint8 Straight_Line_flag=0;                     //直线次数       
    
    int temp_int=0;                                     //int_临时变量
    
    int R_back=79;                                      //最远不丢线右值
    int L_back=0;                                       //最远不丢线左值    
    //变量初始化
    Img_R[59]=picture_wide*8-1;
    farthest_scan_Y_max=59;
    farthest_scan_Y_max_R=59;
    farthest_scan_Y_max_L=59;
       if(
          ring_flag==1
         &&R_mutation_shrink_flag<30)//&&R_mutation_shrink_flag<30
              {
                aa=R_shrink_flag[R_mutation_shrink_flag]+2;
              }
     else if(ring_flag==2
             &&L_mutation_shrink_flag<30 )//&&L_mutation_shrink_flag<30
              {
                aa=L_shrink_flag[L_mutation_shrink_flag]-2;
              }
      else
          {
            aa=39;
         }
    
    //纵向扫描
    for(j=5;j<75;j++)
    {
        for(i=59;i>0;i--)//基准点寻找从跳变点开始寻找
        {
            if((imgbuff[i*10+j/8]&(1<<(7-j%8)))!=0)
            { 
               if(j>aa)    //右最远
                  {
                     if(farthest_scan_Y_max_R>=i)
                    {
                        farthest_scan_Y_max_R=i+2;
                        farthest_scan_X_max_R=j;
                    } 
                  }
                else        //左最远
                {
                    if(farthest_scan_Y_max_L>i)
                    {
                        farthest_scan_Y_max_L=i+2;
                        farthest_scan_X_max_L=j;
                    }
                }
               
                break;
            }
        }
    }
    //确定横向扫描基准点
    if(ring_flag==1)//圆环
    {
        farthest_scan_Y_max=farthest_scan_Y_max_R;
        farthest_scan_X_max=farthest_scan_X_max_R;
    }
    else if(ring_flag==2)//圆环
    {
        farthest_scan_Y_max=farthest_scan_Y_max_L;
        farthest_scan_X_max=farthest_scan_X_max_L;
    }
      else//不在圆环取最远
   {
        if(farthest_scan_Y_max_R<farthest_scan_Y_max_L)
        {
            farthest_scan_Y_max=farthest_scan_Y_max_R;
            farthest_scan_X_max=farthest_scan_X_max_R;
        }
        else
        {
            farthest_scan_Y_max=farthest_scan_Y_max_L;
            farthest_scan_X_max=farthest_scan_X_max_L;
        }
   }
   //printf("最远准点：%d\n",farthest_scan_Y_max);
        //判断分叉路口
   /* if((farthest_scan_X_max_R-40)>5&&(39-farthest_scan_X_max_L)>5)
    {
        anticipation_ring_flag=1;
       // a_ring_flag=i;
    }*/
 
    //判断最远处
    if(farthest_scan_Y_max>=55)//最远处大于等于55，这判断图像无效，全黑
    {
        if(middle_Line_last>39)
        {
            middle_Line=middle_Line_last+1;
        }
        else
        {
            middle_Line=middle_Line_last-1;
        }
        stop_line_check=1;
        middle_Line_last=middle_Line;
        return ;
    }
   // printf("扫描基准点：%d\n",portrait_scan_X_max);
    //横向扫描基准线    
    for(i=59;i>=55;i--,i--)
    {  
        R_flag=1;       //默认找不到线
        L_flag=1;       //默认找不到线
        Img_R[i]=picture_wide*8-1;
        Img_L[i]=0;
        
        j=farthest_scan_X_max/8;
        
        //判断边线是否在横向扫描基准点的字节内 
        if((imgbuff[i*picture_wide+j]&((0xff>>((farthest_scan_X_max%8))+1)&0xff))!=0)
        {
            for(k=7-(farthest_scan_X_max%8);k>=0;k--)
            {
              //白点&&黑点&&+3<(i+1)*picture&&+3黑点
              if(
                 (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //白色
                 &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//找到右线                   //黑色
                {
                    if((j*8+7-k)<75)
                    {
                        Img_R[i]=j*8+7-k;
                        if(R_back>Img_R[i])
                            R_back=Img_R[i];
                        R_flag=0;                        
                        goto Right_first_line_finish;
                    }
                }
            }
        }
        else
        {
            for(j=j+1;j<picture_wide;j++)//右边
            {
                if(imgbuff[i*picture_wide+j]!=0)
                {
                    for(k=7;k>=0;k--)
                    {
                      //白点&&黑点&&+3<(i+1)*picture&&+3黑点
                      if(
                         (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //白色
                         &&(imgbuff[i*picture_wide+j]&(1<<k))!=0 )//找到右线                   //黑色
                        {
                            if((j*8+7-k)<75)
                            {
                                Img_R[i]=j*8+7-k;
                                if(R_back>Img_R[i])
                                    R_back=Img_R[i];
                                R_flag=0;
                                goto Right_first_line_finish;
                            }
                        }
                    }
                }
            }
        }        
        Right_first_line_finish:
           
        j=farthest_scan_X_max/8;
        //判断边线是否在横向扫描基准点的字节内 
        if((imgbuff[i*picture_wide+j]&((0xff<<(8-(farthest_scan_X_max%8)))&0xff))!=0)
        {
            for(k=7-(farthest_scan_X_max%8);k<8;k++)
            {
                if(
                   (((k-1)>=0)
                    ?(imgbuff[i*picture_wide+j]&(1<<(k-1)))==0
                    :(imgbuff[i*picture_wide+j+1]&(1<<(k-1+8)))==0)
                   &&(imgbuff[i*picture_wide+j]&(1<<k))!=0
                   )
                {
                    if((j*8+7-k)>5)
                    {
                        Img_L[i]=j*8+7-k;
                        if(L_back<Img_L[i])
                            L_back=Img_L[i];
                        L_flag=0;
                        goto Left_first_line_finish;
                    }
                }
            }
        }
        else
        {
            for(j=j-1;j>=0;j--)//左边
            {
                if(imgbuff[i*picture_wide+j]!=0)
                {
                    for(k=0;k<8;k++)
                    {
                        if(
                           (((k-1)>=0)
                            ?(imgbuff[i*picture_wide+j]&(1<<(k-1)))==0
                            :(imgbuff[i*picture_wide+j+1]&(1<<(k-1+8)))==0)
                           &&(imgbuff[i*picture_wide+j]&(1<<k))!=0
                           )
                        {
                            if((j*8+7-k)>5)
                            {
                                Img_L[i]=j*8+7-k;
                                if(L_back<Img_L[i])
                                    L_back=Img_L[i];
                                L_flag=0;
                                goto Left_first_line_finish;
                            }
                        }
                    }
                }
            }
        }
        
        Left_first_line_finish:
          
        //判断收敛 
        if(i==55)   
        {
            if((Img_R[i]-Img_L[i])>=(Img_R[i+2]-Img_L[i+2])) 
            {
                if(R_convergence_flag==0&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4])     //收敛条件
                {
                    R_convergence_flag=i+2;     //右收敛标志
      //             printf("右收敛标志:%d\n",R_convergence_flag);
                }
                if(L_convergence_flag==0&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4])     //收敛条件
                {
                    L_convergence_flag=i+2;     //左收敛标志
       //           printf("左收敛标志:%d\n",L_convergence_flag);
                }
            }
            if(R_flag==1&&L_flag==1)
            {
                lose_line_flag=1; 
                lose_line_start=i;
      //          printf("全丢线标志:%d\n",lose_line_start);
            }            
        }
        
        //左丢线
        if(R_flag==1&&R_lose_line_front_flag==0)
        {
            R_lose_line_front_flag=i;
      //       printf("右丢线开始标志:%d\n",R_lose_line_front_flag);
        }
        if(L_flag==1&&L_lose_line_front_flag==0)
        {
            L_lose_line_front_flag=i;
      //     printf("左丢线开始标志:%d\n",L_lose_line_front_flag);
        }
            
          
        middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
        Line_wide=Img_R[i]-Img_L[i];
        
        //中线显示        
#if (display_original_line==1)
        imgbuff[i*picture_wide+(int)((int)middle_Line_temp[i]/8+0.5)]|=1<<7-(int)((int)middle_Line_temp[i]%8+0.5);       //现实中线
        imgbuff[i*picture_wide+picture_wide/2]=imgbuff[i*picture_wide+picture_wide/2]|(1<<7);                 //摄像头理论中线
#endif
    }
    for(i=53;i>=farthest_scan_Y_max+1;i--,i--)
    {
        R_flag=1;       //默认找不到线
        L_flag=1;       //默认找不到线
        Img_R[i]=picture_wide*8-1;    
        j=farthest_scan_X_max/8;
        if((imgbuff[i*picture_wide+j]&((0xff>>((farthest_scan_X_max%8))+1)&0xff))!=0)
        {
            for(k=7-(farthest_scan_X_max%8);k>=0;k--)
            {
              //白点&&黑点&&+3<(i+1)*picture&&+3黑点
                if(
                    (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //白色
                    &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//找到右线                   //黑色         
                {
                  if((j*8+7-k)<=Img_R[i+2]+5&&(j*8+7-k)<77)
                  {
                    Img_R[i]=j*8+7-k;
                    if(R_back>Img_R[i])
                        R_back=Img_R[i];
                    R_flag=0;
                    goto Right_line_finish;
                  }

                }
            }
        }
        else
        {
            for(j=farthest_scan_X_max/8;j<picture_wide;j++)//右边
            {
                if(imgbuff[i*picture_wide+j]!=0)
                {
                    for(k=7;k>=0;k--)
                    {
                      //白点&&黑点&&+3<(i+1)*picture&&+3黑点
                      if(
                         (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //白色
                         &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//找到右线                   //黑色
                        {
                          if((j*8+7-k)<=Img_R[i+2]+5&&(j*8+7-k)<77)
                          {
                            Img_R[i]=j*8+7-k;
                            if(R_back>Img_R[i])
                                R_back=Img_R[i];
                            R_flag=0;
                            goto Right_line_finish;
                          }

                        }
                    }
                }
            }
        }
          
        Right_line_finish:
        j=farthest_scan_X_max/8;
        if(
           (imgbuff[i*picture_wide+j]&((0xff<<(8-(farthest_scan_X_max%8)))&0xff))!=0
           )
        {
            for(k=7-(farthest_scan_X_max%8);k<8;k++)
            {
                if(
                   (((k-1)>=0)
                    ?(imgbuff[i*picture_wide+j]&(1<<(k-1)))==0
                    :(imgbuff[i*picture_wide+j+1]&(1<<(k-1+8)))==0)
                   &&(imgbuff[i*picture_wide+j]&(1<<k))!=0
                   )
                {
                  if((j*8+7-k)>=Img_L[i+2]-5&&(j*8+7-k)>2)
                  {
                    Img_L[i]=j*8+7-k;
                    if(L_back<Img_L[i])
                        L_back=Img_L[i];
                    L_flag=0;
                    goto Left_line_finish;
                  }
                }
            }
        }
        else
        {
            for(j=j-1;j>=0;j--)//左边
            {
                if(imgbuff[i*picture_wide+j]!=0)
                {
                    for(k=0;k<8;k++)
                    {
                        if(
                           (((k-1)>=0)
                            ?(imgbuff[i*picture_wide+j]&(1<<(k-1)))==0
                            :(imgbuff[i*picture_wide+j+1]&(1<<(k-1+8)))==0)
                           &&(imgbuff[i*picture_wide+j]&(1<<k))!=0
                           )
                        {
                          if((j*8+7-k)>=Img_L[i+2]-5&&(j*8+7-k)>2)
                          {
                            Img_L[i]=j*8+7-k;
                            if(L_back<Img_L[i])
                                L_back=Img_L[i];
                            L_flag=0;
                            goto Left_line_finish;
                          }
                        }
                    }
                }
            }
        }
            
        Left_line_finish:
              
        //跳变点判断
        if((Img_L[i]-Img_L[i+2])>10&&L_mutation_shrink_flag==0)
        {
            L_mutation_shrink_flag=i;
            L_shrink_flag[i]=Img_L[i];
      //     printf("左缩：%d\n",L_mutation_shrink_flag);
      //     printf("左缩j：%d\n",L_shrink_flag[i]);
        }
        
        if((Img_R[i+2]-Img_R[i])>10&&R_mutation_shrink_flag==0)
        {
            R_mutation_shrink_flag=i;
            R_shrink_flag[i]=Img_R[i];
       //  printf("右缩：%d\n",R_mutation_shrink_flag);
        }
        
        if((Img_L[i+2]-Img_L[i])>10&&L_mutation_wide_flag==0)
        {
            L_mutation_wide_flag=i+2;
      //     printf("左宽：%d\n",L_mutation_wide_flag);
        }
        
        if((Img_R[i]-Img_R[i+2])>10&&R_mutation_wide_flag==0)
        {
            R_mutation_wide_flag=i+2;
      //      printf("右宽：%d\n",R_mutation_wide_flag);
        }       
          
         //判断收敛          
        if((Img_R[i+2]-Img_L[i+2])>=(Img_R[i+4]-Img_L[i+4])) 
        {
            if(R_convergence_flag==0&&Img_R[i]>Img_R[i+4]&&Img_R[i+2]>Img_R[i+4]&&Img_R[i+4]<=Img_R[i+6])     //收敛条件
            {
                R_convergence_flag=i+4;     //右收敛标志
      //        printf("右收敛点：%d\n",R_convergence_flag);      
            }
            if(L_convergence_flag==0&&Img_L[i]<Img_L[i+4]&&Img_L[i+2]<Img_L[i+4]&&Img_L[i+4]>=Img_L[i+6])     //收敛条件
            {
                L_convergence_flag=i+4;     //左收敛标志
         //      printf("左收敛点：%d\n",L_convergence_flag);
            }
        }  
            
        //全丢线标志
        if(R_flag==1&&L_flag==1&&lose_line_flag==0)
        {
            lose_line_flag=1;
            lose_line_start=i;
        }
        //十字上收敛
        if(R_flag==0&&L_flag==0&&lose_line_flag==1)  
        {
            lose_line_back_effective_line_flag=i-2;
            lose_line_flag=2;
        }

        if(Img_R[i]>Img_R[i+2]&&Img_R[i+2]>Img_R[i+4]&&R_curve_flag==0&&i>25)
        {
          R_curve_flag=1;
       //    printf("右弯");
        }
        if(Img_L[i]<Img_L[i+2]&&Img_L[i+2]<Img_L[i+4]&&L_curve_flag==0&&i>25)
        {
          L_curve_flag=1;
        //   printf("左弯");
        }
    /*    if(farthest_scan_Y_max>19)
        {
            if(R_curve_flag==1
               &&R_lose_line_front_flag>25
               &&R_lose_line_front_flag!=59
               &&(L_lose_line_front_flag==59||L_lose_line_front_flag==0)
               &&(-3<R_lose_line_back_flag-farthest_scan_Y_max<3)
               &&R_slow_downe==0)
            {
                  R_slow_downe=1;
            }
           if(L_curve_flag==1
              &&L_lose_line_front_flag>25
              &&L_lose_line_front_flag!=59
              &&(R_lose_line_front_flag==59||R_lose_line_front_flag==0)
              &&(-3<L_lose_line_back_flag-farthest_scan_Y_max<3)
              &&L_slow_downe==0)
            {
              L_slow_downe=1;
            }
        
        }*/
    /*    if(R_flag==1&&L_flag==0&&R_lose_line_flag==0) //右边丢线
        {
            R_lose_line_flag=1;
        }
       if(R_flag==0&&L_flag==0&&R_lose_line_flag==1) //右边找到线
        {
            R_lose_line_flag=2;
            R_UP_convergence_flag=i;       
            
        }
        
        
        if(R_flag==0&&L_flag==1&&L_lose_line_flag==0)  //左边丢线
        {
            L_lose_line_flag=1;
        }
       if(R_flag==0&&L_flag==0&&L_lose_line_flag==1)  //左边找到线
        {
            L_lose_line_flag=2;
            L_UP_convergence_flag=i;
            
        }*/
        

        
        //右丢线开始
        if(R_flag==1&&R_lose_line_front_flag==0)
        {
            R_lose_line_front_flag=i;
         //printf("右丢线开始：%d\n",R_lose_line_front_flag);
        }
        //右丢线结束
        if(R_lose_line_front_flag!=0&&R_flag==0&&R_lose_line_back_flag==0)
        {
            R_lose_line_back_flag=i;
      //   printf("右丢线结束：%d\n",R_lose_line_back_flag);
        }
        //右丢线开始
        if(L_flag==1&&L_lose_line_front_flag==0)
        {
            L_lose_line_front_flag=i;
       //      printf("左丢线开始：%d\n",L_lose_line_front_flag);
        }
        //右丢线结束
        if(L_lose_line_front_flag!=0&&L_flag==0&&L_lose_line_back_flag==0)
        {
            L_lose_line_back_flag=i;
     //       printf("左丢线结束：%d\n",L_lose_line_back_flag);
        }
                
        //简单丢线处理
        if(L_flag==1&&R_flag==0)
            middle_Line_temp[i]=(Img_R[i]-(float)Line_wide/2);
        else if(L_flag==0&&R_flag==1)
            middle_Line_temp[i]=(Img_L[i]+(float)Line_wide/2);
        else if(L_flag==1&&R_flag==1)           
            middle_Line_temp[i]= middle_Line_temp[i+2];           
        else
        {
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
            Line_wide=Img_R[i]-Img_L[i];
        }
                    
        //清除标志
        L_flag=0;
        R_flag=0;

        //中线显示        
#if (display_original_line==1)
        imgbuff[i*picture_wide+(int)((int)middle_Line_temp[i]/8+0.5)]|=1<<7-(int)((int)middle_Line_temp[i]%8+0.5);       //现实中线
        imgbuff[i*picture_wide+picture_wide/2]=imgbuff[i*picture_wide+picture_wide/2]|(1<<7);                 //摄像头理论中线
#endif            
    }
    //最远行（基数）
    farthest_scan_Y_max=i+2;                 
  //*************************十字处理（只存在上收敛）**********************//
    if(
       (R_convergence_flag<lose_line_back_effective_line_flag||L_convergence_flag<lose_line_back_effective_line_flag)//收敛在白前
       &&lose_line_flag==2//中白，上收敛
       &&cross_flag==1//十字标志
       )
    {
      starting_line=1;
        for(i=59;i>lose_line_back_effective_line_flag;i--,i--)
        {
            Img_R[i]=Img_R[lose_line_back_effective_line_flag];
            
            Img_L[i]=Img_L[lose_line_back_effective_line_flag];
                
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
            
#if (display_cross_supplement_line==1)
            imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
            imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
            imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8); 
#endif
        }
      //  ring_flag=0;
      //  ring_direction_flag=0;
        Obstacle_avoidance_flag=0;
        
    }
    //十字处理（存在下收敛）    
    else if(
            (R_convergence_flag!=0&&L_convergence_flag!=0)//下收敛
            &&(R_convergence_flag-L_convergence_flag)<10
            &&(R_convergence_flag-L_convergence_flag)>-10
            &&lose_line_flag==2//中白，上收敛，
            &&(Img_R[R_convergence_flag]-Img_R[lose_line_back_effective_line_flag])>-2
            &&(Img_L[lose_line_back_effective_line_flag]-Img_L[L_convergence_flag])>-2
            &&lose_line_start-lose_line_back_effective_line_flag>8
            &&lose_line_back_effective_line_flag<R_convergence_flag
            &&lose_line_back_effective_line_flag<L_convergence_flag
            )
    {
      starting_line=1;
        //取最近的收敛点
        if(R_convergence_flag>L_convergence_flag)
        {
            lose_line_front_effective_line_flag=R_convergence_flag;
        }
        else
        {
            lose_line_front_effective_line_flag=L_convergence_flag;
        }
        //十字补线
        for(i=lose_line_front_effective_line_flag;i>lose_line_back_effective_line_flag;i--,i--)
        {        
            if(R_convergence_flag>=i)
            {
                Img_R[i]=(int)(Img_R[lose_line_front_effective_line_flag+2]+(i-lose_line_front_effective_line_flag+2)*(Img_R[(lose_line_back_effective_line_flag-2)]-Img_R[lose_line_front_effective_line_flag+2]) /((lose_line_back_effective_line_flag-2)-lose_line_front_effective_line_flag+2)+0.5);  //线性直插
            }
            if(L_convergence_flag>=i)
            {
                Img_L[i]=(int)(Img_L[lose_line_front_effective_line_flag+2]+(i-lose_line_front_effective_line_flag+2)*(Img_L[(lose_line_back_effective_line_flag-2)]-Img_L[lose_line_front_effective_line_flag+2])/((lose_line_back_effective_line_flag-2)-lose_line_front_effective_line_flag+2)+0.5);  //线性直插
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
            
#if (display_cross_supplement_line==1)            
            imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
            imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
            imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8; 
#endif
        }
        cross_flag=1;
        //ring_flag=0;
        //ring_direction_flag=0;
        Obstacle_avoidance_flag=0;
        
    }  
 //***********************   起跑线   *****************************//
        else if(stop_line_check==0
            &&(Img_R[59]-Img_L[59])<8
            &&(Img_R[57]-Img_L[57])<8
            &&(Img_R[55]-Img_L[55])<8
            &&middle_Line<43
            &&middle_Line>37
            &&starting_line==1)
    {
        stop_line_check=1;
    }
    //*******************************圆环****************************//
   else if(                   //右圆环前部分   
            (R_convergence_flag>40||R_mutation_wide_flag>40)//收敛
            &&R_mutation_shrink_flag>15
            &&R_lose_line_front_flag>40 //右丢线
            &&R_lose_line_front_flag!=59 
            &&(R_convergence_flag-R_mutation_shrink_flag>8||R_mutation_wide_flag-R_mutation_shrink_flag>8)
            &&(R_convergence_flag-R_lose_line_front_flag==2||R_mutation_wide_flag-R_lose_line_front_flag==2)
            &&lose_line_flag!=2//不丢线
            &&ring_flag==0              //没有方向
            &&L_convergence_flag==0
            &&R_curve_flag==0
            &&farthest_scan_Y_max<22
            &&(L_lose_line_front_flag==0||L_lose_line_front_flag==59||R_lose_line_front_flag<20) 
          //&&ring_direction_flag==0  
          //&&((Img_R[R_convergence_flag]-Img_R[R_lose_line_front_flag])<-2||(Img_R[R_mutation_wide_flag]-Img_R[R_lose_line_front_flag])<-2)
          //&&(Img_R[R_convergence_flag]<70||Img_R[R_mutation_wide_flag]<70)
            )
          {
            slow_down=1;
             ring_flag=1;
             starting_line=1;
         //  printf("右圆环前部分 111");
        for(i=57;i>R_lose_line_back_flag;i--,i--)
        {
            temp_int=(int)(Img_R[59]+(i-59)*(Img_R[R_convergence_flag]-Img_R[59])/(R_convergence_flag-59)+0.5);  //线性直插;
            if(Img_R[i]>temp_int)
            {
                Img_R[i]=temp_int;
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
              imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
              imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
          }
   else if(          //左圆环前部分            
           (L_convergence_flag>40||L_mutation_wide_flag>40)    //收敛
            &&L_mutation_shrink_flag>15
            &&L_lose_line_front_flag>40   //左丢线
            &&L_lose_line_front_flag!=59 
            &&(L_convergence_flag-L_mutation_shrink_flag>8||L_mutation_wide_flag-L_mutation_shrink_flag>8)
            &&(L_convergence_flag-L_lose_line_front_flag==2||L_mutation_wide_flag-L_lose_line_front_flag==2)
            &&lose_line_flag!=2//不丢线
            &&ring_flag==0              //没有方向
            &&R_convergence_flag==0
            &&L_curve_flag==0
            &&farthest_scan_Y_max<22
            &&(R_lose_line_front_flag==0||R_lose_line_front_flag==59||R_lose_line_front_flag<20)
          //&&ring_direction_flag==0  
          //&&((Img_L[L_lose_line_front_flag]-Img_L[L_convergence_flag])<-2||(Img_L[L_lose_line_front_flag]-Img_L[L_mutation_wide_flag])<-2)
          //&&(Img_L[L_convergence_flag]>10||Img_L[L_mutation_wide_flag]>10)
            )
            {
              slow_down=1;
              starting_line=1;
          //     printf("左圆环前部分 2222");
               ring_flag=2;  
            for(i=57;i>L_lose_line_back_flag;i--,i--)
            {
                temp_int=(int)(Img_L[59]+(i-59)*(Img_L[L_convergence_flag]-Img_L[59])/(L_convergence_flag-59)+0.5);  //线性直插;
                if(Img_L[i]<temp_int)
                {
                    Img_L[i]=temp_int;
                }
                middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
              imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8; 
              imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
              }
             }
  /*  else if( //右圆环后部分
            R_lose_line_flag==2
        //   &&R_convergence_flag<45
            &&20<R_UP_convergence_flag
            &&R_UP_convergence_flag<40
         //   &&lose_line_flag!=2
           &&ring_flag==1
           &&Img_R[R_UP_convergence_flag]<53
        //   &&anticipation_ring_flag==1
         //&&L_convergence_flag==0
         //&&L_lose_line_front_flag==0
            )
    { 
      printf("右圆环后部分 3333");
             R_lose_line_flag=0;
             for(i=57;i>R_UP_convergence_flag;i--,i--)
             {
                temp_int=(int)(Img_L[59]+(i-59)*(Img_R[R_UP_convergence_flag]+8-Img_L[59])/(R_UP_convergence_flag-59)+0.5);  //线性直插;
                if(Img_L[i]<temp_int)
                {
                    Img_L[i]=temp_int;
                }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
    }
     else if(     //左圆环后部分
            L_lose_line_flag==2
          // &&L_convergence_flag<45
            &&20<L_UP_convergence_flag
            &&L_UP_convergence_flag<40 
          //  &&anticipation_ring_flag==1
           // &&lose_line_flag!=2
            &&ring_flag==2
            &&Img_L[L_UP_convergence_flag]>27
           // &&R_convergence_flag==0
              )
          {  
             printf("左圆环后部分 44444\n");
           L_lose_line_flag=0;
          for(i=57;i>L_UP_convergence_flag;i--,i--)
          {
            temp_int=(int)(Img_R[59]+(i-59)*(Img_L[L_UP_convergence_flag]-8-Img_R[59])/(L_UP_convergence_flag-59)+0.5);  //线性直插;
            if(Img_R[i]>temp_int)
            {
                Img_R[i]=temp_int;
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
              }
          }*/
        else if( //右圆环后部分
                R_mutation_shrink_flag>20
                &&R_convergence_flag>30
                &&R_mutation_shrink_flag<R_convergence_flag
                &&R_lose_line_front_flag>30   //左丢线
                &&R_lose_line_front_flag!=59 
                &&(R_mutation_shrink_flag-R_lose_line_back_flag==2||R_mutation_shrink_flag-R_lose_line_back_flag==0)
                &&(L_lose_line_front_flag==0||L_lose_line_front_flag==59||L_lose_line_front_flag<20)
                &&ring_flag==1 
                &&R_curve_flag==1   
              //&&lose_line_flag!=2//不丢线
              //&&anticipation_ring_flag==1
              //&&L_mutation_shrink_flag==0
              //&&Img_R[R_mutation_shrink_flag]<55
            )
    {
              R_ring_flag=1;
          //    printf("右圆环后部分 55555");
             for(i=57;i>R_mutation_shrink_flag;i--,i--)
             {
                temp_int=(int)(Img_L[59]+(i-59)*(Img_R[R_mutation_shrink_flag]+20-Img_L[59])/(R_mutation_shrink_flag-59)+0.5);  //线性直插;
                if(Img_L[i]<temp_int)
                {
                    Img_L[i]=temp_int;
                }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
    }
     else if(     //左圆环后部分
           L_mutation_shrink_flag>20
           &&L_convergence_flag>30
           &&L_mutation_shrink_flag<L_convergence_flag
           &&L_lose_line_front_flag>30   //左丢线
           &&L_lose_line_front_flag!=59 
           &&(L_mutation_shrink_flag-L_lose_line_back_flag==2||L_mutation_shrink_flag-L_lose_line_back_flag==0)
           &&(R_lose_line_front_flag==0||R_lose_line_front_flag==59||R_lose_line_front_flag<20)
           &&ring_flag==2
           &&L_curve_flag==1 
         //&&lose_line_flag!=2//不丢线 
         //&&anticipation_ring_flag==1
         //&&R_mutation_shrink_flag==0
         //&&Img_L[L_mutation_shrink_flag]>25
              )
          {   
          L_ring_flag=1;
          //slow_down=1;
        //  printf("左圆环后部分 666666\n");
          for(i=57;i>L_mutation_shrink_flag ;i--,i--)
          {
            temp_int=(int)(Img_R[59]+(i-59)*(Img_L[L_mutation_shrink_flag]-20-Img_R[59])/(L_mutation_shrink_flag-59)+0.5);  //线性直插;
            if(Img_R[i]>temp_int)
            {
                Img_R[i]=temp_int;
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
          }
  /*      else if( //右圆环后部分
                R_mutation_shrink_flag>20
                &&R_convergence_flag==0
                &&R_lose_line_front_flag==59
                &&(R_mutation_shrink_flag-R_lose_line_back_flag==2||R_mutation_shrink_flag-R_lose_line_back_flag==0)
                &&(L_lose_line_front_flag==0||L_lose_line_front_flag==59||L_lose_line_front_flag<20)
                &&ring_flag==1 
            )
    {     
            R_ring_flag=1;
            //slow_down=1;
              printf("右圆环后部分 777777");
             for(i=57;i>R_mutation_shrink_flag;i--,i--)
             {
                temp_int=(int)(Img_L[59]+(i-59)*(Img_R[R_mutation_shrink_flag]+20-Img_L[59])/(R_mutation_shrink_flag-59)+0.5);  //线性直插;
                if(Img_L[i]<temp_int)
                {
                    Img_L[i]=temp_int;
                }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
    }
     else if(     //左圆环后部分
            L_mutation_shrink_flag>20
            &&L_convergence_flag==0
            &&L_lose_line_front_flag==59
            &&(L_mutation_shrink_flag-L_lose_line_back_flag==2||L_mutation_shrink_flag-L_lose_line_back_flag==0)
            &&(R_lose_line_front_flag==0||R_lose_line_front_flag==59||R_lose_line_front_flag<20)
            &&ring_flag==2
              )
        { 
          L_ring_flag=1;
          printf("左圆环后部分 88888\n");
          for(i=57;i>L_mutation_shrink_flag ;i--,i--)
          {
            temp_int=(int)(Img_R[59]+(i-59)*(Img_L[L_mutation_shrink_flag]-20-Img_R[59])/(L_mutation_shrink_flag-59)+0.5);  //线性直插;
            if(Img_R[i]>temp_int)
            {
                Img_R[i]=temp_int;
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
          }
        }*/
 else if(
            R_ring_flag==1  //右环
            &&(L_mutation_wide_flag>35||L_convergence_flag>35)
            &&L_lose_line_front_flag>35 
            &&L_lose_line_front_flag!=59
            &&farthest_scan_Y_max>22
          //  &&(L_convergence_flag-L_lose_line_back_flag>3||L_mutation_wide_flag-L_lose_line_back_flag>3)
            &&(L_convergence_flag-L_lose_line_front_flag==2||L_mutation_wide_flag-L_lose_line_front_flag==2)
     //       &&(R_mutation_shrink_flag>30||L_mutation_shrink_flag>30)
         //   &&R_mutation_shrink_flag==0
       //     &&R_mutation_wide_flag==0
          //  &&L_lose_line_flag==2
            )
          {
     // printf("右圆环出弯1111 \n");
              R=1;
              ring_flag=0;
              curve_flag=1;
              //anticipation_ring_flag=0;
              for(i=57;i>farthest_scan_Y_max;i--,i--)
             {
                temp_int=(int)(Img_L[59]+(i-59)*(79-Img_L[59])/(35-59)+0.5);  //线性直插;
                if(Img_L[i]<temp_int)
                {
                    Img_L[i]=temp_int;
                }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
    }
        else if(
            L_ring_flag==1  //左环
            &&(R_mutation_wide_flag>35||R_convergence_flag>35)
            &&R_lose_line_front_flag>35 //右丢线
            &&R_lose_line_front_flag!=59 
            &&farthest_scan_Y_max>22
        //    &&(R_convergence_flag-R_lose_line_back_flag>3||R_mutation_wide_flag-R_lose_line_back_flag>3)
            &&(R_convergence_flag-R_lose_line_front_flag==2||R_mutation_wide_flag-R_lose_line_front_flag==2)
         //   &&(R_mutation_shrink_flag>30||L_mutation_shrink_flag>30)
         //   &&L_mutation_shrink_flag==0
        //    &&L_mutation_wide_flag==0
        //    &&R_lose_line_flag==2
            )
          {   
      //      printf("左圆环出弯2222\n");
             L=1;  
             ring_flag=0;
             curve_flag=2;
            for(i=57;i>farthest_scan_Y_max;i--,i--)
            {
              temp_int=(int)(Img_R[59]+(i-59)*(0-Img_R[59])/(35-59)+0.5);  //线性直插;
            if(Img_R[i]>temp_int)
            {
                Img_R[i]=temp_int;
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
          }
     else if(
            R_ring_flag==1  //右环
            &&(L_mutation_wide_flag>35||L_convergence_flag>35)
            &&L_lose_line_front_flag>35 
            &&farthest_scan_Y_max>22
            &&R_curve_flag==1 
            )
          {
              R=1;
              ring_flag=0;
              curve_flag=1;
              for(i=57;i>farthest_scan_Y_max;i--,i--)
             {
                temp_int=(int)(Img_L[59]+(i-59)*(79-Img_L[59])/(35-59)+0.5);  //线性直插;
                if(Img_L[i]<temp_int)
                {
                    Img_L[i]=temp_int;
                }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
    }
        else if(
            L_ring_flag==1  //左环
            &&(R_mutation_wide_flag>35||R_convergence_flag>35)
            &&R_lose_line_front_flag>35 //右丢线
           // &&R_lose_line_front_flag==59 
            &&farthest_scan_Y_max>22
            &&L_curve_flag==1 
            )
          {   
             L=1;  
             ring_flag=0;
             curve_flag=2;
            for(i=57;i>farthest_scan_Y_max;i--,i--)
            {
              temp_int=(int)(Img_R[59]+(i-59)*(0-Img_R[59])/(35-59)+0.5);  //线性直插;
            if(Img_R[i]>temp_int)
            {
                Img_R[i]=temp_int;
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
          }
       else if(
            R_ring_flag==1  //右环
            &&lose_line_start>40
            &&lose_line_start!=59
            &&curve_flag==1
            )
          {
      //    printf("右圆环出弯 33333333\n");
              ring_flag=0;
              R=1;
              //anticipation_ring_flag=0;
              for(i=57;i>farthest_scan_Y_max;i--,i--)
             {
                temp_int=(int)(Img_L[59]+(i-59)*(79-Img_L[59])/(35-59)+0.5);  //线性直插;
                if(Img_L[i]<temp_int)
                {
                    Img_L[i]=temp_int;
                }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
    }
       else if(
            L_ring_flag==1  //左环
            &&lose_line_start>40
            &&lose_line_start!=59
            &&curve_flag==2
            )
          {   
      //    printf("左圆环出弯444444444\n");
             ring_flag=0;
             L=1;
          //   ring_direction_flag=0;
            //anticipation_ring_flag=0;
            for(i=57;i>farthest_scan_Y_max;i--,i--)
            {
              temp_int=(int)(Img_R[59]+(i-59)*(0-Img_R[59])/(35-59)+0.5);  //线性直插;
            if(Img_R[i]>temp_int)
            {
                Img_R[i]=temp_int;
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_ring_supplement_line==1)
                imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
            }
          }
      else if( //右圆环
             // R_mutation_shrink_flag<40
              R_mutation_shrink_flag>10
              &&ring_flag==0
              &&curve_flag==1
            )
             {
       //        printf("右圆环清零 \n");
               R_ring_flag=0;
               curve_flag=0;
               slow_down=0;
               R=0;
             }
        else if(     //左圆环
           // L_mutation_shrink_flag<40
            L_mutation_shrink_flag>10
            &&ring_flag==0
            &&curve_flag==2
              )
             {
          //     printf("左圆环清零\n");
               L_ring_flag=0;
               curve_flag=0;
               slow_down=0;
               L=0;
             }
    //********************避障处理***********************//
    else if(
             R_mutation_shrink_flag>31
             &&L_mutation_shrink_flag==0
             &&R_convergence_flag!=0
             &&(R_convergence_flag<R_mutation_shrink_flag||R_mutation_wide_flag<R_mutation_shrink_flag)
             &&R_lose_line_front_flag<R_mutation_shrink_flag
             )
    {
        Obstacle_avoidance_flag=2;
        for(i=57;i>farthest_scan_Y_max;i--,i--)
        {
            {
                temp_int=Img_R[R_convergence_flag];
                if(Img_R[i]>temp_int)
                {
                    Img_R[i]=temp_int;
                }
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_Obstacle_supplement_line==1)
            imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
            imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
        }       
    }
    else if(
            L_mutation_shrink_flag>31
            &&R_mutation_shrink_flag==0
            &&L_convergence_flag!=0
            &&(L_convergence_flag<L_mutation_shrink_flag||L_mutation_wide_flag<L_mutation_shrink_flag)
            &&L_lose_line_front_flag<L_mutation_shrink_flag
            )
    {
        Obstacle_avoidance_flag=1;
        for(i=57;i>farthest_scan_Y_max;i--,i--)
        {
            {
                temp_int=Img_L[L_convergence_flag]+1;
                if(Img_L[i]<temp_int)
                {
                    Img_L[i]=temp_int;
                }
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_Obstacle_supplement_line==1)
            imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
            imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
        }
    }
    else if(Obstacle_avoidance_flag==1&&(L_mutation_wide_flag!=0||L_convergence_flag!=0))
    {
        for(i=59;i>farthest_scan_Y_max;i--,i--)
        {
            temp_int=Img_L[L_convergence_flag]+1;
            if(Img_L[i]<temp_int)
            {
                Img_L[i]=temp_int;
                
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_Obstacle_supplement_line==1)
            imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
            imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
        }
    }
    else if(Obstacle_avoidance_flag==2&&(R_mutation_wide_flag!=0||L_convergence_flag!=0))
    {
        for(i=59;i>farthest_scan_Y_max;i--,i--)
        {
            temp_int=Img_R[R_convergence_flag]+1;
            if(Img_R[i]>temp_int)
            {
                Img_R[i]=temp_int;
            }
            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
#if (display_Obstacle_supplement_line==1)
            imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
            imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
#endif
        }
    }
    else
    {
        Obstacle_avoidance_flag=0;
        cross_flag=0;
    }   
    //****************************中线提取*****************************//
    if(farthest_scan_Y_max<=prospect)
    {   
        middle_Line=(middle_Line_temp[55]+middle_Line_temp[57]+middle_Line_temp[53])/3*0.5+(middle_Line_temp[prospect]+middle_Line_temp[prospect+2])/2*0.5;
    }
    else
    {
        if((farthest_scan_Y_max-prospect)>=4)
        {
            middle_Line_temp[prospect]=(int)(middle_Line_temp[59]+(prospect-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);  //线性直插
            middle_Line_temp[(prospect+2)]=(int)(middle_Line_temp[59]+((prospect+2)-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);
        }
        else
        {
            middle_Line_temp[prospect]=(int)(middle_Line_temp[59]+(prospect-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);  //线性直插
        }
        middle_Line=(middle_Line_temp[55]+middle_Line_temp[57]+middle_Line_temp[53])/3*0.5+(middle_Line_temp[prospect]+middle_Line_temp[prospect+2])/2*0.5;

    }  
    middle_Line_last=middle_Line;
}