//#include "common.h"
//#include "headfile.h"
//
//#define picture_wide    (OV7725_W/8)
//#define picture_high    OV7725_H
//
//#define imgbuff  image_bin
//
////图像处理，提取中线
//#define display_original_line                   1       //0关闭原始中线、理论中线，1打开原始中线、理论中线，
//#define display_cross_supplement_line           1       //0关闭十字补线，1打开十字补线。
//#define display_ring_supplement_line            1       //0关闭圆环补线，1打开圆环补线。
//#define display_Obstacle_supplement_line        0       //0关闭障碍补线，1打开障碍补线。
//
//#define picture_wide    (OV7725_W/8)
//#define picture_high    OV7725_H
//
//int ring_front=0;
//int ling_front=0;
//
//int middle_Line;                      //中线值
//int Line_wide;                          //线宽度
//
//uint8  farthest_scan_Y_max;                //纵向扫描最远点的行21
//uint8  farthest_scan_X_max;              //纵向扫描最远点的列  
//
//uint8 L_F=0;
//uint8 R_F=0;
//
//uint8  add_line_flag=0;
//uint8  add_line_flag_R=0;
//
//
//uint8  farthest_scan_Y_max_R;
//uint8  farthest_scan_X_max_R;
//
//uint8  farthest_scan_Y_max_L;
//uint8  farthest_scan_X_max_L;
//
//uint8 starting_line;
//
//int8 prospect=33;//前瞻
//
//uint8 stop_line_check=0;                   //起跑线检测标志，0为关闭起跑线检测 1为开始起跑线检测
//
//uint8 slow_down=0;                    //减速标记
//
//uint8  curve_flag=0;                              //出环标记
//
//
//int L,R;
//
//float middle_Line_temp[60]={0};                       //3行中线的值
//int R_shrink_flag[60]={44};
//int L_shrink_flag[60]={36};
//
//int L_wide_flag[60]={36};
//int R_wide_flag[60]={44};
//
//void find_middle_line()
//{
//    int Img_L[picture_high]={0};
//    int Img_R[picture_high]={0};
//
//    uint8 L_flag=2,R_flag=2;                               //扫描成功0，扫描失败1,未扫描2
//    
//    static float middle_Line_last;
//    int i,j,k;                                          //行 列字节 列位
//    int aa=39;
//    
//    uint8  L_convergence_flag=0;                           //右收敛标志
//    uint8  R_convergence_flag=0;                           //左收敛标志
//    
//    uint8 R_ce=0;
//    uint8 L_ce=0;
//    
//    
//    uint8  lose_line_front_effective_line_flag=53 ;        //全丢线前标志位
//    uint8  lose_line_back_effective_line_flag=53;          //全丢线后标志位
//    
//    uint8 lose_line_start=59;                              //全丢线开始标志位
//    
//    uint8  lose_line_flag=0;                               //全丢线标志位
//    
//    uint8  R_curve_flag=0;                               //右弯标志位         
//    uint8  L_curve_flag=0;                               //左弯标志位
//    
//    //uint8  R_slow_downe=0;
//   // uint8  L_slow_downe=0;
//
//    uint8  R_lose_line_front_flag=0;                       //左丢线前标志
//    uint8  R_lose_line_back_flag=0;                        //左丢线后标志
//    
//    uint8  L_lose_line_front_flag=0;                       //左丢线前标志
//    uint8  L_lose_line_back_flag=0;                        //左丢线后标志
//    
//    uint8  R_mutation_shrink_flag=0;                       //右缩跳变点
//    uint8  L_mutation_shrink_flag=0;                       //左缩跳变点
//    
//    
//    uint8  R_mutation_wide_flag=0;                         //右宽跳变点
//    uint8  L_mutation_wide_flag=0;                         //左宽跳变点
//    
//   // uint8  anticipation_ring_flag=0;
//    // uint8  a_ring_flag=0;
//    
//    static uint8 Obstacle_avoidance_flag=0;                //避障标志位 0无障碍 1右 2左
//    
//    static uint8 ring_flag=0;                              //默认不进入圆环//0不进入圆环 1右 2左
//    
//    static uint8 cross_flag=0;                             //默认没进十字
//      
//    static uint8 L_ring_flag=0;                   
//    static uint8 R_ring_flag=0;                    
//    
//   // static uint8 Straight_Line_flag=0;                     //直线次数       
//    
//    int temp_int=0;                                     //int_临时变量
//    
//    int R_back=79;                                      //最远不丢线右值
//    int L_back=0;                                       //最远不丢线左值    
//    //变量初始化
//    Img_R[59]=picture_wide*8-1;
//    farthest_scan_Y_max=59;
//    farthest_scan_Y_max_R=59;
//    farthest_scan_Y_max_L=59;
//
//
//    //纵向扫描
//    for(j=5;j<75;j++)
//    {
//        for(i=59;i>0;i--)
//        {
//            if((imgbuff[i*10+j/8]&(1<<(7-j%8)))!=0)
//            { 
//               if(j>aa)    //右最远
//                  {
//                     if(farthest_scan_Y_max_R>=i)
//                    {
//                        farthest_scan_Y_max_R=i;
//                        farthest_scan_X_max_R=j;
//                    } 
//                  }
//                else        //左最远
//                {
//                    if(farthest_scan_Y_max_L>i)
//                    {
//                        farthest_scan_Y_max_L=i;
//                        farthest_scan_X_max_L=j;
//                    }
//                }
//                break;
//            }
//        }
//    }
//    //确定横向扫描基准点
//    
//  
//      if(ring_flag==1)//圆环
//    {
//        farthest_scan_Y_max=farthest_scan_Y_max_R;
//        farthest_scan_X_max=farthest_scan_X_max_R;
//    }
//    else if(ring_flag==2)//圆环
//    {
//        farthest_scan_Y_max=farthest_scan_Y_max_L;
//        farthest_scan_X_max=farthest_scan_X_max_L;
//    }
//      else//不在圆环取最远
//   {
//        if(farthest_scan_Y_max_R<farthest_scan_Y_max_L)
//        {
//            farthest_scan_Y_max=farthest_scan_Y_max_R;
//            farthest_scan_X_max=farthest_scan_X_max_R;
//        }
//        else
//        {
//            farthest_scan_Y_max=farthest_scan_Y_max_L;
//            farthest_scan_X_max=farthest_scan_X_max_L;
//        }
//   }
//   
//
////    uart_putchar(uart0,farthest_scan_X_max/10+'0');
////            uart_putchar(uart0,farthest_scan_X_max%10+'0');
////            uart_putchar(uart0,0);
//   //printf("最远准点：%d\n",farthest_scan_Y_max);
//        //判断分叉路口
//   /* if((farthest_scan_X_max_R-40)>5&&(39-farthest_scan_X_max_L)>5)
//    {
//        anticipation_ring_flag=1;
//       // a_ring_flag=i;
//    }*/
// 
//    //判断最远处
//    if(farthest_scan_Y_max>=55)//最远处大于等于55，这判断图像无效，全黑
//    {
//      
//        if(middle_Line_last>39)
//        {
//            middle_Line=middle_Line_last+1;
//        }
//        else
//        {
//            middle_Line=middle_Line_last-1;
//        }
//        stop_line_check=1;
//        middle_Line_last=middle_Line;
//        return ;
//    }
//   // printf("扫描基准点：%d\n",portrait_scan_X_max);
//    //横向扫描基准线    
//    for(i=59;i>=55;i--,i--)
//    {  
//        R_flag=1;       //默认找不到线
//        L_flag=1;       //默认找不到线
//        Img_R[i]=picture_wide*8-1;
//        Img_L[i]=0;
//        
//        j=farthest_scan_X_max/8;
//        
//        //判断边线是否在横向扫描基准点的字节内 
//        if((imgbuff[i*picture_wide+j]&((0xff>>((farthest_scan_X_max%8))+1)&0xff))!=0)
//        {
//            for(k=7-(farthest_scan_X_max%8);k>=0;k--)
//            {
//              //白点&&黑点&&+3<(i+1)*picture&&+3黑点
//              if(
//                 (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //白色
//                 &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//找到右线                   //黑色
//                {
//                    if((j*8+7-k)<77)
//                    {
//                        Img_R[i]=j*8+7-k;
//                        if(R_back>Img_R[i])
//                            R_back=Img_R[i];
//                        R_flag=0;                        
//                        goto Right_first_line_finish;
//                    }
//                }
//            }
//        }
//        else
//        {
//            for(j=j+1;j<picture_wide;j++)//右边
//            {
//                if(imgbuff[i*picture_wide+j]!=0)
//                {
//                    for(k=7;k>=0;k--)
//                    {
//                      //白点&&黑点&&+3<(i+1)*picture&&+3黑点
//                      if(
//                         (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //白色
//                         &&(imgbuff[i*picture_wide+j]&(1<<k))!=0 )//找到右线                   //黑色
//                        {
//                            if((j*8+7-k)<77)
//                            {
//                                Img_R[i]=j*8+7-k;
//                                if(R_back>Img_R[i])
//                                    R_back=Img_R[i];
//                                R_flag=0;
//                                goto Right_first_line_finish;
//                            }
//                        }
//                    }
//                }
//            }
//        }        
//        Right_first_line_finish:
//           
//        j=farthest_scan_X_max/8;
//        //判断边线是否在横向扫描基准点的字节内 
//        if((imgbuff[i*picture_wide+j]&((0xff<<(8-(farthest_scan_X_max%8)))&0xff))!=0)
//        {
//            for(k=7-(farthest_scan_X_max%8);k<8;k++)
//            {
//                if(
//                   (((k-1)>=0)
//                    ?(imgbuff[i*picture_wide+j]&(1<<(k-1)))==0
//                    :(imgbuff[i*picture_wide+j+1]&(1<<(k-1+8)))==0)
//                   &&(imgbuff[i*picture_wide+j]&(1<<k))!=0
//                   )
//                {
//                    if((j*8+7-k)>2)
//                    {
//                        Img_L[i]=j*8+7-k;
//                        if(L_back<Img_L[i])
//                            L_back=Img_L[i];
//                        L_flag=0;
//                        goto Left_first_line_finish;
//                    }
//                }
//            }
//        }
//        else
//        {
//            for(j=j-1;j>=0;j--)//左边
//            {
//                if(imgbuff[i*picture_wide+j]!=0)
//                {
//                    for(k=0;k<8;k++)
//                    {
//                        if(
//                           (((k-1)>=0)
//                            ?(imgbuff[i*picture_wide+j]&(1<<(k-1)))==0
//                            :(imgbuff[i*picture_wide+j+1]&(1<<(k-1+8)))==0)
//                           &&(imgbuff[i*picture_wide+j]&(1<<k))!=0
//                           )
//                        {
//                            if((j*8+7-k)>2)
//                            {
//                                Img_L[i]=j*8+7-k;
//                                if(L_back<Img_L[i])
//                                    L_back=Img_L[i];
//                                L_flag=0;
//                                goto Left_first_line_finish;
//                            }
//                        }
//                    }
//                }
//            }
//        }     
//        Left_first_line_finish:  
//          if(i==55)
//          if((Img_R[i]-Img_L[i])>=(Img_R[i+2]-Img_L[i+2])) 
//            {
//                if(R_convergence_flag==0&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4])     //收敛条件
//                {
//                    R_convergence_flag=i+2;     //右收敛标志
//      //             printf("右收敛标志:%d\n",R_convergence_flag);
//                }
//                if(L_convergence_flag==0&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4])     //收敛条件
//                {
//                    L_convergence_flag=i+2;     //左收敛标志
//       //           printf("左收敛标志:%d\n",L_convergence_flag);
//                }
//                if(R_convergence_flag!=0&&R_ce==0&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4])     //收敛条件
//                {
//                    R_ce=i+2;     //右收敛标志
//      //             printf("右收敛标志:%d\n",R_convergence_flag);
//                }
//                if(L_convergence_flag!=0&&L_ce==0&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4])     //收敛条件
//                {
//                    L_convergence_flag=i+2;     //左收敛标志
//       //           printf("左收敛标志:%d\n",L_convergence_flag);
//                }
//            }
//          //一行扫描结束后存储中线值
//        middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//        Line_wide=Img_R[i]-Img_L[i];    
//        
//        //中线显示        
//#if (display_original_line==1)
//        imgbuff[i*picture_wide+(int)middle_Line_temp[i]/8]|=1<<7-(int)middle_Line_temp[i]%8;       //现实中线
//        //行数*行宽+该行中线所在组，进行或运算将该点置1
//        imgbuff[i*picture_wide+picture_wide/2]=imgbuff[i*picture_wide+picture_wide/2]|(1<<7);                 //摄像头理论中线
//#endif
//    }
//    
//    
//    
//    
//    
//    
//    
//    
//    
//    
//    
//    
//    for(i=53;i>=farthest_scan_Y_max+1;i--,i--)
//    {
//        R_flag=1;       //默认找不到线
//        L_flag=1;       //默认找不到线
//        Img_R[i]=picture_wide*8-1;    
//        j=farthest_scan_X_max/8;
//        if((imgbuff[i*picture_wide+j]&((0xff>>((farthest_scan_X_max%8))+1)&0xff))!=0)
//        {
//            for(k=7-(farthest_scan_X_max%8);k>=0;k--)
//            {
//              //白点&&黑点&&+3<(i+1)*picture&&+3黑点
//                if(
//                    (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //白色
//                    &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//找到右线                   //黑色         
//                {
//                  if((j*8+7-k)<=Img_R[i+2]+2&&(j*8+7-k)<77)
//                  {
//                    Img_R[i]=j*8+7-k;
//                    if(R_back>Img_R[i])
//                        R_back=Img_R[i];
//                    R_flag=0;
//                    goto Right_line_finish;
//                  }
//
//                }
//            }
//        }
//        else
//        {
//            for(j=farthest_scan_X_max/8;j<picture_wide;j++)//右边
//            {
//                if(imgbuff[i*picture_wide+j]!=0)
//                {
//                    for(k=7;k>=0;k--)
//                    {
//                      //白点&&黑点&&+3<(i+1)*picture&&+3黑点
//                      if(
//                         (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //白色
//                         &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//找到右线                   //黑色
//                        {
//                          if((j*8+7-k)<=Img_R[i+2]+2&&(j*8+7-k)<77)
//                          {
//                            Img_R[i]=j*8+7-k;
//                            if(R_back>Img_R[i])
//                                R_back=Img_R[i];
//                            R_flag=0;
//                            goto Right_line_finish;
//                          }
//
//                        }
//                    }
//                }
//            }
//        }
//          
//        Right_line_finish:
//        j=farthest_scan_X_max/8;
//        if(
//           (imgbuff[i*picture_wide+j]&((0xff<<(8-(farthest_scan_X_max%8)))&0xff))!=0
//           )
//        {
//            for(k=7-(farthest_scan_X_max%8);k<8;k++)
//            {
//                if(
//                   (((k-1)>=0)
//                    ?(imgbuff[i*picture_wide+j]&(1<<(k-1)))==0
//                    :(imgbuff[i*picture_wide+j+1]&(1<<(k-1+8)))==0)
//                   &&(imgbuff[i*picture_wide+j]&(1<<k))!=0
//                   )
//                {
//                  if((j*8+7-k)>=Img_L[i+2]-2&&(j*8+7-k)>2)
//                  {
//                    Img_L[i]=j*8+7-k;
//                    if(L_back<Img_L[i])
//                        L_back=Img_L[i];
//                    L_flag=0;
//                    goto Left_line_finish;
//                  }
//                }
//            }
//        }
//        else
//        {
//            for(j=j-1;j>=0;j--)//左边
//            {
//                if(imgbuff[i*picture_wide+j]!=0)
//                {
//                    for(k=0;k<8;k++)
//                    {
//                        if(
//                           (((k-1)>=0)
//                            ?(imgbuff[i*picture_wide+j]&(1<<(k-1)))==0
//                            :(imgbuff[i*picture_wide+j+1]&(1<<(k-1+8)))==0)
//                           &&(imgbuff[i*picture_wide+j]&(1<<k))!=0
//                           )
//                        {
//                          if((j*8+7-k)>=Img_L[i+2]-2&&(j*8+7-k)>2)
//                          {
//                            Img_L[i]=j*8+7-k;
//                            if(L_back<Img_L[i])
//                                L_back=Img_L[i];
//                            L_flag=0;
//                            goto Left_line_finish;
//                          }
//                        }
//                    }
//                }
//            }
//        }
//            
//        Left_line_finish:
//          
//          
//             
//          //跳变点判断
//        if((Img_L[i]-Img_L[i+2])>10&&L_mutation_shrink_flag==0)
//        {
//            L_mutation_shrink_flag=i;
//                L_shrink_flag[i]=Img_L[i];
//                     uart_putchar(uart0,'L');
//                     uart_putchar(uart0,'S');
//                     uart_putchar(uart0,'=');
//                     uart_putchar(uart0,L_mutation_shrink_flag/10+'0');
//                     uart_putchar(uart0,L_mutation_shrink_flag%10+'0');
//                     uart_putchar(uart0,0);
//      //     printf("左缩：%d\n",L_mutation_shrink_flag);
//      //     printf("左缩j：%d\n",L_shrink_flag[i]);
//        }
//        
//        if((Img_R[i+2]-Img_R[i])>10&&R_mutation_shrink_flag==0)
//        {
//            R_mutation_shrink_flag=i;
////                     uart_putchar(uart0,'R');
////                     uart_putchar(uart0,'S');
////                     uart_putchar(uart0,'=');
////                     uart_putchar(uart0,R_mutation_shrink_flag/10+'0');
////                     uart_putchar(uart0,R_mutation_shrink_flag%10+'0');
////                     uart_putchar(uart0,0);
//            R_shrink_flag[i]=Img_R[i];
//       //  printf("右缩：%d\n",R_mutation_shrink_flag);
//        }
//          
//           if((Img_L[i+2]-Img_L[i])>8&&L_mutation_wide_flag==0)
//        {
//            L_mutation_wide_flag=i+2;
//            L_wide_flag[i+2]=Img_L[i+2];
//      //     printf("左宽：%d\n",L_mutation_wide_flag);
//        }
//        
//        if((Img_R[i]-Img_R[i+2])>10&&R_mutation_wide_flag==0)
//        {
//            R_mutation_wide_flag=i+2;
//            R_wide_flag[i+2]=Img_R[i+2];
//                 
//                     
//
//        }             
//          
//          
//           if((Img_R[i]-Img_L[i])>=(Img_R[i+2]-Img_L[i+2])) 
//            {
//                if(R_convergence_flag==0&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4]&&Img_R[i+4]<Img_R[i+6])     //收敛条件
//                {
//                    R_convergence_flag=i+2;     //右收敛标志
//                    
//                  
//      //             printf("右收敛标志:%d\n",R_convergence_flag);
//                }
//                
//                
//                if(L_convergence_flag==0&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4]&&Img_L[i+4]>Img_L[i+6])     //收敛条件
//                {
//                    L_convergence_flag=i+2;     //左收敛标志
//                    
//       //           printf("左收敛标志:%d\n",L_convergence_flag);
//                }
//                
//                
//               if(R_convergence_flag!=0&&R_ce==0&&R_convergence_flag!=i+2&&R_convergence_flag-i-2>10&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4])     //收敛条件
//                {
//                    R_ce=i+2;     //右收敛标志
//                    
//                
//      //             printf("右收敛标志:%d\n",R_convergence_flag);
//                }
//                if(L_convergence_flag!=0&&L_ce==0&&L_convergence_flag!=i+2&&L_convergence_flag-i-2>10&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4])     //收敛条件
//                {
//                    L_ce=i+2;     //左收敛标志
//                 
//       //           printf("左收敛标志:%d\n",L_convergence_flag);
//                }
//            }
//
//        if(i<=47) 
//        {
//        //右丢线开始
//        if(R_flag==1&&R_lose_line_front_flag==0)
//        {
//            R_lose_line_front_flag=i;
//         //printf("右丢线开始：%d\n",R_lose_line_front_flag);
//        }
//        //右丢线结束
//        if(R_lose_line_front_flag!=0&&R_flag==0&&R_lose_line_back_flag==0)
//        {
//            R_lose_line_back_flag=i;
//      //   printf("右丢线结束：%d\n",R_lose_line_back_flag);
//        }
//        //左丢线开始
//        if(L_flag==1&&L_lose_line_front_flag==0)
//        {
//            L_lose_line_front_flag=i;
//             
//       //      printf("左丢线开始：%d\n",L_lose_line_front_flag);
//        }
//        //左丢线结束
//        if(L_lose_line_front_flag!=0&&L_flag==0&&L_lose_line_back_flag==0)
//        {
//            L_lose_line_back_flag=i;
//            
//
//     //       printf("左丢线结束：%d\n",L_lose_line_back_flag);
//        }
//        }
//        //简单丢线处理
//        if(L_flag==1&&R_flag==0)
//            middle_Line_temp[i]=(Img_R[i]-(float)Line_wide/2);
//        else if(L_flag==0&&R_flag==1)
//            middle_Line_temp[i]=(Img_L[i]+(float)Line_wide/2);
//        else if(L_flag==1&&R_flag==1)           
//            middle_Line_temp[i]= middle_Line_temp[i+2];           
//        else
//        {
//            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//            Line_wide=Img_R[i]-Img_L[i];
//        }
//
//        //中线显示        
//#if (display_original_line==1)
//        imgbuff[i*picture_wide+(int)middle_Line_temp[i]/8]|=1<<7-(int)middle_Line_temp[i]%8;       //现实中线
//        imgbuff[i*picture_wide+picture_wide/2]=imgbuff[i*picture_wide+picture_wide/2]|(1<<7);                 //摄像头理论中线
//#endif            
//    }
//    //最远行（基数）
//    farthest_scan_Y_max=i+2;    
//    
//    
//             
//  //*********************左右环前后部分标志位********************//
//                    
//if(ring_front==0) 
//{
//   if(add_line_flag==1&&L_convergence_flag==0&&L_mutation_wide_flag==0&&L_mutation_shrink_flag>25)
//   {
//        add_line_flag=2;
//   }
//    if(add_line_flag==0&&L_ce!=0&&R_mutation_wide_flag==0&&R_mutation_shrink_flag==0)
//    {
//        add_line_flag=1;
//        add_line_flag_R=0;
//    }
//}
//     
//if(ling_front==0) 
// {
//     if(add_line_flag_R==1&&R_convergence_flag==0&&R_mutation_wide_flag==0&&R_mutation_shrink_flag>25)
//   {
//        add_line_flag_R=2;
//   }
//    if(add_line_flag_R==0&&R_ce!=0&&L_mutation_wide_flag==0&&L_mutation_shrink_flag==0)
//    {
//        add_line_flag_R=1;
//        add_line_flag=0;
//    }
// }   
//
//    
// //*********************左右出环前标志********************//
//             
//   
//if(ring_flag==1&&(L_convergence_flag==35||L_convergence_flag==37))
//{
//ling_front=1;       //出环标志位
//ring_flag=0;        //进环标志位
//add_line_flag_R=0;    //补线标志位
//}
//
//if(ring_flag==2&&(R_convergence_flag==35||R_convergence_flag==37))
//{
//ring_front=1;       //出环标志位
//ring_flag=0;        //进环标志位
//add_line_flag=0;    //补线标志位
//}
//
//
//       
////********************左右出环后标志位******************//
//
//if(ring_front==1)
//{
// if(R_convergence_flag==0&&R_mutation_wide_flag==0&&L_mutation_shrink_flag>23)
//   {
//        add_line_flag=2;
//   }
//}
//
//
//if(ling_front==1)
//{
// if(L_convergence_flag==0&&L_mutation_wide_flag==0&&R_mutation_shrink_flag>23)
//   {
//        add_line_flag_R=2;
//   }
//}
//
////****************左环清标志位****************//
//if(ring_front==1&&add_line_flag==2&&L_mutation_shrink_flag>=37)
//{
//      ring_front=0;
//      add_line_flag=0;
//      ring_flag=0;
// }
// 
// //****************右环清标志位****************//
//if(ling_front==1&&add_line_flag_R==2&&R_mutation_shrink_flag>=37)
//{
//      ling_front=0;
//      add_line_flag_R=0;
//      ring_flag=0;
// }
//// uart_putchar(uart0,'r');
//// uart_putchar(uart0,ling_front+'0');
//// uart_putchar(uart0,add_line_flag_R+'0');
//// uart_putchar(uart0,0);
//// uart_putchar(uart0,'l');
//// uart_putchar(uart0,ring_front+'0');
//// uart_putchar(uart0,add_line_flag+'0');
//// uart_putchar(uart0,0);
// 
// //***************左出环前部分*****************//
//if(ring_front==1&&add_line_flag!=2)
// {
//            for(i=57;i>farthest_scan_Y_max;i--,i--)
//          {
//            temp_int=(int)(Img_R[59]+(i-59)*(25-Img_R[59])/(farthest_scan_Y_max-59)+0.5);  //线性直插;
//            if(Img_R[i]>temp_int)
//            {
//                Img_R[i]=temp_int;
//            }
//            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//#if (display_ring_supplement_line==1)
//                imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
//                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
//#endif
//            }
//
// }
//
////***************左出环后部分*****************//
//else if(ring_front==1&&add_line_flag==2)
//{
//        for(i=57;i>L_lose_line_back_flag;i--,i--)
//            {
//                temp_int=(int)(Img_L[59]+(i-59)*(Img_L[L_mutation_shrink_flag]-Img_L[59]+15)/(L_mutation_shrink_flag-59)+0.5);  //线性直插;
//                if(Img_L[i]<temp_int)
//                {
//                    Img_L[i]=temp_int;
//                }
//                middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//#if (display_ring_supplement_line==1)
//              imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8; 
//              imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
//#endif
//              }
//}
//
//  
//
// //***************左前部分*****************//
// 
//else if(add_line_flag==1&&ring_front==0)
// {
////                    uart_putchar(uart0,'O');
////                    uart_putchar(uart0,'K');
//                  
//     
//          for(i=57;i>L_lose_line_back_flag;i--,i--)
//            {
//                temp_int=(int)(Img_L[59]+(i-59)*(Img_L[L_mutation_wide_flag]-Img_L[59])/(L_mutation_wide_flag-59)+0.5);  //线性直插;
//                if(Img_L[i]<temp_int)
//                {
//                    Img_L[i]=temp_int;
//                }
//                middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//#if (display_ring_supplement_line==1)
//              imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8; 
//              imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
//#endif
//              }
//}
//
// //***************左后部分*****************//
//
//else if(add_line_flag==2&&ring_front==0)
//{
//                    
////                    uart_putchar(uart0,'E');
////                    uart_putchar(uart0,'R');
////                    uart_putchar(uart0,'R');
////                    uart_putchar(uart0,'O');
////                    uart_putchar(uart0,'R');
//                    ring_flag=2;
//         for(i=57;i>L_mutation_shrink_flag ;i--,i--)
//          {
//            temp_int=(int)(Img_R[59]+(i-59)*(Img_L[L_mutation_shrink_flag]-5-Img_R[59])/(L_mutation_shrink_flag-59)+0.5);  //线性直插;
//            if(Img_R[i]>temp_int)
//            {
//                Img_R[i]=temp_int;
//            }
//            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//#if (display_ring_supplement_line==1)
//                imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8;
//                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
//#endif
//            }
//
// }
//
// 
// //***************右前部分*****************//
// 
//else if(add_line_flag_R==1&&ling_front==0)
// {
////                    uart_putchar(uart0,'O');
////                    uart_putchar(uart0,'K');
//                  
//     
//          for(i=57;i>R_lose_line_back_flag;i--,i--)
//            {
//                temp_int=(int)(Img_R[59]+(i-59)*(Img_R[R_mutation_wide_flag]-Img_R[59])/(R_mutation_wide_flag-59)+0.5);  //线性直插;
//                if(Img_R[i]>temp_int)
//                {
//                    Img_R[i]=temp_int;
//                }
//                middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//#if (display_ring_supplement_line==1)
//              imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8; 
//              imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
//#endif
//              }
//}
//    
//    
//
// //***************右后部分*****************//
//
//else if(add_line_flag_R==2&&ling_front==0)
//{
//                    
////                    uart_putchar(uart0,'E');
////                    uart_putchar(uart0,'R');
////                    uart_putchar(uart0,'R');
////                    uart_putchar(uart0,'O');
////                    uart_putchar(uart0,'R');
//                    ring_flag=1;
//      for(i=57;i>R_mutation_shrink_flag ;i--,i--)
//          {
//            temp_int=(int)(Img_L[59]+(i-59)*(Img_R[R_mutation_shrink_flag]+5-Img_L[59])/(R_mutation_shrink_flag-59)+0.5);  //线性直插;
//            if(Img_L[i]<temp_int)
//            {
//                Img_L[i]=temp_int;
//            }
//            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//#if (display_ring_supplement_line==1)
//                imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
//                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
//#endif
//            }
//
// }
//
////***************右出环后部分*****************//
//else if(ling_front==1&&add_line_flag_R==2)
//{
//        for(i=57;i>R_lose_line_back_flag;i--,i--)
//            {
//                temp_int=(int)(Img_R[59]+(i-59)*(Img_R[R_mutation_shrink_flag]-Img_R[59]-15)/(R_mutation_shrink_flag-59)+0.5);  //线性直插;
//                if(Img_R[i]>temp_int)
//                {
//                    Img_R[i]=temp_int;
//                }
//                middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//#if (display_ring_supplement_line==1)
//              imgbuff[i*picture_wide+(int) Img_R[i]/8]|=1<<7-(int) Img_R[i]%8; 
//              imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
//#endif
//              }
//}
//
////***************右出环前部分*****************//
//else if(ling_front==1&&add_line_flag_R!=2)
// {
//            for(i=57;i>farthest_scan_Y_max;i--,i--)
//          {
//            temp_int=(int)(Img_L[59]+(i-59)*(54-Img_L[59])/(farthest_scan_Y_max-59)+0.5);  //线性直插;
//            if(Img_L[i]<temp_int)
//            {
//                Img_L[i]=temp_int;
//            }
//            middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//#if (display_ring_supplement_line==1)
//                imgbuff[i*picture_wide+(int) Img_L[i]/8]|=1<<7-(int) Img_L[i]%8;
//                imgbuff[i*picture_wide+((int)middle_Line_temp[i]/8)]|=1<<7-((int)middle_Line_temp[i]%8);
//#endif
//            }
//
// }
//
//    //****************************中线提取*****************************//
//    if(farthest_scan_Y_max<=prospect)
//    {   
//        middle_Line=(middle_Line_temp[55]+middle_Line_temp[57]+middle_Line_temp[53])/3*0.5+(middle_Line_temp[prospect]+middle_Line_temp[prospect+2])/2*0.5;
//    }
//    else
//    {
//        if((farthest_scan_Y_max-prospect)>=4)
//        {
//            middle_Line_temp[prospect]=(int)(middle_Line_temp[59]+(prospect-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);  //线性直插
//            middle_Line_temp[(prospect+2)]=(int)(middle_Line_temp[59]+((prospect+2)-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);
//        }
//        else
//        {
//            middle_Line_temp[prospect]=(int)(middle_Line_temp[59]+(prospect-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);  //线性直插
//        }
//        middle_Line=(middle_Line_temp[55]+middle_Line_temp[57]+middle_Line_temp[53])/3*0.5+(middle_Line_temp[prospect]+middle_Line_temp[prospect+2])/2*0.5;
//
//    }  
//    middle_Line_last=middle_Line;
//}
   

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

int8 prospect=31;//29;//前瞻//为隔行扫描所以前瞻取值应为奇数

uint8 stop_line_check=0;                   //起跑线检测标志，0为关闭起跑线检测 1为开始起跑线检测

uint8 slow_down=0;                    //减速标记

uint8  curve_flag=0;                              //出环标记

uint8  R_curve_flag=0;                               //右弯标志位         
uint8  L_curve_flag=0;                               //左弯标志位

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
    
    R_curve_flag=0;                               //右弯标志位         
    L_curve_flag=0;                               //左弯标志位
    
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
    
    static uint8 ring_flag=0;                              //默认不进入圆环//0不进入圆环 1右 2左
    
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

        if(Img_R[i]>Img_R[i+2]&&Img_R[i+2]>Img_R[i+4]&&R_curve_flag==0&&i>25)//i>25
        {
          R_curve_flag=1;
       //    printf("右弯");
        }
        if(Img_L[i]<Img_L[i+2]&&Img_L[i+2]<Img_L[i+4]&&L_curve_flag==0&&i>25)//i>25
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

