//#include "common.h"
//#include "headfile.h"
//
//#define picture_wide    (OV7725_W/8)
//#define picture_high    OV7725_H
//
//#define imgbuff  image_bin
//
////ͼ������ȡ����
//#define display_original_line                   1       //0�ر�ԭʼ���ߡ��������ߣ�1��ԭʼ���ߡ��������ߣ�
//#define display_cross_supplement_line           1       //0�ر�ʮ�ֲ��ߣ�1��ʮ�ֲ��ߡ�
//#define display_ring_supplement_line            1       //0�ر�Բ�����ߣ�1��Բ�����ߡ�
//#define display_Obstacle_supplement_line        0       //0�ر��ϰ����ߣ�1���ϰ����ߡ�
//
//#define picture_wide    (OV7725_W/8)
//#define picture_high    OV7725_H
//
//int ring_front=0;
//int ling_front=0;
//
//int middle_Line;                      //����ֵ
//int Line_wide;                          //�߿��
//
//uint8  farthest_scan_Y_max;                //����ɨ����Զ�����21
//uint8  farthest_scan_X_max;              //����ɨ����Զ�����  
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
//int8 prospect=33;//ǰհ
//
//uint8 stop_line_check=0;                   //�����߼���־��0Ϊ�ر������߼�� 1Ϊ��ʼ�����߼��
//
//uint8 slow_down=0;                    //���ٱ��
//
//uint8  curve_flag=0;                              //�������
//
//
//int L,R;
//
//float middle_Line_temp[60]={0};                       //3�����ߵ�ֵ
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
//    uint8 L_flag=2,R_flag=2;                               //ɨ��ɹ�0��ɨ��ʧ��1,δɨ��2
//    
//    static float middle_Line_last;
//    int i,j,k;                                          //�� ���ֽ� ��λ
//    int aa=39;
//    
//    uint8  L_convergence_flag=0;                           //��������־
//    uint8  R_convergence_flag=0;                           //��������־
//    
//    uint8 R_ce=0;
//    uint8 L_ce=0;
//    
//    
//    uint8  lose_line_front_effective_line_flag=53 ;        //ȫ����ǰ��־λ
//    uint8  lose_line_back_effective_line_flag=53;          //ȫ���ߺ��־λ
//    
//    uint8 lose_line_start=59;                              //ȫ���߿�ʼ��־λ
//    
//    uint8  lose_line_flag=0;                               //ȫ���߱�־λ
//    
//    uint8  R_curve_flag=0;                               //�����־λ         
//    uint8  L_curve_flag=0;                               //�����־λ
//    
//    //uint8  R_slow_downe=0;
//   // uint8  L_slow_downe=0;
//
//    uint8  R_lose_line_front_flag=0;                       //����ǰ��־
//    uint8  R_lose_line_back_flag=0;                        //���ߺ��־
//    
//    uint8  L_lose_line_front_flag=0;                       //����ǰ��־
//    uint8  L_lose_line_back_flag=0;                        //���ߺ��־
//    
//    uint8  R_mutation_shrink_flag=0;                       //���������
//    uint8  L_mutation_shrink_flag=0;                       //���������
//    
//    
//    uint8  R_mutation_wide_flag=0;                         //�ҿ������
//    uint8  L_mutation_wide_flag=0;                         //��������
//    
//   // uint8  anticipation_ring_flag=0;
//    // uint8  a_ring_flag=0;
//    
//    static uint8 Obstacle_avoidance_flag=0;                //���ϱ�־λ 0���ϰ� 1�� 2��
//    
//    static uint8 ring_flag=0;                              //Ĭ�ϲ�����Բ��//0������Բ�� 1�� 2��
//    
//    static uint8 cross_flag=0;                             //Ĭ��û��ʮ��
//      
//    static uint8 L_ring_flag=0;                   
//    static uint8 R_ring_flag=0;                    
//    
//   // static uint8 Straight_Line_flag=0;                     //ֱ�ߴ���       
//    
//    int temp_int=0;                                     //int_��ʱ����
//    
//    int R_back=79;                                      //��Զ��������ֵ
//    int L_back=0;                                       //��Զ��������ֵ    
//    //������ʼ��
//    Img_R[59]=picture_wide*8-1;
//    farthest_scan_Y_max=59;
//    farthest_scan_Y_max_R=59;
//    farthest_scan_Y_max_L=59;
//
//
//    //����ɨ��
//    for(j=5;j<75;j++)
//    {
//        for(i=59;i>0;i--)
//        {
//            if((imgbuff[i*10+j/8]&(1<<(7-j%8)))!=0)
//            { 
//               if(j>aa)    //����Զ
//                  {
//                     if(farthest_scan_Y_max_R>=i)
//                    {
//                        farthest_scan_Y_max_R=i;
//                        farthest_scan_X_max_R=j;
//                    } 
//                  }
//                else        //����Զ
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
//    //ȷ������ɨ���׼��
//    
//  
//      if(ring_flag==1)//Բ��
//    {
//        farthest_scan_Y_max=farthest_scan_Y_max_R;
//        farthest_scan_X_max=farthest_scan_X_max_R;
//    }
//    else if(ring_flag==2)//Բ��
//    {
//        farthest_scan_Y_max=farthest_scan_Y_max_L;
//        farthest_scan_X_max=farthest_scan_X_max_L;
//    }
//      else//����Բ��ȡ��Զ
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
//   //printf("��Զ׼�㣺%d\n",farthest_scan_Y_max);
//        //�жϷֲ�·��
//   /* if((farthest_scan_X_max_R-40)>5&&(39-farthest_scan_X_max_L)>5)
//    {
//        anticipation_ring_flag=1;
//       // a_ring_flag=i;
//    }*/
// 
//    //�ж���Զ��
//    if(farthest_scan_Y_max>=55)//��Զ�����ڵ���55�����ж�ͼ����Ч��ȫ��
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
//   // printf("ɨ���׼�㣺%d\n",portrait_scan_X_max);
//    //����ɨ���׼��    
//    for(i=59;i>=55;i--,i--)
//    {  
//        R_flag=1;       //Ĭ���Ҳ�����
//        L_flag=1;       //Ĭ���Ҳ�����
//        Img_R[i]=picture_wide*8-1;
//        Img_L[i]=0;
//        
//        j=farthest_scan_X_max/8;
//        
//        //�жϱ����Ƿ��ں���ɨ���׼����ֽ��� 
//        if((imgbuff[i*picture_wide+j]&((0xff>>((farthest_scan_X_max%8))+1)&0xff))!=0)
//        {
//            for(k=7-(farthest_scan_X_max%8);k>=0;k--)
//            {
//              //�׵�&&�ڵ�&&+3<(i+1)*picture&&+3�ڵ�
//              if(
//                 (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //��ɫ
//                 &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//�ҵ�����                   //��ɫ
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
//            for(j=j+1;j<picture_wide;j++)//�ұ�
//            {
//                if(imgbuff[i*picture_wide+j]!=0)
//                {
//                    for(k=7;k>=0;k--)
//                    {
//                      //�׵�&&�ڵ�&&+3<(i+1)*picture&&+3�ڵ�
//                      if(
//                         (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //��ɫ
//                         &&(imgbuff[i*picture_wide+j]&(1<<k))!=0 )//�ҵ�����                   //��ɫ
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
//        //�жϱ����Ƿ��ں���ɨ���׼����ֽ��� 
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
//            for(j=j-1;j>=0;j--)//���
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
//                if(R_convergence_flag==0&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4])     //��������
//                {
//                    R_convergence_flag=i+2;     //��������־
//      //             printf("��������־:%d\n",R_convergence_flag);
//                }
//                if(L_convergence_flag==0&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4])     //��������
//                {
//                    L_convergence_flag=i+2;     //��������־
//       //           printf("��������־:%d\n",L_convergence_flag);
//                }
//                if(R_convergence_flag!=0&&R_ce==0&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4])     //��������
//                {
//                    R_ce=i+2;     //��������־
//      //             printf("��������־:%d\n",R_convergence_flag);
//                }
//                if(L_convergence_flag!=0&&L_ce==0&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4])     //��������
//                {
//                    L_convergence_flag=i+2;     //��������־
//       //           printf("��������־:%d\n",L_convergence_flag);
//                }
//            }
//          //һ��ɨ�������洢����ֵ
//        middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
//        Line_wide=Img_R[i]-Img_L[i];    
//        
//        //������ʾ        
//#if (display_original_line==1)
//        imgbuff[i*picture_wide+(int)middle_Line_temp[i]/8]|=1<<7-(int)middle_Line_temp[i]%8;       //��ʵ����
//        //����*�п�+�������������飬���л����㽫�õ���1
//        imgbuff[i*picture_wide+picture_wide/2]=imgbuff[i*picture_wide+picture_wide/2]|(1<<7);                 //����ͷ��������
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
//        R_flag=1;       //Ĭ���Ҳ�����
//        L_flag=1;       //Ĭ���Ҳ�����
//        Img_R[i]=picture_wide*8-1;    
//        j=farthest_scan_X_max/8;
//        if((imgbuff[i*picture_wide+j]&((0xff>>((farthest_scan_X_max%8))+1)&0xff))!=0)
//        {
//            for(k=7-(farthest_scan_X_max%8);k>=0;k--)
//            {
//              //�׵�&&�ڵ�&&+3<(i+1)*picture&&+3�ڵ�
//                if(
//                    (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //��ɫ
//                    &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//�ҵ�����                   //��ɫ         
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
//            for(j=farthest_scan_X_max/8;j<picture_wide;j++)//�ұ�
//            {
//                if(imgbuff[i*picture_wide+j]!=0)
//                {
//                    for(k=7;k>=0;k--)
//                    {
//                      //�׵�&&�ڵ�&&+3<(i+1)*picture&&+3�ڵ�
//                      if(
//                         (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //��ɫ
//                         &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//�ҵ�����                   //��ɫ
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
//            for(j=j-1;j>=0;j--)//���
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
//          //������ж�
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
//      //     printf("������%d\n",L_mutation_shrink_flag);
//      //     printf("����j��%d\n",L_shrink_flag[i]);
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
//       //  printf("������%d\n",R_mutation_shrink_flag);
//        }
//          
//           if((Img_L[i+2]-Img_L[i])>8&&L_mutation_wide_flag==0)
//        {
//            L_mutation_wide_flag=i+2;
//            L_wide_flag[i+2]=Img_L[i+2];
//      //     printf("���%d\n",L_mutation_wide_flag);
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
//                if(R_convergence_flag==0&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4]&&Img_R[i+4]<Img_R[i+6])     //��������
//                {
//                    R_convergence_flag=i+2;     //��������־
//                    
//                  
//      //             printf("��������־:%d\n",R_convergence_flag);
//                }
//                
//                
//                if(L_convergence_flag==0&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4]&&Img_L[i+4]>Img_L[i+6])     //��������
//                {
//                    L_convergence_flag=i+2;     //��������־
//                    
//       //           printf("��������־:%d\n",L_convergence_flag);
//                }
//                
//                
//               if(R_convergence_flag!=0&&R_ce==0&&R_convergence_flag!=i+2&&R_convergence_flag-i-2>10&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4])     //��������
//                {
//                    R_ce=i+2;     //��������־
//                    
//                
//      //             printf("��������־:%d\n",R_convergence_flag);
//                }
//                if(L_convergence_flag!=0&&L_ce==0&&L_convergence_flag!=i+2&&L_convergence_flag-i-2>10&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4])     //��������
//                {
//                    L_ce=i+2;     //��������־
//                 
//       //           printf("��������־:%d\n",L_convergence_flag);
//                }
//            }
//
//        if(i<=47) 
//        {
//        //�Ҷ��߿�ʼ
//        if(R_flag==1&&R_lose_line_front_flag==0)
//        {
//            R_lose_line_front_flag=i;
//         //printf("�Ҷ��߿�ʼ��%d\n",R_lose_line_front_flag);
//        }
//        //�Ҷ��߽���
//        if(R_lose_line_front_flag!=0&&R_flag==0&&R_lose_line_back_flag==0)
//        {
//            R_lose_line_back_flag=i;
//      //   printf("�Ҷ��߽�����%d\n",R_lose_line_back_flag);
//        }
//        //���߿�ʼ
//        if(L_flag==1&&L_lose_line_front_flag==0)
//        {
//            L_lose_line_front_flag=i;
//             
//       //      printf("���߿�ʼ��%d\n",L_lose_line_front_flag);
//        }
//        //���߽���
//        if(L_lose_line_front_flag!=0&&L_flag==0&&L_lose_line_back_flag==0)
//        {
//            L_lose_line_back_flag=i;
//            
//
//     //       printf("���߽�����%d\n",L_lose_line_back_flag);
//        }
//        }
//        //�򵥶��ߴ���
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
//        //������ʾ        
//#if (display_original_line==1)
//        imgbuff[i*picture_wide+(int)middle_Line_temp[i]/8]|=1<<7-(int)middle_Line_temp[i]%8;       //��ʵ����
//        imgbuff[i*picture_wide+picture_wide/2]=imgbuff[i*picture_wide+picture_wide/2]|(1<<7);                 //����ͷ��������
//#endif            
//    }
//    //��Զ�У�������
//    farthest_scan_Y_max=i+2;    
//    
//    
//             
//  //*********************���һ�ǰ�󲿷ֱ�־λ********************//
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
// //*********************���ҳ���ǰ��־********************//
//             
//   
//if(ring_flag==1&&(L_convergence_flag==35||L_convergence_flag==37))
//{
//ling_front=1;       //������־λ
//ring_flag=0;        //������־λ
//add_line_flag_R=0;    //���߱�־λ
//}
//
//if(ring_flag==2&&(R_convergence_flag==35||R_convergence_flag==37))
//{
//ring_front=1;       //������־λ
//ring_flag=0;        //������־λ
//add_line_flag=0;    //���߱�־λ
//}
//
//
//       
////********************���ҳ������־λ******************//
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
////****************�����־λ****************//
//if(ring_front==1&&add_line_flag==2&&L_mutation_shrink_flag>=37)
//{
//      ring_front=0;
//      add_line_flag=0;
//      ring_flag=0;
// }
// 
// //****************�һ����־λ****************//
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
// //***************�����ǰ����*****************//
//if(ring_front==1&&add_line_flag!=2)
// {
//            for(i=57;i>farthest_scan_Y_max;i--,i--)
//          {
//            temp_int=(int)(Img_R[59]+(i-59)*(25-Img_R[59])/(farthest_scan_Y_max-59)+0.5);  //����ֱ��;
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
////***************������󲿷�*****************//
//else if(ring_front==1&&add_line_flag==2)
//{
//        for(i=57;i>L_lose_line_back_flag;i--,i--)
//            {
//                temp_int=(int)(Img_L[59]+(i-59)*(Img_L[L_mutation_shrink_flag]-Img_L[59]+15)/(L_mutation_shrink_flag-59)+0.5);  //����ֱ��;
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
// //***************��ǰ����*****************//
// 
//else if(add_line_flag==1&&ring_front==0)
// {
////                    uart_putchar(uart0,'O');
////                    uart_putchar(uart0,'K');
//                  
//     
//          for(i=57;i>L_lose_line_back_flag;i--,i--)
//            {
//                temp_int=(int)(Img_L[59]+(i-59)*(Img_L[L_mutation_wide_flag]-Img_L[59])/(L_mutation_wide_flag-59)+0.5);  //����ֱ��;
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
// //***************��󲿷�*****************//
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
//            temp_int=(int)(Img_R[59]+(i-59)*(Img_L[L_mutation_shrink_flag]-5-Img_R[59])/(L_mutation_shrink_flag-59)+0.5);  //����ֱ��;
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
// //***************��ǰ����*****************//
// 
//else if(add_line_flag_R==1&&ling_front==0)
// {
////                    uart_putchar(uart0,'O');
////                    uart_putchar(uart0,'K');
//                  
//     
//          for(i=57;i>R_lose_line_back_flag;i--,i--)
//            {
//                temp_int=(int)(Img_R[59]+(i-59)*(Img_R[R_mutation_wide_flag]-Img_R[59])/(R_mutation_wide_flag-59)+0.5);  //����ֱ��;
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
// //***************�Һ󲿷�*****************//
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
//            temp_int=(int)(Img_L[59]+(i-59)*(Img_R[R_mutation_shrink_flag]+5-Img_L[59])/(R_mutation_shrink_flag-59)+0.5);  //����ֱ��;
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
////***************�ҳ����󲿷�*****************//
//else if(ling_front==1&&add_line_flag_R==2)
//{
//        for(i=57;i>R_lose_line_back_flag;i--,i--)
//            {
//                temp_int=(int)(Img_R[59]+(i-59)*(Img_R[R_mutation_shrink_flag]-Img_R[59]-15)/(R_mutation_shrink_flag-59)+0.5);  //����ֱ��;
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
////***************�ҳ���ǰ����*****************//
//else if(ling_front==1&&add_line_flag_R!=2)
// {
//            for(i=57;i>farthest_scan_Y_max;i--,i--)
//          {
//            temp_int=(int)(Img_L[59]+(i-59)*(54-Img_L[59])/(farthest_scan_Y_max-59)+0.5);  //����ֱ��;
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
//    //****************************������ȡ*****************************//
//    if(farthest_scan_Y_max<=prospect)
//    {   
//        middle_Line=(middle_Line_temp[55]+middle_Line_temp[57]+middle_Line_temp[53])/3*0.5+(middle_Line_temp[prospect]+middle_Line_temp[prospect+2])/2*0.5;
//    }
//    else
//    {
//        if((farthest_scan_Y_max-prospect)>=4)
//        {
//            middle_Line_temp[prospect]=(int)(middle_Line_temp[59]+(prospect-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);  //����ֱ��
//            middle_Line_temp[(prospect+2)]=(int)(middle_Line_temp[59]+((prospect+2)-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);
//        }
//        else
//        {
//            middle_Line_temp[prospect]=(int)(middle_Line_temp[59]+(prospect-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);  //����ֱ��
//        }
//        middle_Line=(middle_Line_temp[55]+middle_Line_temp[57]+middle_Line_temp[53])/3*0.5+(middle_Line_temp[prospect]+middle_Line_temp[prospect+2])/2*0.5;
//
//    }  
//    middle_Line_last=middle_Line;
//}
   

#include "headfile.h"
#include "common.h"


//ͼ������ȡ����
#define display_original_line                   1       //0�ر�ԭʼ���ߡ��������ߣ�1��ԭʼ���ߡ��������ߣ�
#define display_cross_supplement_line           0       //0�ر�ʮ�ֲ��ߣ�1��ʮ�ֲ��ߡ�
#define display_ring_supplement_line            0       //0�ر�Բ�����ߣ�1��Բ�����ߡ�
#define display_Obstacle_supplement_line        0       //0�ر��ϰ����ߣ�1���ϰ����ߡ�

#define picture_wide    (OV7725_W/8)
#define picture_high    OV7725_H

#define imgbuff  image_bin

int middle_Line;                      //����ֵ
int Line_wide;                        //�߿��

uint8  farthest_scan_Y_max;                //����ɨ����Զ�����21
uint8  farthest_scan_X_max;              //����ɨ����Զ�����  

uint8  farthest_scan_Y_max_R;
uint8  farthest_scan_X_max_R;

uint8  farthest_scan_Y_max_L;
uint8  farthest_scan_X_max_L;

uint8 starting_line;

int8 prospect=31;//29;//ǰհ//Ϊ����ɨ������ǰհȡֵӦΪ����

uint8 stop_line_check=0;                   //�����߼���־��0Ϊ�ر������߼�� 1Ϊ��ʼ�����߼��

uint8 slow_down=0;                    //���ٱ��

uint8  curve_flag=0;                              //�������

uint8  R_curve_flag=0;                               //�����־λ         
uint8  L_curve_flag=0;                               //�����־λ

int L,R;

float middle_Line_temp[60]={0};                       //3�����ߵ�ֵ
int R_shrink_flag[60]={44};
int L_shrink_flag[60]={36};

void find_middle_line()
{
    int Img_L[picture_high]={0};
    int Img_R[picture_high]={0};

    uint8 L_flag=2,R_flag=2;                               //ɨ��ɹ�0��ɨ��ʧ��1,δɨ��2
    
    static float middle_Line_last;
    int i,j,k;                                          //�� ���ֽ� ��λ
    int aa;
    
    uint8  L_convergence_flag=0;                           //��������־
    uint8  R_convergence_flag=0;                           //��������־
    
    
    uint8  lose_line_front_effective_line_flag=53 ;        //ȫ����ǰ��־λ
    uint8  lose_line_back_effective_line_flag=53;          //ȫ���ߺ��־λ
    
    uint8 lose_line_start=59;                              //ȫ���߿�ʼ��־λ
    
    uint8  lose_line_flag=0;                               //ȫ���߱�־λ
    
    R_curve_flag=0;                               //�����־λ         
    L_curve_flag=0;                               //�����־λ
    
    //uint8  R_slow_downe=0;
   // uint8  L_slow_downe=0;

    uint8  R_lose_line_front_flag=0;                       //����ǰ��־
    uint8  R_lose_line_back_flag=0;                        //���ߺ��־
    
    uint8  L_lose_line_front_flag=0;                       //����ǰ��־
    uint8  L_lose_line_back_flag=0;                        //���ߺ��־
    
    uint8  R_mutation_shrink_flag=0;                       //���������
    uint8  L_mutation_shrink_flag=0;                       //���������
    
    
    uint8  R_mutation_wide_flag=0;                         //�ҿ������
    uint8  L_mutation_wide_flag=0;                         //��������
    
   // uint8  anticipation_ring_flag=0;
    // uint8  a_ring_flag=0;
    
    static uint8 Obstacle_avoidance_flag=0;                //���ϱ�־λ 0���ϰ� 1�� 2��
    
    static uint8 ring_flag=0;                              //Ĭ�ϲ�����Բ��//0������Բ�� 1�� 2��
    
    static uint8 cross_flag=0;                             //Ĭ��û��ʮ��
      
    static uint8 L_ring_flag=0;                   
    static uint8 R_ring_flag=0;                    
    
   // static uint8 Straight_Line_flag=0;                     //ֱ�ߴ���       
    
    int temp_int=0;                                     //int_��ʱ����
    
    int R_back=79;                                      //��Զ��������ֵ
    int L_back=0;                                       //��Զ��������ֵ    
    //������ʼ��
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
    
    //����ɨ��
    for(j=5;j<75;j++)
    {
        for(i=59;i>0;i--)//��׼��Ѱ�Ҵ�����㿪ʼѰ��
        {
            if((imgbuff[i*10+j/8]&(1<<(7-j%8)))!=0)
            { 
               if(j>aa)    //����Զ
                  {
                     if(farthest_scan_Y_max_R>=i)
                    {
                        farthest_scan_Y_max_R=i+2;
                        farthest_scan_X_max_R=j;
                    } 
                  }
                else        //����Զ
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
    //ȷ������ɨ���׼��
    if(ring_flag==1)//Բ��
    {
        farthest_scan_Y_max=farthest_scan_Y_max_R;
        farthest_scan_X_max=farthest_scan_X_max_R;
    }
    else if(ring_flag==2)//Բ��
    {
        farthest_scan_Y_max=farthest_scan_Y_max_L;
        farthest_scan_X_max=farthest_scan_X_max_L;
    }
      else//����Բ��ȡ��Զ
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
   //printf("��Զ׼�㣺%d\n",farthest_scan_Y_max);
        //�жϷֲ�·��
   /* if((farthest_scan_X_max_R-40)>5&&(39-farthest_scan_X_max_L)>5)
    {
        anticipation_ring_flag=1;
       // a_ring_flag=i;
    }*/
 
    //�ж���Զ��
    if(farthest_scan_Y_max>=55)//��Զ�����ڵ���55�����ж�ͼ����Ч��ȫ��
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
   // printf("ɨ���׼�㣺%d\n",portrait_scan_X_max);
    //����ɨ���׼��    
    for(i=59;i>=55;i--,i--)
    {  
        R_flag=1;       //Ĭ���Ҳ�����
        L_flag=1;       //Ĭ���Ҳ�����
        Img_R[i]=picture_wide*8-1;
        Img_L[i]=0;
        
        j=farthest_scan_X_max/8;
        
        //�жϱ����Ƿ��ں���ɨ���׼����ֽ��� 
        if((imgbuff[i*picture_wide+j]&((0xff>>((farthest_scan_X_max%8))+1)&0xff))!=0)
        {
            for(k=7-(farthest_scan_X_max%8);k>=0;k--)
            {
              //�׵�&&�ڵ�&&+3<(i+1)*picture&&+3�ڵ�
              if(
                 (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //��ɫ
                 &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//�ҵ�����                   //��ɫ
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
            for(j=j+1;j<picture_wide;j++)//�ұ�
            {
                if(imgbuff[i*picture_wide+j]!=0)
                {
                    for(k=7;k>=0;k--)
                    {
                      //�׵�&&�ڵ�&&+3<(i+1)*picture&&+3�ڵ�
                      if(
                         (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //��ɫ
                         &&(imgbuff[i*picture_wide+j]&(1<<k))!=0 )//�ҵ�����                   //��ɫ
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
        //�жϱ����Ƿ��ں���ɨ���׼����ֽ��� 
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
            for(j=j-1;j>=0;j--)//���
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
          
        //�ж����� 
        if(i==55)   
        {
            if((Img_R[i]-Img_L[i])>=(Img_R[i+2]-Img_L[i+2])) 
            {
                if(R_convergence_flag==0&&Img_R[i]>Img_R[i+2]&&Img_R[i+2]<=Img_R[i+4])     //��������
                {
                    R_convergence_flag=i+2;     //��������־
      //             printf("��������־:%d\n",R_convergence_flag);
                }
                if(L_convergence_flag==0&&Img_L[i]<Img_L[i+2]&&Img_L[i+2]>=Img_L[i+4])     //��������
                {
                    L_convergence_flag=i+2;     //��������־
       //           printf("��������־:%d\n",L_convergence_flag);
                }
            }
            if(R_flag==1&&L_flag==1)
            {
                lose_line_flag=1; 
                lose_line_start=i;
      //          printf("ȫ���߱�־:%d\n",lose_line_start);
            }            
        }
        
        //����
        if(R_flag==1&&R_lose_line_front_flag==0)
        {
            R_lose_line_front_flag=i;
      //       printf("�Ҷ��߿�ʼ��־:%d\n",R_lose_line_front_flag);
        }
        if(L_flag==1&&L_lose_line_front_flag==0)
        {
            L_lose_line_front_flag=i;
      //     printf("���߿�ʼ��־:%d\n",L_lose_line_front_flag);
        }
            
          
        middle_Line_temp[i]=(float)(Img_R[i]+Img_L[i])/2;
        Line_wide=Img_R[i]-Img_L[i];
        
        //������ʾ        
#if (display_original_line==1)
        imgbuff[i*picture_wide+(int)((int)middle_Line_temp[i]/8+0.5)]|=1<<7-(int)((int)middle_Line_temp[i]%8+0.5);       //��ʵ����
        imgbuff[i*picture_wide+picture_wide/2]=imgbuff[i*picture_wide+picture_wide/2]|(1<<7);                 //����ͷ��������
#endif
    }
    for(i=53;i>=farthest_scan_Y_max+1;i--,i--)
    {
        R_flag=1;       //Ĭ���Ҳ�����
        L_flag=1;       //Ĭ���Ҳ�����
        Img_R[i]=picture_wide*8-1;    
        j=farthest_scan_X_max/8;
        if((imgbuff[i*picture_wide+j]&((0xff>>((farthest_scan_X_max%8))+1)&0xff))!=0)
        {
            for(k=7-(farthest_scan_X_max%8);k>=0;k--)
            {
              //�׵�&&�ڵ�&&+3<(i+1)*picture&&+3�ڵ�
                if(
                    (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //��ɫ
                    &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//�ҵ�����                   //��ɫ         
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
            for(j=farthest_scan_X_max/8;j<picture_wide;j++)//�ұ�
            {
                if(imgbuff[i*picture_wide+j]!=0)
                {
                    for(k=7;k>=0;k--)
                    {
                      //�׵�&&�ڵ�&&+3<(i+1)*picture&&+3�ڵ�
                      if(
                         (imgbuff[i*picture_wide+j-(k+1)/8]&(1<<(k+1)%8))==0       //��ɫ
                         &&(imgbuff[i*picture_wide+j]&(1<<k))!=0)//�ҵ�����                   //��ɫ
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
            for(j=j-1;j>=0;j--)//���
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
              
        //������ж�
        if((Img_L[i]-Img_L[i+2])>10&&L_mutation_shrink_flag==0)
        {
            L_mutation_shrink_flag=i;
            L_shrink_flag[i]=Img_L[i];
      //     printf("������%d\n",L_mutation_shrink_flag);
      //     printf("����j��%d\n",L_shrink_flag[i]);
        }
        
        if((Img_R[i+2]-Img_R[i])>10&&R_mutation_shrink_flag==0)
        {
            R_mutation_shrink_flag=i;
            R_shrink_flag[i]=Img_R[i];
       //  printf("������%d\n",R_mutation_shrink_flag);
        }
        
        if((Img_L[i+2]-Img_L[i])>10&&L_mutation_wide_flag==0)
        {
            L_mutation_wide_flag=i+2;
      //     printf("���%d\n",L_mutation_wide_flag);
        }
        
        if((Img_R[i]-Img_R[i+2])>10&&R_mutation_wide_flag==0)
        {
            R_mutation_wide_flag=i+2;
      //      printf("�ҿ�%d\n",R_mutation_wide_flag);
        }       
          
         //�ж�����          
        if((Img_R[i+2]-Img_L[i+2])>=(Img_R[i+4]-Img_L[i+4])) 
        {
            if(R_convergence_flag==0&&Img_R[i]>Img_R[i+4]&&Img_R[i+2]>Img_R[i+4]&&Img_R[i+4]<=Img_R[i+6])     //��������
            {
                R_convergence_flag=i+4;     //��������־
      //        printf("�������㣺%d\n",R_convergence_flag);      
            }
            if(L_convergence_flag==0&&Img_L[i]<Img_L[i+4]&&Img_L[i+2]<Img_L[i+4]&&Img_L[i+4]>=Img_L[i+6])     //��������
            {
                L_convergence_flag=i+4;     //��������־
         //      printf("�������㣺%d\n",L_convergence_flag);
            }
        }  
            
        //ȫ���߱�־
        if(R_flag==1&&L_flag==1&&lose_line_flag==0)
        {
            lose_line_flag=1;
            lose_line_start=i;
        }
        //ʮ��������
        if(R_flag==0&&L_flag==0&&lose_line_flag==1)  
        {
            lose_line_back_effective_line_flag=i-2;
            lose_line_flag=2;
        }

        if(Img_R[i]>Img_R[i+2]&&Img_R[i+2]>Img_R[i+4]&&R_curve_flag==0&&i>25)//i>25
        {
          R_curve_flag=1;
       //    printf("����");
        }
        if(Img_L[i]<Img_L[i+2]&&Img_L[i+2]<Img_L[i+4]&&L_curve_flag==0&&i>25)//i>25
        {
          L_curve_flag=1;
          
        //   printf("����");
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
    /*    if(R_flag==1&&L_flag==0&&R_lose_line_flag==0) //�ұ߶���
        {
            R_lose_line_flag=1;
        }
       if(R_flag==0&&L_flag==0&&R_lose_line_flag==1) //�ұ��ҵ���
        {
            R_lose_line_flag=2;
            R_UP_convergence_flag=i;       
            
        }
        
        
        if(R_flag==0&&L_flag==1&&L_lose_line_flag==0)  //��߶���
        {
            L_lose_line_flag=1;
        }
       if(R_flag==0&&L_flag==0&&L_lose_line_flag==1)  //����ҵ���
        {
            L_lose_line_flag=2;
            L_UP_convergence_flag=i;
            
        }*/
        

        
        //�Ҷ��߿�ʼ
        if(R_flag==1&&R_lose_line_front_flag==0)
        {
            R_lose_line_front_flag=i;
         //printf("�Ҷ��߿�ʼ��%d\n",R_lose_line_front_flag);
        }
        //�Ҷ��߽���
        if(R_lose_line_front_flag!=0&&R_flag==0&&R_lose_line_back_flag==0)
        {
            R_lose_line_back_flag=i;
      //   printf("�Ҷ��߽�����%d\n",R_lose_line_back_flag);
        }
        //�Ҷ��߿�ʼ
        if(L_flag==1&&L_lose_line_front_flag==0)
        {
            L_lose_line_front_flag=i;
       //      printf("���߿�ʼ��%d\n",L_lose_line_front_flag);
        }
        //�Ҷ��߽���
        if(L_lose_line_front_flag!=0&&L_flag==0&&L_lose_line_back_flag==0)
        {
            L_lose_line_back_flag=i;
     //       printf("���߽�����%d\n",L_lose_line_back_flag);
        }
                
        //�򵥶��ߴ���
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
                    
        //�����־
        L_flag=0;
        R_flag=0;

        //������ʾ        
#if (display_original_line==1)
        imgbuff[i*picture_wide+(int)((int)middle_Line_temp[i]/8+0.5)]|=1<<7-(int)((int)middle_Line_temp[i]%8+0.5);       //��ʵ����
        imgbuff[i*picture_wide+picture_wide/2]=imgbuff[i*picture_wide+picture_wide/2]|(1<<7);                 //����ͷ��������
#endif            
    }
    //��Զ�У�������
    farthest_scan_Y_max=i+2;                 

    
   
    //****************************������ȡ*****************************//
    if(farthest_scan_Y_max<=prospect)
    {   
        middle_Line=(middle_Line_temp[55]+middle_Line_temp[57]+middle_Line_temp[53])/3*0.5+(middle_Line_temp[prospect]+middle_Line_temp[prospect+2])/2*0.5;
    }
    else
    {
        if((farthest_scan_Y_max-prospect)>=4)
        {
            middle_Line_temp[prospect]=(int)(middle_Line_temp[59]+(prospect-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);  //����ֱ��
            middle_Line_temp[(prospect+2)]=(int)(middle_Line_temp[59]+((prospect+2)-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);
        }
        else
        {
            middle_Line_temp[prospect]=(int)(middle_Line_temp[59]+(prospect-59)*(middle_Line_temp[farthest_scan_Y_max]-middle_Line_temp[59])/(farthest_scan_Y_max-59)+0.5);  //����ֱ��
        }
        middle_Line=(middle_Line_temp[55]+middle_Line_temp[57]+middle_Line_temp[53])/3*0.5+(middle_Line_temp[prospect]+middle_Line_temp[prospect+2])/2*0.5;

    }  
    middle_Line_last=middle_Line;
}

