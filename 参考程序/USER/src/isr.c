/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		�ж��ļ�
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/



#include "isr.h"
#include "loop.h"
#include "Balance.h"
#include "ANO_Data_Transfer.h"
#include "control.h"
#include "PID.h"
//#include "MPU6050.h"
//#include "My_I2C.h"
u32 time[7],time_sum;
void PIT_CH0_IRQHandler(void)
{

	PIT_FLAG_CLR(pit0);//�����ʱ���жϱ�־λ
}

void PIT_CH1_IRQHandler(void)
{
    
	static uint16 Time = 0;
	sysTickUptime++;
	if(sysTickUptime>5000)
	{
		gpio_set(I1,0);
	}
	Start_Control();
//	if(sysTickUptime<8000)//����ǰ3sѸ�ٵ��������ٶ� ֮������ٶȿ���
//	{
//		Speed[0] = 50;
//	}
//	else Speed[0] = 10;
	if (Run_OK==0)
	{
		Time++;
		if (Time >= 8000)			//��ʱ4s
		{
			Time=0;
			Run_OK=1;
		}
	}
	else
	{
		if (Stop_Flag==1)			//��ʱ
		{
			Time++;
			if (Time >= 1000)
			{
				Time = 0;
				Stop_Flag=0;
			}

		}
	}
	PIT_FLAG_CLR(pit1);
}

void IRQ_IRQHandler(void)
{
    CLEAR_IRQ_FLAG;
}


void KBI0_IRQHandler(void)
{
    CLEAN_KBI0_FLAG;
    
}

void UART1_IRQHandler(void)
{
    char ch;

    if(UART1->S1 & UART_S1_RDRF_MASK)   //�������ݼĴ�����
    {
			ch  =   UART1->D; 
			ANO_DT_Data_Receive_Prepare(ch);
			
    }
}
/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ���������ڶ�ʱ�� ͨ��0���ж�
void PIT_CH0_IRQHandler(void)
{
    ;
}
�ǵý����жϺ������־λ

FTMRE_IRQHandler      
PMC_IRQHandler        
IRQ_IRQHandler        
I2C0_IRQHandler       
I2C1_IRQHandler       
SPI0_IRQHandler       
SPI1_IRQHandler       
UART0_IRQHandler 
UART1_IRQHandler 
UART2_IRQHandler 
ADC0_IRQHandler       
ACMP0_IRQHandler      
FTM0_IRQHandler       
FTM1_IRQHandler       
FTM2_IRQHandler       
RTC_IRQHandler        
ACMP1_IRQHandler      
PIT_CH0_IRQHandler    
PIT_CH1_IRQHandler    
KBI0_IRQHandler       
KBI1_IRQHandler       
Reserved26_IRQHandler 
ICS_IRQHandler        
WDG_IRQHandler        
PWT_IRQHandler        
MSCAN_Rx_IRQHandler   
MSCAN_Tx_IRQHandler   
*/



