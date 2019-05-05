#include "System.h"
#include "led.h"
#include "Balance.h"
#include "MY_PID.h"
#include "ADRC.h"
#include "control.h"
#include "PID.h"
#include "MPU6050.h"
#include "My_I2C.h"
#include "EM.h"
#include "show.h"
uint8 Send_OK = 0;
uint8 System_OK = 0;
uint8 sector1 = FLASH_SECTOR_NUM - 1,sector2 = FLASH_SECTOR_NUM - 2,sector3 = FLASH_SECTOR_NUM - 3,sector4 = FLASH_SECTOR_NUM - 4;//flash�����
float Power=0;
u8 mark_set= 0;
void System_Init(void)
{
	DisableInterrupts ; 
	led_init();
	button_init();
	gpio_set(C2,0);// ��LEDָʾ��  
	OLED_Init();
	FLASH_Init(); 

	/************************ ������ʼ�� **********************************/

	PID_Parameter_Init(&Speed_PID);		// �ٶȻ�PID������ʼ��
	PID_Parameter_Init(&Angle_PID);		// �ǶȻ�PID������ʼ��
	PID_Parameter_Init(&Ang_gyro_PID);	// ���ٶȻ�PID������ʼ��
	PID_Parameter_Init(&Turn_gyro_PID);	// ת��PID������ʼ��
	PID_Parameter_Init(&Turn_PID);	// λ�û�PID������ʼ��
	Radius = 0;				// ��ʼ��Ŀ��ת��뾶����Ϊ0
	Speed_Set = 0;			// ��ʼ��Ŀ���ٶ�Ϊ0
	ADRC_Init(&ADRC_GYRO_Controller,&ADRC_SPEED_Controller,&ADRC_SPEED_MIN_Controller);//�Կ��ſ�������ʼ��

	/************************ ��� ��ʼ�� ************************************/
	ftm_pwm_init(ftm2,ftm_ch0,16000,0);
	ftm_pwm_init(ftm2,ftm_ch1,16000,0);
	ftm_pwm_init(ftm2,ftm_ch2,16000,0);
	ftm_pwm_init(ftm2,ftm_ch3,16000,0);
	
	
	/************************ ���� ��ʼ�� ************************************/
	uart_init(uart1, 115200);	
	uart_rx_irq_en(uart1);
	
	/************************ ������ ��ʼ�� **********************************/
	while (MPU6050_Init());		// �����ǳ�ʼ���ɹ�����0
	systick_delay_ms(20);
	/************************ ���������� ***************************************/   
	ftm_count_init(ftm0);   //��E0���������������м���    E0�ӱ�����LSB    
	gpio_init(C5,GPI,0);    //�����жϷ���                  C5�ӱ�����DIR
	port_pull(C5);          //IO����
	
	ftm_count_init(ftm1);   //��E7���������������м���    E7�ӱ�����LSB
	gpio_init(H5,GPI,0);    //�����жϷ���                  H5�ӱ�����DIR
	port_pull(H5);          //IO����
	

	/************************ ��ʱ�� ��ʼ��  *********************************/ 
 
	pit_init_ms(pit1, 1);                               //  ��ʱ 5ms    
	enable_irq(PIT_CH1_IRQn);                             // ʹ��PIT_CH1�ж�    
	
	
	/************************ LED ��ʼ��  ************************************/

	Speed_Control();	// �ٶ�����
	gpio_init(I5,GPI,1); //��ʼ���ɻɹ�

	/************************ ��вɼ� ***************************************/
	adc_init(ADC0_SE4);
	adc_init(ADC0_SE5);
	adc_init(ADC0_SE6);
	adc_init(ADC0_SE7);
	adc_init(ADC0_SE12);
	adc_init(ADC0_SE13);
	adc_init(ADC0_SE14);
	adc_init(ADC0_SE15);
	
	gpio_set(I4,0);// �ر�LEDָʾ�� ��ʼ�����
	EnableInterrupts;   //�����жϣ������õ��жϵģ�����Ҫ�ģ�
	Run_Flag =1;
	if (gpio_get(BUTTON_LEFT)==0)//���û���
	{
		Run_state = 0;
		while (1)
		{
			OLED_P6x8Str(0,0,"Ring_Num:");
			OLED_Print_Num1(80,0,Ring_Num);
			
			OLED_P6x8Str(0,01,"RING_1:");
			OLED_Print_Num1(80,1,Ring[0]);
			OLED_P6x8Str(0,2,"RING_1_ERR:");
			OLED_Print_Num1(80, 2,Ring_Err[0]);
//			OLED_P6x8Str(0,3,"Ring_En:");
//			OLED_Print_Num1(80, 3,Ring_En);
			OLED_P6x8Str(0,3,"RING_2:");
			OLED_Print_Num1(80, 3,Ring[1]);
			OLED_P6x8Str(0,4,"RING_2_ERR:");
			OLED_Print_Num1(80,4,Ring_Err[1] );
			
			OLED_P6x8Str(0,5,"RING_3:");
			OLED_Print_Num1(80,5, Ring[2]);
			OLED_P6x8Str(0,6,"RING_3_ERR:");
			OLED_Print_Num1(80,6,Ring_Err[2] );
			
			Ring_Set();
			if(gpio_get(BUTTON_MID)==0)//�˳�����
			{
				FLASH_EraseSector(sector1);  //�������һ������
				FLASH_EraseSector(sector2);
				FLASH_EraseSector(sector3);
				FLASH_EraseSector(sector4);
				FLASH_WriteSector(sector1,(const uint8 *)Ring,8,0);//������д��flash
				FLASH_WriteSector(sector2,(const uint8 *)Ring_Err,8,0);//     
				FLASH_WriteSector(sector3,(const uint8 *)&Ring_Num,4,0);// 
				FLASH_WriteSector(sector4,(const uint8 *)&Ring_En,4,0);// 
				break;
			}
		}
	}
	else if (gpio_get(BUTTON_RIGHT)==0)//����
	{
		Run_state = 0;
		Ring_Num = flash_read(sector3,0,uint16);     
		
		Ring[1] = !(flash_read(sector1,0,uint16));                           
		Ring[0] = !(flash_read(sector1,2,uint16));
		//Ring[0] = flash_read(sector1,4,uint16);

		Ring_Err[1] = flash_read(sector2,0,uint16);
		Ring_Err[0] = flash_read(sector2,2,uint16);
		//Ring_Err[0] = flash_read(sector2,4,uint16);
		
		FLASH_EraseSector(sector1);  //�������һ������
		FLASH_EraseSector(sector2);
		FLASH_WriteSector(sector1,(const uint8 *)Ring,8,0);//������д��flash
		FLASH_WriteSector(sector2,(const uint8 *)Ring_Err,8,0);//
	}
	else if (gpio_get(BUTTON_UP)==0)//�Զ���
	{
		Run_state = 1;
		Run_Stop = 1;
		Ring_Num = flash_read(sector3,0,uint16);     
		Ring[0] = flash_read(sector1,0,uint16);                           
		Ring[1] = flash_read(sector1,2,uint16);
		Ring[2] = flash_read(sector1,4,uint16);
		
		Ring_Err[0] = flash_read(sector2,0,uint16);
		Ring_Err[1] = flash_read(sector2,2,uint16);
		Ring_Err[2] = flash_read(sector2,4,uint16);
	}
	else                            //�����ϵ�
	{
		Run_state = 0;
		Ring_Num = flash_read(sector3,0,uint16);     
		Ring[0] = flash_read(sector1,0,uint16);                           
		Ring[1] = flash_read(sector1,2,uint16);
		Ring[2] = flash_read(sector1,4,uint16);

		Ring_Err[0] = flash_read(sector2,0,uint16);
		Ring_Err[1] = flash_read(sector2,2,uint16);
		Ring_Err[2] = flash_read(sector2,4,uint16);
		Ring_En = flash_read(sector4,0,uint16); 

	}

}




