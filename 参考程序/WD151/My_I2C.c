
/******************************* ģ��I2C������ *******************************/
#include "My_I2C.h"

//�ڲ����ݶ���
uint8 IIC_ad_main; //�����ӵ�ַ	    
uint8 IIC_ad_sub;  //�����ӵ�ַ	   
uint8 *IIC_buf;    //����|�������ݻ�����	    
uint8 IIC_num;     //����|�������ݸ���

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC�˿ڳ�ʼ��
//  @param      NULL
//  @return     void	
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void IIC_init(void)
{
	gpio_init(My_SCL, GPO,1);
	gpio_init(My_SDA, GPO,1);
	
	port_pull (My_SCL);//ODO
	port_pull (My_SDA);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ʱ
//  @return     void						
//  @since      v1.0
//  Sample usage:				���IICͨѶʧ�ܿ��Գ�������j��ֵ
//-------------------------------------------------------------------------------------------------------------------
void simiic_delay(void)
{
	uint16 j=20;   
	while(j--);
}

void My_Delay_Us(uint32 us)
{
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	
}

//�ڲ�ʹ�ã��û��������
void IIC_start(void)
{
	DIR_OUT();
	SDA1();
	SCL1();
	My_Delay_Us(4);
	SDA0();
	My_Delay_Us(4);
	SCL0();
}

//�ڲ�ʹ�ã��û��������
void IIC_stop(void)
{
	DIR_OUT();
	SCL0();
	SDA0();
	My_Delay_Us(4);
	SCL1();
	SDA1();
	My_Delay_Us(4);
}

//��Ӧ��(����ack:SDA=0��no_ack:SDA=1)
//�ڲ�ʹ�ã��û��������
void I2C_SendACK(unsigned char ack_dat)
{
	SCL0();
	DIR_OUT();
	if(ack_dat) 
	{
		SDA0();
	}
    else    	
	{
		SDA1();
	}
	My_Delay_Us(2);
    SCL1();
    My_Delay_Us(2);
    SCL0();
}

int SCCB_WaitAck(void)
{
	uint8 ErrorTime = 0;
	
//	DIR_OUT();
//	SDA1();
//	My_Delay_Us(1);
	DIR_IN();
    SCL1();
	My_Delay_Us(1);
	
    while (SDA)           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
    {
		ErrorTime++;
		if (ErrorTime > 250)
		{
			IIC_stop();
			
			return 1;
		}
    }
    SCL0();

    return 0;
}

//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
//�ڲ�ʹ�ã��û��������
void send_ch(uint8 c)
{
	uint8 i = 8;
	
	DIR_OUT();
	SCL0();
    while(i--)
    {
        if(c & 0x80)	SDA1();//SDA �������
        else			SDA0();
        c <<= 1;
		My_Delay_Us(2);
        SCL1();                //SCL ���ߣ��ɼ��ź�
		My_Delay_Us(2);
        SCL0();                //SCL ʱ��������
		My_Delay_Us(2);
    }
	SCCB_WaitAck();
}

//�ֽڽ��ճ���
//�����������������ݣ��˳���Ӧ���|��Ӧ����|IIC_ack_main()ʹ��
//�ڲ�ʹ�ã��û��������
uint8 read_ch(uint8 Ack)
{
    uint8 i, c = 0;
	
    DIR_IN();
    for(i=0;i<8;i++)
    {
        SCL0();         //��ʱ����Ϊ�ͣ�׼����������λ
        My_Delay_Us(2);
        SCL1();         //��ʱ����Ϊ�ߣ�ʹ��������������Ч
        c<<=1;
        if(SDA) c+=1;   //������λ�������յ����ݴ�c
		My_Delay_Us(1);
    }
	if (Ack == 0)
	{
		I2C_SendACK(no_ack);
	}
	else
	{
		I2C_SendACK(ack);
	}
	
    return c;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IICд���ݵ��豸�Ĵ�������
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      dat				д�������
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);   //����������ַ��дλ
	send_ch( reg );   				 //���ʹӻ��Ĵ�����ַ
	send_ch( dat );   				 //������Ҫд�������
	IIC_stop();
}
//  @brief      ģ��IIC����д���ݵ��豸�Ĵ�������
uint8 simiic_write_len(uint8 dev_add, uint8 reg, uint8 len, uint8 *dat)
{
	uint8 i;
	
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);   //����������ַ��дλ
	send_ch( reg );   				 //���ʹӻ��Ĵ�����ַ
	for (i = 0; i < len; i++)
	{
		send_ch(dat[i]);   				 //������Ҫд�������
	}
	IIC_stop();
	
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC���豸�Ĵ�����ȡ����
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      type			ѡ��ͨ�ŷ�ʽ��IIC  ���� SCCB
//  @return     uint8			���ؼĴ���������			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8 simiic_read_reg(uint8 dev_add, uint8 reg, IIC_type type)
{
	uint8 dat;
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
	send_ch( reg );   				//���ʹӻ��Ĵ�����ַ
	if(type == SCCB)IIC_stop();
	
	IIC_start();
	send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
	dat = read_ch(0);   			//������Ҫд�������
	IIC_stop();
	
	return dat;
}
//  @brief      ģ��IIC�������豸�Ĵ�����ȡ����
uint8 simiic_read_len(uint8 dev_add, uint8 reg, uint8 len, uint8 *dat)
{
	uint8 i;

	IIC_start();
    send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
	send_ch( reg );   				//���ʹӻ��Ĵ�����ַ
	IIC_start();
	send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
	for (i = 0; i < len-1; i++)
	{
		dat[i] = read_ch(1);	//������Ҫд�������
	}
	dat[i] = read_ch(0);	//������Ҫд�������
	IIC_stop();
	
	return 0;
}

