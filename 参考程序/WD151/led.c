#include "led.h"
/******************************************************************************************************************
2018��ȫ����ѧ�������ֱ����ܳ�����
�ɶ���Ϣ���̴�ѧ   ����WD��
����QQ��97354734
�ļ�����:
*******************************************************************************************************************/
void led_init(void)
{
	gpio_init(I4,GPO,1);                         //��ʼ��LED0  ����    
	gpio_init(I1,GPO,1);                         //��ʼ��LED0  ����    
	gpio_init(I0,GPO,1);                         //��ʼ��LED0  ����       
}
void button_init(void)
{
  gpio_init (BUTTON_UP, GPI,1);
  gpio_init (BUTTON_LEFT, GPI,1);
  gpio_init (BUTTON_RIGHT, GPI,1);
  gpio_init (BUTTON_MID, GPI,1);
  gpio_init (BUTTON_DOWN, GPI,1);
	port_pull(BUTTON_UP);
	port_pull(BUTTON_LEFT);
	port_pull(BUTTON_RIGHT);
	port_pull(BUTTON_MID);
	port_pull(BUTTON_DOWN);

}
