#include "led.h" 

//��ʼ��PB0��PB1Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{    	 
	RCC->AHB1ENR|=1<<1;	//ʹ��PORTBʱ�� 
	GPIO_Set(GPIOB,PIN1,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PB0,PB1����
	LED0(1);			//�ر�DS0
}







