#ifndef __LED_H
#define __LED_H	 
#include "sys.h" 
	 

//LED�˿ڶ���
#define LED0(x)			GPIO_Pin_Set(GPIOB,PIN1,x)		// DS0
//#define LED1(x)			GPIO_Pin_Set(GPIOB,PIN0,x)		// DS1 

void LED_Init(void);	//��ʼ��		 				    
#endif

















