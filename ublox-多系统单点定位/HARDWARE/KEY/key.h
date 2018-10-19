#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/7/11
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

#define KEY0 		GPIO_Pin_Get(GPIOH,PIN3)   	//PH3
#define KEY1 		GPIO_Pin_Get(GPIOH,PIN2)	//PH2 
#define KEY2 		GPIO_Pin_Get(GPIOC,PIN13)	//PC13
#define WK_UP 		GPIO_Pin_Get(GPIOA,PIN0)	//PA0 

#define KEY0_PRES 	1	//KEY0����
#define KEY1_PRES	2	//KEY1����
#define KEY2_PRES	3	//KEY2����
#define WKUP_PRES   4	//KEY_UP����(��WK_UP)

void KEY_Init(void);	//IO��ʼ��
u8 KEY_Scan(u8);  		//����ɨ�躯��					    
#endif
