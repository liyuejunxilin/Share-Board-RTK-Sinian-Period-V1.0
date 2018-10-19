#include "timer.h"
#include "led.h"
#include "usart.h"



extern vu16 USART3_RX_STA;
//��ʱ��7�жϷ������		    
void TIM7_IRQHandler(void)
{ 	  		    
	if(TIM7->SR&0X01)//�Ǹ����ж�
	{	 			   
		USART3_RX_STA|=1<<15;	//��ǽ������
		TIM7->SR&=~(1<<0);		//����жϱ�־λ		   
		TIM7->CR1&=~(1<<0);		//�رն�ʱ��7	  
	}	      											 
} 
//ͨ�ö�ʱ��7�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ42M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz 
void TIM7_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<5;	//TIM7ʱ��ʹ��    
 	TIM7->ARR=arr;  	//�趨�������Զ���װֵ 
	TIM7->PSC=psc;  	//Ԥ��Ƶ��	  
	TIM7->CNT=0;  		//����������	  
	TIM7->DIER|=1<<0;   //��������ж�	  
	TIM7->CR1|=0x01;    //ʹ�ܶ�ʱ��7
  MY_NVIC_Init(0,1,TIM7_IRQn,2);	//��ռ0�������ȼ�1����2									 
} 
























