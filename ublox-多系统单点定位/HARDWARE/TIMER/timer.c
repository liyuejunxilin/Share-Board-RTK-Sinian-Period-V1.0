#include "timer.h"
#include "led.h"
#include "usart.h"



extern vu16 USART3_RX_STA;
//定时器7中断服务程序		    
void TIM7_IRQHandler(void)
{ 	  		    
	if(TIM7->SR&0X01)//是更新中断
	{	 			   
		USART3_RX_STA|=1<<15;	//标记接收完成
		TIM7->SR&=~(1<<0);		//清除中断标志位		   
		TIM7->CR1&=~(1<<0);		//关闭定时器7	  
	}	      											 
} 
//通用定时器7中断初始化
//这里时钟选择为APB1的2倍，而APB1为42M
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz 
void TIM7_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<5;	//TIM7时钟使能    
 	TIM7->ARR=arr;  	//设定计数器自动重装值 
	TIM7->PSC=psc;  	//预分频器	  
	TIM7->CNT=0;  		//计数器清零	  
	TIM7->DIER|=1<<0;   //允许更新中断	  
	TIM7->CR1|=0x01;    //使能定时器7
  MY_NVIC_Init(0,1,TIM7_IRQn,2);	//抢占0，子优先级1，组2									 
} 
























