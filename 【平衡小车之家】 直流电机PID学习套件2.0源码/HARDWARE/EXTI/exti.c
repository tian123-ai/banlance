#include "exti.h"
#include "key.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
�������ܣ��ⲿ�жϳ�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void EXTI_Init(void)
{
	KEY_Init();	
	if(PAin(12)==0)    Menu_MODE=2;   //��סA12���������������ٶ�ģʽ
	Ex_NVIC_Config(GPIO_A,7,FTIR);		//�½��ش���
	Ex_NVIC_Config(GPIO_A,5,FTIR);		//�½��ش���
	Ex_NVIC_Config(GPIO_A,11,FTIR);		//�½��ش���
	Ex_NVIC_Config(GPIO_A,12,FTIR);		//�½��ش���

	MY_NVIC_Init(2,2,EXTI9_5_IRQn,2);  	//��ռ2�������ȼ�2����2
	MY_NVIC_Init(2,2,EXTI15_10_IRQn,2);	//��ռ2�������ȼ�2����2	  
}

//�ⲿ�ж�9~5�������
void EXTI9_5_IRQHandler(void)
{			
	delay_ms(4);   //����			 
   if(KEY5==0)	// 
	{
		Flag_Stop=!Flag_Stop; 
	}		
	 if(KEY7==0)	///Menu 
	{
		if(Menu_MODE==1)
		{	
		if(++Menu_PID>3) Menu_PID=1;
		}
		else
		{	
		if(++Menu_PID>2) Menu_PID=1;
		}	
	}		
		EXTI->PR=1<<5;     //���LINE5�ϵ��жϱ�־λ  
		EXTI->PR=1<<7;     //���LINE7�ϵ��жϱ�־λ
}
//�ⲿ�ж�15~10�������
void EXTI15_10_IRQHandler(void)
{			
	delay_ms(4);   //����		
	if(Menu_MODE==1)
	{	
			if(KEY11==0)	//PID-
			{
				      if(Menu_PID==1)  Position_KP-=Amplitude_PKP;
				else	if(Menu_PID==2)  Position_KI-=Amplitude_PKI;
				else	if(Menu_PID==3)  Position_KD-=Amplitude_PKD;
			}		
			 if(KEY12==0)	//PID+ 
			{
							if(Menu_PID==1)  Position_KP+=Amplitude_PKP;
				else	if(Menu_PID==2)  Position_KI+=Amplitude_PKI;
				else	if(Menu_PID==3)  Position_KD+=Amplitude_PKD;
			}		
  }
	else
	{	
			if(KEY11==0)	//PID-
			{
				      if(Menu_PID==1)  Velocity_KP-=Amplitude_VKP;
				else	if(Menu_PID==2)  Velocity_KI-=Amplitude_VKI;
			}		
			 if(KEY12==0)	//PID+ 
			{
							if(Menu_PID==1)  Velocity_KP+=Amplitude_VKP;
				else	if(Menu_PID==2)  Velocity_KI+=Amplitude_VKI;
			}		
  }
	if(Velocity_KP<=0) Velocity_KP=0;
	if(Velocity_KI<=0) Velocity_KI=0;
	if(Position_KD<=0) Position_KD=0;
	if(Position_KP<=0) Position_KP=0;
	if(Position_KI<=0) Position_KI=0;
  EXTI->PR=1<<11; //���LINE11�ϵ��жϱ�־λ  
	EXTI->PR=1<<12; //���LINE12�ϵ��жϱ�־λ 
}





