#include "control.h"		
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
          TIM1���ƵĶ�ʱ�ж� 
**************************************************************************/
u8 Flag_Velocity=1;
int TIM1_UP_IRQHandler(void)  
{    
	if(TIM1->SR&0X0001)//5ms��ʱ�ж�
	{   
		  TIM1->SR&=~(1<<0);                             //===�����ʱ��1�жϱ�־λ	      
	  if(Menu_MODE==1)		
		{
				Encoder=Read_Position(4);                      //===����λ����Ϣ
				Moto=Position_PID(Encoder,Target_Position);    //===λ��PID������
		}
	  else if(++Flag_Velocity>2)                         //===�ٶȿ���20msһ�Σ�һ�㽨��10ms����Ϊ����ʹ��USB���磬�ٶȱȽ�����20ms�����ӳ���ȡ�ٶȵĵ�λʱ�䣬�����ֵ
		{
				Flag_Velocity=1;
				Encoder=Read_Velocity(4);                      //===�����ٶ���Ϣ
			  if(Encoder<0) Encoder=0;
				Moto=Incremental_PI(Encoder,Target_Velocity);  //===�ٶ�PI������
		}
	  if(delay_flag==1)
	  {
		 if(++delay_50==5)	 delay_50=0,delay_flag=0;    //===���������ṩ50ms�ľ�׼��ʱ
	  }		
	  Xianfu_Pwm();                                    //===PWM�޷�
		if(Flag_Stop==0)			Set_Pwm(Moto);             //===��ֵ��PWM�Ĵ���
		else 	               	Set_Pwm(0);                //===��ֵ��PWM�Ĵ���
	  Led_Flash(100);                                  //===LED��˸ָʾϵͳ�������� 
		Key();                                           //===ɨ�谴���仯
		Voltage=Get_battery_volt();                      //===�ɼ������ѹ
	}       	
	 return 0;	  
} 
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto)
{
    	if(moto<0)			AIN2=1,			AIN1=0;
			else 	          AIN2=0,			AIN1=1;
			PWMA=myabs(moto);
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=7100;    //===PWM������7200 ������7100
	  if(Moto<-Amplitude) Moto=-Amplitude;	
		if(Moto>Amplitude)  Moto=Amplitude;		
}
/**************************************************************************
�������ܣ������޸�����״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)  
{	
	int tmp,tmp2,Position_Amplitude=260,Velocity_Amplitude=15; 
	tmp=click_N_Double(50); 
	 if(Menu_MODE==1)  //�ı�����λ�� 
	 {
		 	if(tmp==1)Target_Position+=Position_Amplitude;
			if(tmp==2)Target_Position-=Position_Amplitude;
	 }
	 else             //�ı������ٶ�
	 {
			if(tmp==1)Target_Velocity+=Velocity_Amplitude;
			if(tmp==2)Target_Velocity-=Velocity_Amplitude;
	 }
	 if(Target_Velocity>50)Target_Velocity=50; //�ٶ����ֵ�޷�
	 if(Target_Velocity<0)Target_Velocity=0;   //�ٶ���Сֵ�޷�
	tmp2=Long_Press();                   
  if(tmp2==1) Flash_Send=1;		//�����Ѳ���д��Flashʵ�ֵ��籣��
 }

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 Last_bias=Bias;	                                     //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
