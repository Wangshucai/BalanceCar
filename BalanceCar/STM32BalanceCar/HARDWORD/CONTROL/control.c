#include "control.h"	

extern float Pitch,Roll; 
extern int Moto1,Moto2,Moto,Final_Moto1,Final_Moto2;

float Balance_Kp=350,Balance_Kd=0,Velocity_Kp=40,Velocity_Ki= 0.7 ;//0.7;//PID����
float Angle_Balance,Gyro_Balance;      //ƽ�����

int Angle_PWM,velocity_PWM,Turn_PWM,Balance_PWM;

void EXTI3_IRQHandler(void)
{
	Get_Angle();
	
	Angle_PWM = balance(Angle_Balance,Gyro_Balance);
 	velocity_PWM = velocity(Balance_PWM,Balance_PWM);
	Turn_PWM = turn(Balance_PWM,Balance_PWM);
	
	
	Balance_PWM = Angle_PWM + velocity_PWM;
    Moto1 = Balance_PWM - Turn_PWM;
	Moto2 = Balance_PWM + Turn_PWM;
	
	Xianfu_Pwm();
	
	Set_Pwm(Moto1,Moto2);
	//printf("%d\r\n",Final_Moto1);
    //ANO_DT_Send_Status(Pitch);�������ݵ���λ��
	EXTI_ClearITPendingBit(EXTI_Line3);  
}



int balance(float Angle ,float Gyro)
{  
     float Bias;    //����DΪ��
	 int balance;
	 Bias=Angle-ZHONGZHI;          //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=Balance_Kp*Bias+Balance_Kd*Gyro;      //===����ƽ����Ƶĵ��PWM 
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ��˲�
��ڲ������ٶ�
����  ֵ���˲�����ٶ�
**************************************************************************/
extern u8 Flag_Qian,Flag_Tui;


int Mean_Filter(int moto1,int moto2)
{
  u8 i;
  s32 Sum_Speed = 0; 
	s16 Filter_Speed;
  static  s16 Speed_Buf[FILTERING_TIMES]={0};
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Speed_Buf[i - 1] = Speed_Buf[i];
  }
  Speed_Buf[FILTERING_TIMES - 1] =moto1+moto2;

  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Speed += Speed_Buf[i];
  }
  Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);
	return Filter_Speed;
}

int velocity(int velocity_left,int velocity_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	static float Target_Velocity=10000;
	static float Encoder_Integral;  
	
		if(1==Flag_Qian)    	Movement=Target_Velocity;	           //===ǰ����־λ��1 
		else if(1==Flag_Tui)	Movement=-Target_Velocity;           //===���˱�־λ��1
  	else  Movement=0;
	 
   //=============�ٶ�PI������=======================//	
		Encoder_Least=Mean_Filter(velocity_left,velocity_right);          //�ٶ��˲�  
		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� 
		Encoder_Integral +=Movement;                                  //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>210000)  	Encoder_Integral=210000;            //===�����޷�
		if(Encoder_Integral<-210000)	Encoder_Integral=-210000;              //===�����޷�	
		Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/100;     //===�ٶȿ���	
  	
	  return Velocity;
}

extern u8 Flag_Left,Flag_Right;

int turn(int velocity_left,int velocity_right)//ת�����
{
	
	  static float Turn_Amplitude=300,Turn_Target,Turn_Convert=0.9;     
	  static float Turn_Count,Encoder_temp;
	  	if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(velocity_left+velocity_right);
			Turn_Convert=1000/Encoder_temp;
			if(Turn_Convert<6)Turn_Convert=6;
			if(Turn_Convert>50)Turn_Convert=50;
		}	
	  else
		{
			Turn_Convert=6;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target+=Turn_Convert;
		else if(1==Flag_Right)	       Turn_Target-=Turn_Convert; 
		else Turn_Target=0;
		
		
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
	  return Turn_Target;


}




/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���,�����ж�ת��
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
      if(moto1>0)			    Right_Direction=0;
			else 	              Right_Direction=1;
	  if(moto2>0)			    Left_Direction=1;
			else 	              Left_Direction=0;

	    Final_Moto1=Linear_Conversion(moto1);  //���Ի�
     	Final_Moto2=Linear_Conversion(moto2);
}



void Xianfu_Pwm(void)
{	
	  int Amplitude_H=5000, Amplitude_L=-5000;       
    if(Moto1<Amplitude_L)  Moto1=Amplitude_L;	
		if(Moto1>Amplitude_H)  Moto1=Amplitude_H;	
	  if(Moto2<Amplitude_L)  Moto2=Amplitude_L;	
		if(Moto2>Amplitude_H)  Moto2=Amplitude_H;	
}


void Get_Angle(void)
{ 
	 Read_DMP();                     
	 Angle_Balance=Pitch;             
	 Gyro_Balance=gyro[1];             	
}


int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


u16  Linear_Conversion(int moto)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=36000000/(PRESCALER+1)/13000*5000/myabs(moto);
	 if(temp>65235) Linear_Moto=65235;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	

