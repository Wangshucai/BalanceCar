#include "Time.h"
  
  
  
void MiniBalance_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE); 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);		

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;	//�˿�����
  GPIO_Init(GPIOA, &GPIO_InitStructure);				
}
/**************************************************************************
�������ܣ��������PWM�����ʼ������ ��ʱ��ģʽΪ����Ƚ�
��ڲ�������
����  ֵ����
**************************************************************************/
void MiniBalance_PWM_Init(void)
{		 			
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	  //ʹ��TIMʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  //ʹ��GPIO����ʱ��ʹ��
	                                                                     	

   //���ø�����Ϊ�����������,���PWM���岨��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;   //PB6 PB7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;       //2M����
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	TIM_TimeBaseStructure.TIM_Period = 65535; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =14; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle; //ѡ��ʱ��ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_CounterMode_Up; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);//ʧ��TIMx��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);//ʧ��TIMx��CCR2�ϵ�Ԥװ�ؼĴ���
    TIM_Cmd(TIM4, ENABLE);                            //ʹ��TIM4
    TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2 , ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
} 
/**************************************************************************
�������ܣ�TIM4�жϷ����� ���������ƵPWM���
��ڲ�������
����  ֵ����
**************************************************************************/


void TIM4_IRQHandler(void)
{
	  float Capture1,Capture2;
   if(TIM_GetITStatus(TIM4,TIM_IT_CC1)!=RESET)
	 {
	  TIM_ClearITPendingBit(TIM4, TIM_IT_CC1 );//���TIMx���жϴ�����λ
      Capture1 = TIM_GetCapture1(TIM4);

      TIM_SetCompare1(TIM4, Capture1 + Final_Moto1 );//����TIMx�Զ���װ�ؼĴ���ֵ
	 }
	 if(TIM_GetITStatus(TIM4,TIM_IT_CC2)!=RESET)
	 {
	  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2 );//���TIMx���жϴ�����λ
    Capture2 = TIM_GetCapture2(TIM4);
    TIM_SetCompare2(TIM4, Capture2 + Final_Moto2 );//����TIMx�Զ���װ�ؼĴ���ֵ
	 }
}
