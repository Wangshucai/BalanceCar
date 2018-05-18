/***************************************************************************************
									声明
本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他不可
估量的BUG，修远智控不负任何责任。请勿商用！

程序版本号：	2.0
日期：			2017-1-1
作者：			东方萧雨
版权所有：		修远智控N0.1实验室
****************************************************************************************/
#include "button.h"
#include "systick.h"

//用来保存哪个按键刚被按下
vu8 ButtonMask;

void BUTTON_Config(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	EXTI_InitTypeDef EXTI_initStructure;
	
	//时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB,&GPIO_initStructure);
	
	//给所用到的中断线路配置相应的触发模式和触发时机
	EXTI_initStructure.EXTI_Line = EXTI_Line12;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_initStructure);
	
	EXTI_initStructure.EXTI_Line = EXTI_Line13;
	EXTI_Init(&EXTI_initStructure);
	
	EXTI_initStructure.EXTI_Line = EXTI_Line14;
	EXTI_Init(&EXTI_initStructure);
	
	EXTI_initStructure.EXTI_Line = EXTI_Line15;
	EXTI_Init(&EXTI_initStructure);
	
	//开启GPIO管脚的中断线路
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);		
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource15);
	
	//清除中断标志，防止一打开中断通道就进入中断
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);
	
	//开启NVIC中EXTI的中断通道
	NVIC_EnableIRQ(EXTI15_10_IRQn);									
}






