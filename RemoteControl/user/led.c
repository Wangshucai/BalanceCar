/***************************************************************************************
									声明
本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他不可
估量的BUG，修远智控不负任何责任。请勿商用！

程序版本号：	2.0
日期：			2017-1-1
作者：			东方萧雨
版权所有：		修远智控N0.1实验室
****************************************************************************************/
/********************************************************************************
点亮遥控器上的LED灯

注意：
1.当要使用LED_CyclieOn(...)函数循环点亮LED1和LED2灯时，请先初始化SysTick_init()

主函数初始化代码：
	LED_Config();
	
测试代码为：
	SysTick_init();
	LED_Config();
	LED_CyclieOn(1000);
	
	while(1){}
*******************************************************************************/
#include "led.h"
#include "systick.h"

void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructus;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructus.GPIO_Pin = LED1|LED2;
	GPIO_InitStructus.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructus.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_InitStructus);
	
	LED_Off(LED1|LED2);
}

//点亮LED灯
void LED_On(u16 LEDx)
{
	GPIO_ResetBits(GPIOB,LEDx);
}

void LED_Off(u16 LEDx)
{
	GPIO_SetBits(GPIOB,LEDx);
}


//开关LED
void LED_Toggle(u16 LEDx)
{
	GPIOB->ODR ^= LEDx;		//异或，开关模式输出高低电平
}


//循环点亮LED
void LED_CyclieOn(u32 delayms)
{	
	while(1){
		LED_On(LED1);
		delay_ms(delayms);
		LED_Off(LED1);
		
		LED_On(LED2);
		delay_ms(delayms);
		LED_Off(LED2);
	}
}


