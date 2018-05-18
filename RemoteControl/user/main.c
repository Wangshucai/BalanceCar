/***************************************************************************************
									声明
本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他不可
估量的BUG，修远智控不负任何责任。请勿商用！

程序版本号：	2.0
日期：			2017-1-1
作者：			东方萧雨
版权所有：		修远智控N0.1实验室
****************************************************************************************/
#include "stm32f10x.h"
#include "led.h"
#include "systick.h"
#include "usart.h"
#include "spi.h"
#include "nRF.h"
#include "nvic.h"
#include "adc_dma.h"
#include "tim_octigr.h"
#include "button.h"
extern u8 packetData[11];				//打包后待发送的数据包，原始定义在：nRF.c文件中
int main(void)
{
	USART_Config();
	//printf("usart is ready\r\n");
	SysTick_Init();
	NVIC_Config();
	//BUTTON_Config();
	//LED_Config();
//	LED_On(LED1|LED2);
	NRF_Config();
	//TIM_OCTigrConfig();
	
	//ADC_DmaConfig();	//这个必须最后一个配置，因为一旦配置好这个，ADC就会开始工作了，则DMA会开始每个一段时间产生中断，或者先关闭总中断，最后所有都配置完毕后在打开总中断
	packetData[0] = 0x01;							//前导码
	packetData[1] = 0x02;
	packetData[2] = 0x03;
	packetData[3] = 0x04;
	packetData[4] = 0x05;				//油门数据打包
	packetData[5] = 0x06;
	packetData[6] = 0x07;
	packetData[7] = 0x08;
	packetData[8] = 0x09;
	packetData[9] = 0x10;
	packetData[10] = 0x11;							//这个非常重要，这是防止飞机逃脱遥控的保证
	packetData[11] = 0xa5;								//校验码：1010 0101	
	while(1){
	
	NRF_SendPacket(packetData);
	
	
	}
}







