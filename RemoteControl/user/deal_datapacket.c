/***************************************************************************************
									声明
本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他不可
估量的BUG，修远智控不负任何责任。请勿商用！

程序版本号：	2.0
日期：			2017-1-1
作者：			东方萧雨
版权所有：		修远智控N0.1实验室
****************************************************************************************/
/****************************************************************************************
数据的打包和解包
*****************************************************************************************/
#include "deal_datapacket.h"
#include "usart.h"

u8 packetData[12];									//打包后待发送的数据包，这个长度必须和接收端NRF的接收通道定义的有效数据长度相同，否则接收不到数据
u8 dataPID = 0;										//数据包识别PID	
vu16 accelerator = 0;								//记录油门上一次的数值，用于下拉油门的时候，防止动力损失过大造成的失衡

extern vu8 ButtonMask;								//用来保存哪个按键刚被按下了，原始定义在：button.c文件中
extern vu16 ADC_ConvertedValue[4];					//用来保存ADC各通道转换完成后的数据，原始定义在：adc_dma.c中


/**************************************************************************************
打包数据

注意按照下面的通讯协议进行打包数据：以字节为单位：
前导码-按键MASK--ADC1低8--ADC1高8--ADC2低8--ADC2高8--ADC3低8--ADC3高8--ADC4低8--ADC4高8--数据包标识--校验码0xa5
其中：前导码只有0x01和0x08才表示有效的数据包，0x01表示此数据包是由ADC采样完成触发的，0x08表示
此数据包是由遥控器上的按键触发的，
数据包标识用于接收端NRF识别是否是同一数据包的作用（这在飞机上主要用于当遥控信号中断时，自动开始降落。）
**************************************************************************************/
void PackData(u8 firstByte)
{
	
//	ADC_ConvertedValue[0] = ADC_ConvertedValue[0];
	
	//由于油门ADC是12位的，所以最大值为4095，而电机的最大时钟周期数为7200，所以存在一个比例值，需要进行转换
	if((accelerator-(ADC_ConvertedValue[1]*3/2))>=800.0){
		accelerator -= 800;											//当快速拉低油门时，则进行一个迅速减小动力的措施
	}else{
		accelerator = ADC_ConvertedValue[1]*3/2;					//正常速度拉低油门时，则可以按照正常运行即可，注意加大油门时不会受此机制的影响
	}
//	printf("accelerator:::%d\r\n",accelerator);
	
	//换算左右方向ADC采样值
	if(ADC_ConvertedValue[2]<=1600){
		ADC_ConvertedValue[2] = ADC_ConvertedValue[2]*3/320;
	}else if(ADC_ConvertedValue[2]>=2100){
		ADC_ConvertedValue[2] = ADC_ConvertedValue[2]*3/350-3;
	}else{
		ADC_ConvertedValue[2] = 15;
	}
	
	//换算前后方向ADC采样值
	if(ADC_ConvertedValue[3]<=1800){
		ADC_ConvertedValue[3] = ADC_ConvertedValue[3]/120;
	}else if(ADC_ConvertedValue[3]>=2300){
		ADC_ConvertedValue[3] = ADC_ConvertedValue[3]*3/340-5;
	}else{
		ADC_ConvertedValue[3] = 15;
	}
	
//	printf("ADC1:%d\r\n",ADC_ConvertedValue[0]);
//	printf("ADC2:%d\r\n",ADC_ConvertedValue[1]);
//	printf("ADC3:%d\r\n",ADC_ConvertedValue[2]);
//	printf("ADC4:%d\r\n",ADC_ConvertedValue[3]);
//	printf("==================================\r\n");
	
	
	//数据包识别PID自增，并且超过200时自动归零
	if(dataPID>=200){
		dataPID = 0;
	}else{
		dataPID++;
	}
	
	
	
	//================将处理好的数据进行打包======================================================
	//=========采用移位方式进行数据打包，注意，这种方式有可能会发生精度截取的现象==================
//	packetData[0] = firstByte;							//前导码
//	packetData[1] = ButtonMask;
//	packetData[2] = (u8)(ADC_ConvertedValue[0]>>8);
//	packetData[3] = ADC_ConvertedValue[0];
//	packetData[4] = (u8)(ADC_ConvertedValue[1]>>8);
//	packetData[5] = ADC_ConvertedValue[1];
//	packetData[6] = (u8)(ADC_ConvertedValue[2]>>8);
//	packetData[7] = ADC_ConvertedValue[2];
//	packetData[8] = (u8)(ADC_ConvertedValue[3]>>8);
//	packetData[9] = ADC_ConvertedValue[3];
//	packetData[10] = 0xa5;								//校验码：1010 0101
	
	
	//=========直接采用指针操作内存中的数值将16位转成8位，速度快，并且不会发生精度截取的现象，还要注意，STM32是小端地址============
//	packetData[0] = firstByte;							//前导码
//	packetData[1] = ButtonMask;
//	packetData[2] = *((u8*)ADC_ConvertedValue);
//	packetData[3] = *(((u8*)ADC_ConvertedValue)+1);
//	packetData[4] = *(((u8*)&accelerator));				//油门数据打包
//	packetData[5] = *(((u8*)&accelerator)+1);
//	packetData[6] = *(((u8*)ADC_ConvertedValue)+4);
//	packetData[7] = *(((u8*)ADC_ConvertedValue)+5);
//	packetData[8] = *(((u8*)ADC_ConvertedValue)+6);
//	packetData[9] = *(((u8*)ADC_ConvertedValue)+7);
//	packetData[10] = dataPID;							//这个非常重要，这是防止飞机逃脱遥控的保证
//	packetData[11] = 0xa5;								//校验码：1010 0101
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
//	printf("%x\r\n",packetData[1]);
}







