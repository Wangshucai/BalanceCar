/***************************************************************************************
									声明
本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他不可
估量的BUG，修远智控不负任何责任。请勿商用！

程序版本号：	2.0
日期：			2017-1-1
作者：			东方萧雨
版权所有：		修远智控N0.1实验室
****************************************************************************************/
/*************************************************************************************
nRF无线收发程序，以接收模式为主，当有数据要发送时，则切换到发射模式，并且发送完成后自动
切换回接收模式，等待接收数据，同时当接收到数据时，LED9会闪烁

注意：
1.因为需要用到延迟函数，所以在NRF_Init()初始化前要先把systick_init()初始化
2.因为在EXTI中要用到SYSTICK的中断函数，所以需要配置NVIC，进行中断优先级的配置

主函数初始化代码为：
	NRF_Config();
***************************************************************************************/
#include "nRF.h"
#include "spi.h"
void UnpackData(void);
/**********************************************************************
宏定义区
***********************************************************************/
#define CE_LOW					GPIOA->BRR |= GPIO_Pin_4
#define CE_HIGH					GPIOA->BSRR |= GPIO_Pin_4

//==========================NRF24L01============================================
#define TX_ADR_WIDTH    		5   		//这个地址是接收端的接收通道的地址的宽度
#define RX_ADR_WIDTH    		5   		//这个是本机接收通道0的地址宽度
#define TX_PLOAD_WIDTH  		11  		//要发送的有效数据长度，这个一般和接收端NRF的RX的FIFO设置值相对应
#define RX_PLOAD_WIDTH  		12  		//要接收的有效数据长度，这个决定RX端FIFO达到多少数据量后触发中断
//=========================NRF24L01寄存器指令===================================
#define READ_REG_CMD        	0x00  		// 读寄存器指令
#define WRITE_REG_CMD       	0x20 		// 写寄存器指令
#define RD_RX_PLOAD     		0x61  		// 读取接收数据指令
#define WR_TX_PLOAD     		0xA0  		// 写待发数据指令
#define FLUSH_TX        		0xE1 		// 冲洗发送 FIFO指令
#define FLUSH_RX        		0xE2  		// 冲洗接收 FIFO指令
#define REUSE_TX_PL     		0xE3  		// 定义重复装载数据指令
#define NOP            			0xFF  		// 保留
//========================SPI(nRF24L01)寄存器地址===============================
#define CONFIG          		0x00  		// 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           		0x01  		// 自动应答功能设置
#define EN_RXADDR       		0x02  		// 可用信道设置
#define SETUP_AW        		0x03  		// 收发地址宽度设置
#define SETUP_RETR      		0x04  		// 自动重发功能设置
#define RF_CH           		0x05  		// 工作频率设置
#define RF_SETUP        		0x06  		// 发射速率、功耗功能设置
#define STATUS          		0x07  		// 状态寄存器
#define OBSERVE_TX      		0x08  		// 发送监测功能
#define CD              		0x09  		// 地址检测           
#define RX_ADDR_P0      		0x0A  		// 频道0接收数据地址
#define RX_ADDR_P1      		0x0B  		// 频道1接收数据地址
#define RX_ADDR_P2      		0x0C  		// 频道2接收数据地址
#define RX_ADDR_P3      		0x0D  		// 频道3接收数据地址
#define RX_ADDR_P4      		0x0E  		// 频道4接收数据地址
#define RX_ADDR_P5      		0x0F  		// 频道5接收数据地址
#define TX_ADDR         		0x10  		// 发送地址寄存器
#define RX_PW_P0        		0x11 		// 接收频道0接收数据长度
#define RX_PW_P1        		0x12  		// 接收频道0接收数据长度
#define RX_PW_P2        		0x13  		// 接收频道0接收数据长度
#define RX_PW_P3        		0x14  		// 接收频道0接收数据长度
#define RX_PW_P4        		0x15  		// 接收频道0接收数据长度
#define RX_PW_P5        		0x16  		// 接收频道0接收数据长度
#define FIFO_STATUS     		0x17  		// FIFO栈入栈出状态寄存器设置
//=============================RF24l01状态=====================================
u8 TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};	//此地址用来识别接收端哪个RX通道可以接收发送出去的数据包
u8 RX_ADDRESS[RX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};	//此地址用来配置本机NRF的RX0通道的地址，同时为了能正常收到应答信号，此地址一般都和上面的地址配置相同

//发生中断时，根据STATUS寄存器中的值来判断是哪个中断源触发了IRQ中断
#define TX_DS					0x20		//数据发送完成中断
#define RX_DR					0x40		//数据接收完成中断
#define MAX_RT					0x10		//数据包重发次数超过设定值中断

//============================全局变量定义区==========================================
vu8 sta;									//接收从STATUS寄存器中返回的值
u8 RxBuf[RX_PLOAD_WIDTH];					//数据接收缓冲区

extern u8 buttonFlag;						//指示哪个按键被按下，原始定义在deal_datapacket.c文件中
//====================================================================================



/**********************************************************************
初始化NRF，及其IRQ引脚对应的EXTI中断
***********************************************************************/
void NRF_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructus;
	EXTI_InitTypeDef EXTI_initStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//初始化SPI接口
	SPI_Config();
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	
	//初始化CE引脚
	GPIO_InitStructus.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructus.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructus.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructus);

	//配置IRQ引脚对应的IO口
	GPIO_InitStructus.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructus.GPIO_Mode = GPIO_Mode_IPU;							//nRF中断产生时，IRQ引脚会被拉低，所以这里要配置成上拉输入
	GPIO_Init(GPIOB,&GPIO_InitStructus);

	EXTI_initStructure.EXTI_Line = EXTI_Line0;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger = EXTI_Trigger_Falling;					//下降沿触发
	EXTI_Init(&EXTI_initStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);				//开启GPIO管脚的中断线路
	
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键KEY1所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
	//NVIC_EnableIRQ(EXTI0_IRQn);
	
	
	CE_LOW;																	//拉低CE，注意：读/写nRF寄存器均需要将CE拉低，使其进入待机或者掉电模式才可以
	
	//初始化NRF
	SPI_Write_Byte(WRITE_REG_CMD + SETUP_AW, 0x03);							//配置通信地址的长度，默认值时0x03,即地址长度为5字节
	SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    	//发送的数据包中被一块打包进去的接收端NRF的接收通道的地址
	SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); 	//配置本机接收通道0的接收数据的地址
	SPI_Write_Byte(WRITE_REG_CMD + SETUP_RETR, 0x1a); 						//自动重发延迟为500+86us，重发次数10次
	SPI_Write_Byte(WRITE_REG_CMD + EN_AA, 0x01);      						//接收数据后，只允许频道0自动应答
	SPI_Write_Byte(WRITE_REG_CMD + EN_RXADDR, 0x01);  						//只允许频道0接收数据
	
	//测试NRF作为发射端的时候是否能发出数据的测试代码，程序正常运行时，可以注释掉
//	SPI_Write_Byte(WRITE_REG_CMD + SETUP_RETR, 0x00);
//	SPI_Write_Byte(WRITE_REG_CMD + EN_AA, 0x00); 
//	SPI_Write_Byte(WRITE_REG_CMD + EN_RXADDR, 0x00); 
	
	SPI_Write_Byte(WRITE_REG_CMD + RF_SETUP, 0x07);   						//设置发射速率为1MHZ，发射功率为最大值0dB
	SPI_Write_Byte(WRITE_REG_CMD + RF_CH, 30);        						//设置通道通信频率，工作通道频率可由以下公式计算得出：Fo=（2400+RF-CH）MHz.并且射频收发器工作的频率范围从2.400-2.525GHz
	SPI_Write_Byte(WRITE_REG_CMD + RX_PW_P0, RX_PLOAD_WIDTH); 				//设置接收数据长度，本次设置为5字节，只有接收的数据达到此长度时，才会触发RX_DS中断
	
	SPI_Write_Byte(WRITE_REG_CMD + CONFIG, 0x0f);   						//默认处于接收模式

	//读回配置信息，防止配置出错（调试代码的时候可以使用，程序正常运行代码时，注释掉这些打印代码）
//	printf("SETUP_AW:%x\r\n",SPI_Read_Byte(READ_REG_CMD+SETUP_AW));
//	printf("SETUP_RETR:%x\r\n",SPI_Read_Byte(READ_REG_CMD+SETUP_RETR));
//	printf("EN_RXADDR:%x\r\n",SPI_Read_Byte(READ_REG_CMD+EN_RXADDR));
//	printf("EN_AA:%x\r\n",SPI_Read_Byte(READ_REG_CMD+EN_AA));
//	printf("RF_SETUP:%x\r\n",SPI_Read_Byte(READ_REG_CMD+RF_SETUP));
//	printf("RF_CH:%x\r\n",SPI_Read_Byte(READ_REG_CMD+RF_CH));
//	printf("RX_PW_P0:%x\r\n",SPI_Read_Byte(READ_REG_CMD+RX_PW_P0));
//	printf("CONFIG:%x\r\n",SPI_Read_Byte(READ_REG_CMD+CONFIG));

	CE_HIGH;
}

/*****************************************************************************************
IRQ引脚对应的EXTI中断处理函数
*******************************************************************************************/
void EXTI0_IRQHandler(void)
{
	CE_LOW;													//拉低CE，以便读取NRF中STATUS中的数据
	sta = SPI_Read_Byte(READ_REG_CMD+STATUS);				//读取STATUS中的数据，以便判断是由什么中断源触发的IRQ中断
	
	if(sta & TX_DS){										//数据发送成功，并且收到了应答信号
		RX_Mode();											//将NRF的模式改为接收模式，等待接收数据
	}else if(sta & RX_DR){									//数据接收成功
		NRF_ReceivePacket();								//将数据从RX端的FIFO中读取出来，
//		if(buttonFlag & 0x01){
//			LED_On(LED9);									//LED9常亮表示飞机解锁成功，并且和遥控器正在进行数据通讯
//		}else{
//			LED_Toggle(LED9);								//LED9闪烁表示飞机处于加锁模式，但是和遥控器通讯成功
//		}
	}else if(sta & MAX_RT){									//触发了最大重发MAX_RT中断
		RX_Mode();											//将NRF的模式改为接收模式
		SPI_Write_Byte(WRITE_REG_CMD+STATUS,sta);			//清除MAX_RT中断
	}
	
	EXTI_ClearITPendingBit(EXTI_Line0);
}



/**********************************************************************
配置NRF为RX模式，准备开始接收数据
***********************************************************************/
void RX_Mode(void)
{
	CE_LOW;													//拉低CE，进入待机模式，准备开始往NRF中的寄存器中写入数据
	
	SPI_Write_Byte(WRITE_REG_CMD + CONFIG, 0x0f); 			//配置为接收模式
	SPI_Write_Byte(WRITE_REG_CMD + STATUS, 0x7e);			//写0111 xxxx 给STATUS，清除所有中断标志，防止一进入接收模式就触发中断
	
	CE_HIGH; 												//拉高CE，准备接受从外部发送过来的数据
}


/**********************************************************************
从NRF的RX的FIFO中读取一组数据包
输入参数rx_buf:保存从FIFO中读取到的数据的区域首地址
***********************************************************************/
void NRF_ReceivePacket(void)
{
	CE_LOW;
	
	SPI_Read_Buf(RD_RX_PLOAD,RxBuf,RX_PLOAD_WIDTH);			//从RX端的FIFO中读取数据，并存入指定的区域，注意：读取完FIFO中的数据后，NRF会自动清除其中的数据
	SPI_Write_Byte(WRITE_REG_CMD+STATUS,sta);   			//清楚中断标志
	
	UnpackData();											//将接收到的数据解包，处理数据，并分装
	
	CE_HIGH;												//重新拉高CE，让其重新处于接收模式，准备接收下一个数据
}


/**********************************************************************
配置NRF为TX模式，并发送一个数据包
输入参数tfbuf:即将要发送出去的数据区首地址
***********************************************************************/
void NRF_SendPacket(u8* tfbuf)
{
	CE_LOW;																	//拉低CE，进入待机模式，准备开始往NRF中的寄存器中写入数据
	
	//SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 	//装载接收端地址，由于这里只有一个通道通讯，不用改变接收端的NRF的接收通道地址,所以，这句可以注释掉
	SPI_Write_Buf(WR_TX_PLOAD, tfbuf, TX_PLOAD_WIDTH); 						//将数据写入TX端的FIFO中,写入的个数与TX_PLOAD_WIDTH设置值相同
	
	SPI_Write_Byte(WRITE_REG_CMD + CONFIG, 0x0e); 							//将NRF配置成发射模式
	SPI_Write_Byte(WRITE_REG_CMD + STATUS, 0x7e);							//写0111 xxxx 给STATUS，清除所有中断标志，防止一进入发射模式就触发中断
	
	CE_HIGH;																//拉高CE，准备发射TX端FIFO中的数据
	
	//delay_ms(1);															//CE拉高后，需要延迟至少130us
}



u8 dataPID;												//保存接收到的数据包识别PID值
u8 buttonFlag = 0x00;									//指示哪个按键被按下，0x00的时候表示4个按键都没有被触发，0000 1111各表示KEY1~KEY4按键被按下
vu16 remoteControl[4];									//将从遥控器中接收到的数据重新拼装

u8 Flag_Qian,Flag_Tui;
u8 Flag_Left,Flag_Right;

extern u8 RxBuf[12];									//数据接收缓冲区,原始定义在nRF.c文件中
//===========================================================================================


/******************************************************************************************
解包接收到的数据,并根据发送过来的数据进行相应的处理

注意按照下面的通讯协议进行解码：以字节为单位：
前导码-按键MASK--ADC1低8--ADC1高8--ADC2低8--ADC2高8--ADC3低8--ADC3高8--ADC4低8--ADC4高8--数据包标识--校验码0xa5
其中：前导码只有0x01和0x08才表示有效的数据包，0x01表示此数据包是由ADC采样完成触发的，0x08表示
此数据包是由遥控器上的按键触发的
数据包标识用于识别是否是同一数据包的作用（这在飞机上主要用于当遥控信号中断时，自动开始降落。）
******************************************************************************************/
void UnpackData(void)
{
	if(RxBuf[11]!=0xa5)									//验证校验码是否为0xa5
		return;
	
	if(RxBuf[0] & 0x01){								//当数据包是由遥控器的ADC采样完成时触发发送时
//		remoteControl[0] = RxBuf[3]<<8|RxBuf[2];		//ADC2
		remoteControl[1] = RxBuf[5]<<8|RxBuf[4];		//ADC1
		remoteControl[2] = RxBuf[7]<<8|RxBuf[6];		//ADC4
		remoteControl[3] = RxBuf[9]<<8|RxBuf[8];		//ADC3

		if(remoteControl[2] > 25)
		{
			Flag_Left = 0;
			Flag_Right = 1;
		}else if(remoteControl[2] < 5)
		{
			Flag_Left = 1;
			Flag_Right = 0;
		}else{
		    Flag_Left = 0;
			Flag_Right = 0;
		}
		
		
		if(remoteControl[3] > 25)
		{
			Flag_Qian = 1;
			Flag_Tui = 0;
		}else if(remoteControl[3] < 5)
		{
			Flag_Qian = 0;
			Flag_Tui = 1;
		}else{
		    Flag_Qian = 0;
			Flag_Tui = 0;
		}
		
		
	}else if(RxBuf[0] & 0x08){							//当数据包是由遥控器的按键触发发送时
		buttonFlag = RxBuf[1];
	}
	
	//将数据包识别PID值取出，覆盖之前的值，以表示信号链接正常
	dataPID = RxBuf[10];
}


