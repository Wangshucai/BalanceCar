#include "sys.h" 

int Moto1,Moto2,Moto,Final_Moto1,Final_Moto2;


int main()
{
	
	MY_NVIC_PriorityGroupConfig(2);
	delay_init();
	uart_init(115200);
	   
	OLED_Init();
	EXTIX_Init();
	NRF_Config();
	
	IIC_Init();
	DMP_Init();
    
	MiniBalance_Motor_Init();       
    MiniBalance_PWM_Init();   
	
	
   

//	 Final_Moto1 = 300;//10KHz
//    //Final_Moto1 = 65535;//36Hz
//    Final_Moto2 = 300;
	
	while(1)
	{
				
		oled_Show();
	
	}
}
