#ifndef __TIME_H
#define __TIME_H

#include "sys.h"	 
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
extern int Moto1,Moto2,Final_Moto1,Final_Moto2;
#define Left_Direction   PAout(2) 
#define Right_Direction  PAout(3) 
#define EN1   PBout(8) 
#define EN2   PBout(9) 
void MiniBalance_PWM_Init(void);
void MiniBalance_Motor_Init(void);
#endif
