#ifndef __TIME_H
#define __TIME_H

#include "sys.h"	 
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
extern int Moto1,Moto2,Final_Moto1,Final_Moto2;
#define Left_Direction   PAout(2) 
#define Right_Direction  PAout(3) 
#define EN1   PBout(8) 
#define EN2   PBout(9) 
void MiniBalance_PWM_Init(void);
void MiniBalance_Motor_Init(void);
#endif
