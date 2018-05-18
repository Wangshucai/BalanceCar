#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define FILTERING_TIMES  10

 void Get_Angle(void);
 u16  Linear_Conversion(int moto);
 int balance(float Angle ,float Gyro);
 void Xianfu_Pwm(void);
 void Set_Pwm(int moto1,int moto2);
 
 int myabs(int a);
int velocity(int velocity_left,int velocity_right);
int turn(int velocity_left,int velocity_right);
int Incremental_PI (int Encoder,int Target);
#endif
