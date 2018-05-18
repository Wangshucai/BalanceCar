#ifndef _systick_h
#define _systick_h

#include "stm32f10x.h"
extern volatile u32 count;
void SysTick_Init(void);
void delay_us(u32 time);
void delay_ms(u32 time);

#endif
