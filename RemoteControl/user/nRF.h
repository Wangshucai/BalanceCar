#ifndef _NRF_H
#define _NRF_H

#include "stm32f10x.h"

void NRF_Config(void);

void RX_Mode(void);
void NRF_SendPacket(u8* tfbuf);
void NRF_ReceivePacket(u8* rx_buf);

#endif

