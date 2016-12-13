#ifndef OLED_H
#define OLED_H

#include "nrf.h"
typedef enum
{
	POWER_STATE,
	CONNECT_STATE,
	ONLINE_STATE,
	OFFLINE_STATE,
	SHUTDOWN_STATE
}Work_State;
extern void OLED_SHOW(uint8_t En_Lcd_Flag,uint8_t GPS_Flag,uint8_t ADC_FLAG,uint8_t ADC_DD_FLAG,uint8_t charge_flag1,uint8_t Work_Mode,uint8_t ADC_DL,uint8_t * caucAddr2,uint8_t* aucRTC);
#endif
