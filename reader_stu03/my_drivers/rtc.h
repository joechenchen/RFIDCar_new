#ifndef M_RTC_H__
#define M_RTC_H__

#include"nrf.h"

uint8_t BCDInc(uint8_t *ucByte, uint8_t ucMin, uint8_t ucMax);
uint8_t DateMaxCalc21Cn(uint8_t ucBcdYeah, uint8_t ucBcdMonth);
void rtc_Init(void);
#endif

