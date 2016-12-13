#ifndef _WDT_H
#define _WDT_H
#include "nrf.h"

extern void WDT_Init(void);
void WDT_Start(void);
void WDT_stop(void);
void WDT_Feed(void);  
#endif
