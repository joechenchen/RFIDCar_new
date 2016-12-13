#ifndef _ANT_125K_H
#define _ANT_125K_H
#include "nrf.h"
#include "sys.h"
void Stop_125k(void);
void Send_125k(void);

extern void sendPattern(void);
extern void sendPermanentData(u8* pdata,u8 len);
extern void settingsSetDefaultValues(void);
#endif

