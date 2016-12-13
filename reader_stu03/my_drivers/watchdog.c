#include "watchdog.h"


void WDT_Init(void)
{
	//NRF_WDT->TASKS_START = 0;
	NRF_WDT->CRV = 0xF0000-1; //Æô¶¯¿´ÃÅ¹·£º30Ãë
	NRF_WDT->CONFIG =  WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos | WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos;
	NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;
}

void WDT_Start(void)
{
	NRF_WDT->TASKS_START =  1 ; 
}

void WDT_Feed(void)  
{  
	if(NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk)  
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;  
}  
  
void WDT_Stop(void)  
{  
	NRF_WDT->TASKS_START = 0;  
} 

