#include "tim.h"
#include "nrf.h"


/*
fTIMER = 16 MHz/(2^PRESCALER)
1US*50000 = 50MS?
*/
void Timer0_Init(uint8_t delayms)
{
		NRF_TIMER0->SHORTS     = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER0->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
	  NRF_TIMER0->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.
    NRF_TIMER0->PRESCALER      = 4;                            // 1us
    NRF_TIMER0->BITMODE        = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos); // 16 bit mode.
    NRF_TIMER0->INTENSET      = (TIMER_INTENSET_COMPARE0_Enabled<<TIMER_INTENSET_COMPARE0_Pos);
		NRF_TIMER0->CC[0]       = (uint32_t)delayms * 1000;
//	  NRF_TIMER0->TASKS_START    = 1; // Start timer. 
	  NVIC_SetPriority(TIMER0_IRQn, 1);
	 	NVIC_EnableIRQ(TIMER0_IRQn);
}

/*
fTIMER = 16 MHz/(2^PRESCALER)
1US*50000 = 50MS?
*/
void Timer1_Init(void)
{
    NRF_TIMER1->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
	  NRF_TIMER1->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.
    NRF_TIMER1->PRESCALER      = 4;                            // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us.
    NRF_TIMER1->BITMODE        = (TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos); // 16 bit mode.
    NRF_TIMER1->CC[1]          = 50000; 
    NRF_TIMER1->INTENSET      = (TIMER_INTENSET_COMPARE1_Enabled<<TIMER_INTENSET_COMPARE1_Pos);
	  NRF_TIMER1->TASKS_START    = 1; // Start timer. 
	  NVIC_SetPriority(TIMER1_IRQn, 2);
	  NVIC_EnableIRQ(TIMER1_IRQn);
}

/*
fTIMER = 16 MHz/(2^PRESCALER)
1US*50000 = 50MS?
*/
void Timer2_Init(void)
{
    NRF_TIMER2->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
	  NRF_TIMER2->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.
    NRF_TIMER2->PRESCALER      = 4;                            // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us.
    NRF_TIMER2->BITMODE        = (TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos); // 16 bit mode.
    //NRF_TIMER2->CC[2]          = 50000; 
    NRF_TIMER2->INTENSET      = (TIMER_INTENSET_COMPARE2_Enabled<<TIMER_INTENSET_COMPARE2_Pos);
	  //NRF_TIMER2->TASKS_START    = 1; // Start timer. 
	  NVIC_SetPriority(TIMER2_IRQn, 0);
	  //NVIC_EnableIRQ(TIMER2_IRQn);
}

