#include "ANT_125K.h"
#include "nrf_delay.h"
#include "mxgpio.h"

//as3933
u16 gBitDuration;
u16 gCarrierBurstDuration;
u16 gNrPreambleSymbols;
u8 gManchesterPatternLength;
u8 gCorrelatorDoublePattern;
u32 gPattern;
extern uint8_t DATA_125K_tx[12];//曼彻斯特码元，两bit对应一个曼彻斯特码

// cycles * 125 kHz @ 16 MHz
void sendCarrier(u16 cycles)
{
		while(cycles--)
		{
				PMOS2NMOS1ON();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();//4.1us
				__NOP();__NOP();//4.22us
				PMOS1NMOS2ON();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();//4.1us
				__NOP();__NOP();//4.22us
		}
		ALLOFF();
}

// cycles * 8 us @ 16 MHz
void pauseCarrier(u16 cycles)
{	
		while(cycles--)
		{
				ALLOFF();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				ALLOFF();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		ALLOFF();
}

void settingsSetDefaultValues(void)
{
	// init default values
	gBitDuration = 54;   // default value in AS3933 (12 in register 7[4:0]
	gCarrierBurstDuration = 350;   // duration of the carrier burst (250 == 2 ms carrier burst),scan mode最小时间80Tclk + 16Tcarr
	gNrPreambleSymbols = 5;   // Number of Preamble Symbols， 3.5ms R7[4:0]*Tclk*2 = 3.5MS
	gManchesterPatternLength = 16;   // 16 bit
	gCorrelatorDoublePattern = 0;   // 0 = single pattern, 1 = double pattern
	gPattern = 0x96695555;
}


void sendPattern(void)
{
		u32 cIndex, index;
		u32 pattern;
		u16 bitDuartionDouble;
		u8 nrPreambleSymbols, repeatPattern;
//		u16 current_cpu_ipl;

		/* the USB interrupt is not handled via disi command -> use a "more direct" way */
//		SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);  /* disable interrupts */
		bitDuartionDouble = gBitDuration << 1;
		nrPreambleSymbols = gNrPreambleSymbols;
		pattern = gPattern;
		repeatPattern = gCorrelatorDoublePattern;

		if(gManchesterPatternLength == 16)
		{
			cIndex = 0x8000;
			pattern >>= 16;
		}
		else
		{
			cIndex = 0x80000000;
		}
		index = cIndex;

		sendCarrier(gCarrierBurstDuration);   // carrier burst

		pauseCarrier(gBitDuration);   // 0, separation bit

		while(nrPreambleSymbols--)
		{
			sendCarrier(gBitDuration);   // 1, preamble
			pauseCarrier(gBitDuration);   // 0, preamble
		}

		// pattern 9669 (HEX) = 1001 0110 0110 1001
		// loop optimized for "real" manchester coding (only two times the same bit value at once)
		do
		{
			while(index)
			{
				if(pattern & index)
				{
					if(pattern & (index >> 1))
					{
						sendCarrier(bitDuartionDouble);
						index >>= 2;
					}
					else
					{
						sendCarrier(gBitDuration);
						index >>= 1;
					}
				}
				else
				{
					if(pattern & (index >> 1))
					{
						pauseCarrier(gBitDuration);
						index >>= 1;
					}
					else
					{
						pauseCarrier(bitDuartionDouble);
						index >>= 2;
					}
				}
			}
			index = cIndex;
		}while(repeatPattern--);   // repeat the pattern

//		RESTORE_CPU_IPL(current_cpu_ipl);

}


void sendPermanentData(u8* pdata,u8 len)
{
		u8 ilen,BitCnt;
	  u8 data;
		for(ilen=0;ilen<len;ilen++)
		{
			data = *pdata++;
			for(BitCnt=0;BitCnt<8;BitCnt++)
			{
				if(data&0x80)
				{
					sendCarrier(gBitDuration);
				}
				else
				{
					pauseCarrier(gBitDuration);
				}				 
				data<<=1;
			}
		}
//		while(len)
//		{
//			if(data & index)
//			{
//				if(data & (index >> 1))
//				{
//					sendCarrier(bitDuartionDouble);
//					index >>= 2;
//				}
//				else
//				{
//					sendCarrier(gBitDuration);
//					index >>= 1;
//				}
//			}
//			else
//			{
//				if(data & (index >> 1))
//				{
//					pauseCarrier(gBitDuration);
//					index >>= 1;
//				}
//				else
//				{
//					pauseCarrier(bitDuartionDouble);
//					index >>= 2;
//				}
//			}
//			if(0 == index)
//			{
//				len--;
//				index = 0x80;
//				data = *(pdata++);	
//			}
//		}
}

//void Send_125k(void)
//{
//	  NRF_GPIO->OUTCLR = (1UL << 19);//-----------nrf_gpio_pin_clear
//	  NRF_GPIO->OUTSET = (1UL << 4);//-----------nrf_gpio_pin_set
//	
//	  NRF_GPIO->OUTCLR = (1UL << 18);//-----------nrf_gpio_pin_clear
//	  NRF_GPIO->OUTSET = (1UL << 3);//-----------nrf_gpio_pin_set
//		
//	  __NOP();
//	  __NOP();
//	  __NOP();
//	  __NOP();
//	
//	  __NOP();
//	  __NOP();
//	  __NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//	  		
//		NRF_GPIO->OUTCLR = (1UL << 3);//-----------nrf_gpio_pin_clear
//    NRF_GPIO->OUTSET = (1UL << 18);//-----------nrf_gpio_pin_set
//		
//		NRF_GPIO->OUTCLR = (1UL << 4);//-----------nrf_gpio_pin_clear
//		NRF_GPIO->OUTSET = (1UL << 19);//-----------nrf_gpio_pin_set
//	 		 	
//	  __NOP();
//	  __NOP();
//	  __NOP();
//	  __NOP();
//		
//	  __NOP();
//		__NOP();
//		__NOP();
//	  __NOP();	

//    __NOP();
//	  __NOP();
//	  __NOP();
//	  __NOP();
//		
//	  __NOP();
//		__NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();
//		
//		__NOP();
//	  __NOP();
//		__NOP();
//	  __NOP();

//}

//void Stop_125k(void)
//{	
//	  NRF_GPIO->OUTCLR = (1UL << 19);
//	  NRF_GPIO->OUTCLR = (1UL << 3);
//	  NRF_GPIO->OUTSET = (1UL << 18);
//	  NRF_GPIO->OUTSET = (1UL << 4);
//		
//	  __NOP();
//	  __NOP();
//	  __NOP();
//	  __NOP();
//	
//	  __NOP();
//	  __NOP();
//	  __NOP();
//	  __NOP();
//	
//	  __NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//		__NOP();
//	  __NOP();
//	  __NOP();
//		__NOP();
//		
//					  
//}







































