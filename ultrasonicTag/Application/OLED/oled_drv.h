#ifndef __OLED_H
#define __OLED_H			  	 
#include "sys.h"
#include "stdlib.h"	    
#include "nrf_delay.h"
//OLED模式设置
//0:4线串行模式
		    						  
//-----------------OLED端口定义----------------  					   
    
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
#if OLED_ZK==0
#define OLED_CLS() OLED_Clear()
//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   
							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p,u8 size);
void show_battery(u8 x,u8 y,u8 bat_val,u8 mode);
void show_flash(u8 x,u8 y,u8 mode);
#endif
#endif  
	 



