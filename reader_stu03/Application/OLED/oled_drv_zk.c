#include "oled_drv_zk.h"
#include "string.h"
#include "nrf_delay.h"


void SPI_Init(void)
{
//		nrf_gpio_cfg_output_pull(oled_cs_pin_num,NRF_GPIO_PIN_PULLUP);		
	  nrf_gpio_cfg_output(oled_cs_pin_num);//oled cs引脚设置输出

		nrf_gpio_cfg_output(oled_sclk_pin_num);//oled sclk 引脚
		nrf_gpio_cfg_output(oled_sdi_pin_num);//oled sdi 引脚
		nrf_gpio_cfg_output(oled_zk_cs_pin_num);//字库 cs引脚设置输出
	  
	  nrf_gpio_cfg_output(oled_power_num);//oled vcc引脚设置输出
		
		nrf_gpio_cfg_input(oled_zk_fso_pin_num,NRF_GPIO_PIN_NOPULL);//字库数据输入设为输入
	
	  nrf_gpio_pin_set(oled_power_num);  //打开OLED电源
	
	  OLED_ZK_CS_H;
	  OLED_CLK_H;
}	 

#if OLED_ZK==1
void OLED_WrDat(uint8_t value)
{
	uint8_t i;
	OLED_C_D_DIR_OUT;
	OLED_D_C_H;
	for(i=0;i<8;i++)
	{
		if((value << i) & 0x80)
		{
			OLED_SDA_H;
		}
		else
			OLED_SDA_L;
		OLED_CLK_L;
		OLED_CLK_H;
	}
}

void OLED_WrCmd(uint8_t cmd)//写命令
{
	uint8_t i;
	OLED_C_D_DIR_OUT;
	OLED_D_C_L;
	for(i=0;i<8;i++) //发送一个八位数据
	{
		if((cmd << i) & 0x80)
		{
			OLED_SDA_H;
		}
		else
		{
			OLED_SDA_L;
		}
	  OLED_CLK_L;
		OLED_CLK_H;
	}
}

void OLED_SetPos(uint8_t x, uint8_t y)//设置起始点坐标
{
	OLED_WrCmd(0xb0 + y);
	OLED_WrCmd(((x&0xf0)>>4)|0x10);
	OLED_WrCmd((x&0x0f)|0x01);
}

void OLED_Fill(uint8_t bmp_dat)//全屏填充
{
	uint8_t y,x;
	
	OLED_ZK_CS_H;
	nrf_delay_us(100);
	OLED_CS_L;
	for(y=0;y<8;y++)
	{
		OLED_WrCmd(0xb0+y); //设置页地址（0-7）
		OLED_WrCmd(0x02);   //设置显示位置-列低地址
		OLED_WrCmd(0x10);   //设置显示位置-列高地址
		for(x=0;x<OLED_X_Parameter;x++)
		{
			OLED_WrDat(bmp_dat);
		}
	}
	OLED_CS_H;
}

void OLED_CLS(void)//清屏
{
	OLED_Fill(0x00);
}

void OLED_Init(void)
{
	

	OLED_ZK_CS_H;
	nrf_delay_ms(500);
	OLED_CS_L;
	
	OLED_WrCmd(0xAE);    /*display off*/
	
	OLED_WrCmd(0x02);    /*set lower column address*/
	OLED_WrCmd(0x10);    /*set higher column address*/
	
	OLED_WrCmd(0x40);    /*set display start line*/
	
	OLED_WrCmd(0xB0);    /*set page address*/
	
	OLED_WrCmd(0x81);    /*contract control*/
	OLED_WrCmd(0x80);    /*128*/
	
	OLED_WrCmd(0xA1);    /*set segment remap*/
	
	OLED_WrCmd(0xA6);    /*normal / reverse*/
	
	OLED_WrCmd(0xA8);    /*multiplex ratio*/
	OLED_WrCmd(0x3F);    /*duty = 1/32*/
	
	OLED_WrCmd(0xad);    /*set charge pump enable*/
	OLED_WrCmd(0x8b);     /*    0x8a    外供VCC   */
	
	OLED_WrCmd(0x30);    /*0X30---0X33  set VPP   9V liangdu!!!!*/
	
	OLED_WrCmd(0xC8);    /*Com scan direction*/
	
	OLED_WrCmd(0xD3);    /*set display offset*/
	OLED_WrCmd(0x00);   /*   0x20  */
	
	OLED_WrCmd(0xD5);    /*set osc division*/
	OLED_WrCmd(0x80);    
	
	OLED_WrCmd(0xD9);    /*set pre-charge period*/
	OLED_WrCmd(0x1f);    /*0x22*/
	
	OLED_WrCmd(0xDA);    /*set COM pins*/
	OLED_WrCmd(0x12);    //0x02 -- duanhang xianshi,0x12 -- lianxuhang xianshi!!!!!!!!!
	
	OLED_WrCmd(0xdb);    /*set vcomh*/
	OLED_WrCmd(0x40);     
	
	OLED_WrCmd(0xAF);    /*display ON*/ 
	OLED_Fill(0x00);
	OLED_CS_H;
}


void LcdDisChar(uint8_t xPos,uint8_t yPos,uint8_t zknum,uint8_t *zkzip)
{
	uint8_t i;
	OLED_SetPos(xPos,yPos);
	for(i=0; i<zknum;i++)
	{
		OLED_WrDat(zkzip[i]); 
	}
	OLED_SetPos(xPos,yPos+1);
	for(i=zknum; i<zknum*2;i++)
	{
		OLED_WrDat(zkzip[i]);
	}
}

void FF_Init(void)
{
	OLED_ZK_CS_H;
	OLED_CLK_H;
}


uint8_t Write_FF(uint8_t value)
{
	uint8_t i;
	uint8_t temp=0;
	OLED_CLK_H;
	for(i=0;i<8;i++)
	{
	 OLED_CLK_L;
	 nrf_delay_us(2);
	 if((value&0x80)==0x80)
		 OLED_SDA_H;
	 else
		 OLED_SDA_L; 
	 value<<=1;
	 nrf_delay_us(2);
	 OLED_CLK_H;
	 nrf_delay_us(2);
	 temp<<=1;
	 if(OLED_READ_DATA) 
			temp++;
	}
	return(temp);
}


void Read_FLASH(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{

	nrf_delay_us(100);
	OLED_CS_H;//取消OLED选中
	OLED_ZK_CS_L;//选中FLASH芯片
	nrf_delay_us(100);
	Write_FF(0x03);
	Write_FF((ReadAddr & 0xFF0000) >> 16);
	Write_FF((ReadAddr& 0xFF00) >> 8);
	Write_FF(ReadAddr & 0xFF);
	while(NumByteToRead--)
	{
		*pBuffer = Write_FF(0xA5);
		pBuffer++;
	}
	OLED_ZK_CS_H;
	OLED_CS_L;
}


uint8_t LcdDisplay_HZ(uint8_t xPos,uint8_t yPos,uint8_t *GBCodeptr)
{
	uint8_t msb,lsb,zknum;
	uint8_t zkzip[32];  //读取字库数据的缓存区	
	uint32_t offset;	   //字库地址索引
//	uint8_t oled_i=0,oled_j=0;

	OLED_ZK_CS_H;
	nrf_delay_us(100);
	OLED_CS_L;
	if(xPos>=OLED_X_Parameter || yPos>=OLED_Y_Parameter) return 0 ;  //超范围退出
	msb= *GBCodeptr;     //汉字或ASCII的机器码的低8位。
	lsb= *(GBCodeptr+1); //汉字或ASCII的机器码的高8位。
	if (msb>128 && lsb>128)	//表明为汉字
	{
		if(xPos+16>OLED_X_Parameter || yPos+16>OLED_Y_Parameter)return 0; //超范围退出
		offset =0x00000+((msb-0xA1)*94+(lsb-0xa1))*32;//具体算法详细查看字库原理
		zknum =16;	//汉字为16*16的字库
	}
	else	               //否则为ASCII码
	{
		if(xPos+8>OLED_X_Parameter || yPos+16>OLED_Y_Parameter)return 0;	//超范围退出
		offset =0x8100+(msb-32)*16;  //查看提供的2012_KZ.txt文档中的“!”的首地址
		zknum =8;   // ASCII码位8*16的字库
	}
//	for(oled_i=0;oled_i<32;oled_i++)
//		zkzip[oled_i] = 0x00;
	Read_FLASH(zkzip,offset,zknum*2);	 //从FLASH中读取字库数据。
	//这部分代码暂时用来解决左下角有光标的问题，FLASH读出全是0XFF
//	for(oled_i=0;oled_i<16;oled_i++)
//	{
//		if(0xff == zkzip[oled_i])
//		{
//			oled_j++;
//		}
//	}
	
//	if(oled_j!=16)
		LcdDisChar(xPos,yPos,zknum,zkzip);
//	else{
//		oled_j = 0;
//	}
	OLED_CS_H;

	return 1;
}

//void test(void)
//{
//	uint8_t zkzip[32];  //读取字库数据的缓存区	
//	Read_FLASH(zkzip,0x8100,8*2);	 //从FLASH中读取字库数据。
//	LcdDisChar(0,6,8,zkzip);
//}
void LcdDisplay_Chinese(uint8_t xPos,uint8_t yPos,uint8_t *GBCodeptr)
{
	uint8_t i,len;
	len =  strlen((const char*)GBCodeptr);
	for(i=0;i<len;i++)	
	{
	  LcdDisplay_HZ(xPos+i*8,yPos,GBCodeptr+i);
		i++;
	}
}

void LcdDisplay_char(uint8_t xPos,uint8_t yPos,uint8_t *GBCodeptr)
{
  uint8_t i, len;
	len =  strlen((const char*)GBCodeptr);
  for(i=0;i<len;i++)	
	{
		LcdDisplay_HZ(xPos+i*8,yPos,GBCodeptr+i);
	}
}

//------------------------------------------------------
//void OLED_BMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1,unsigned char BMP[])
//在指定区域显示BMP图片 
//x0(0~131),yo(0~7) -- 图片起始坐标，x1(1~132),y1(1~8)图片结束点坐标
//将BMP图片导入取模软件生成字模,再将字模放到本工程的codetab.h中
//------------------------------------------------------
void OLED_BMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint8_t BMP[])
{
	uint16_t j=0;
	uint8_t x,y;
	OLED_ZK_CS_H;  

	nrf_delay_us(100);
	OLED_CS_L;
  if(y1%8==0) y=y1/8;
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_SetPos(x0,y);
    for(x=x0;x<x1;x++)
		{
			OLED_WrDat(BMP[j++]);
		}
	}
	OLED_CS_H;
}

#endif






