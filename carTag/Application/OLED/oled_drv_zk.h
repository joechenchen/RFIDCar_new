#ifndef HAL_SPI_H__
#define HAL_SPI_H__

#include "nrf_gpio.h"
//#include "stdint.h"
#include "mxgpio.h"

#ifdef ZY_ZHUJI

//	#define	WREN	0x06
//	#define	RDSR	0x05
//	#define	READ	0x03
//	#define	PP		0x02
//	#define	SE		0xd8
//	#define	BE		0xc7

//	/******************M25PXX CS引脚配置,时钟引脚和数据输出引脚配置*******************************/
//	#define M25PXX_CS_H  NRF_GPIO->OUTSET = (1UL << m25pxx_cs_pin_num)	   //输出高电平
//	#define M25PXX_CS_L 	NRF_GPIO->OUTCLR = (1UL << m25pxx_cs_pin_num)    //输出低电平

//	#define SCK_IO_H_OUT 	NRF_GPIO->OUTSET = (1UL << OFlash_sclk_pin_num)  //输出高电平
//	#define SCK_IO_L_OUT 	NRF_GPIO->OUTCLR = (1UL << OFlash_sclk_pin_num)  //输出低电平

//	#define MOSI_IO_H_OUT NRF_GPIO->OUTSET = (1UL << OFlash_datai_pin_num)  //输出高电平
//	#define MOSI_IO_L_OUT NRF_GPIO->OUTCLR = (1UL << OFlash_datai_pin_num)  //输出低电平

//	//#define M25PXX_CS_IO_OUT  nrf_gpio_cfg_output(m25pxx_cs_pin_num)      //io配置
//	#define M25PXX_CS_IO_OUT  nrf_gpio_cfg_output_pull(m25pxx_cs_pin_num,NRF_GPIO_PIN_PULLUP)      //io配置
//	#define SCK_IO_DIR_OUT 	nrf_gpio_cfg_output(OFlash_sclk_pin_num)     
//	#define MOSI_IO_DIR_OUT nrf_gpio_cfg_output(OFlash_datai_pin_num)

//	/******************M25PXX SO引脚配置*******************************/
//	#define MISO_IO_DIR_IN   nrf_gpio_cfg_input(OFlash_cd_data_pin_num, NRF_GPIO_PIN_PULLUP)
//	#define M25PXX_READ_DATA        nrf_gpio_pin_read(OFlash_cd_data_pin_num)

//	/******************OLED C_/D 引脚配置*******************************/
//	#define OLED_D_C_H   nrf_gpio_pin_set(OFlash_cd_data_pin_num)   //高电平写数据
//	#define OLED_D_C_L   nrf_gpio_pin_clear(OFlash_cd_data_pin_num) //低电平写命令
//	#define OLED_C_D_DIR_OUT nrf_gpio_cfg_output(OFlash_cd_data_pin_num)
//	/******************OLED  CLK,SDA，CS电平定义*******************************/
//	#define OLED_SDA_H  nrf_gpio_pin_set(OFlash_datai_pin_num)
//	#define OLED_SDA_L  nrf_gpio_pin_clear(OFlash_datai_pin_num)
//	#define OLED_CLK_H  nrf_gpio_pin_set(OFlash_sclk_pin_num)
//	#define OLED_CLK_L  nrf_gpio_pin_clear(OFlash_sclk_pin_num)
//	#define OLED_CS_H   nrf_gpio_pin_set(oled_cs_pin_num)
//	#define OLED_CS_L   nrf_gpio_pin_clear(oled_cs_pin_num)
//	/******************OLED  字库引脚配置*******************************/
//	#define FLASH_CS_H nrf_gpio_pin_set(oled_zk_cs_pin_num)
//	#define FLASH_CS_L nrf_gpio_pin_clear(oled_zk_cs_pin_num)
//	#define Lcd_READ_DATA  nrf_gpio_pin_read(oled_zk_fso_pin_num)

//	#define OLED_X_Parameter			132	   			//LCD宽度
//	#define OLED_Y_Parameter			64			  	//LCD高度
//	#define ZK_InitAddr       	0x00000
//	#define ASCII_InitAdd				0x8100



//	extern void SPI_Init(void);
//	extern uint8_t hal_spi_read_write(uint8_t byte,uint8_t wr_e);
//	extern void ucfErBulk(void);
//	extern void ucfErSect(uint8_t *ucp);
//	extern void ucfWr(uint8_t *ucp1, uint8_t *ucp2, uint8_t uc);
//	extern void ucfRd(uint8_t *ucp1, uint8_t *ucp2, uint8_t uc);

//	#if OLED_ZK==1
//		extern void OLED_WrDat(uint8_t value);
//		extern void OLED_WrCmd(uint8_t cmd);
//		extern void OLED_SetPos(uint8_t x, uint8_t y);
//		extern void OLED_Fill(uint8_t bmp_dat);
//		extern void OLED_CLS(void);
//		extern void OLED_Init(void);
//		extern void LcdDisChar(uint8_t xPos,uint8_t yPos,uint8_t zknum,uint8_t *zkzip);
//		extern void FF_Init(void);
//		extern uint8_t Write_FF(uint8_t value);
//		extern void Read_FLASH(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
//		extern uint8_t LcdDisplay_HZ(uint8_t xPos,uint8_t yPos,uint8_t *GBCodeptr);
//		extern void LcdDisplay_Chinese(uint8_t xPos,uint8_t yPos,uint8_t *GBCodeptr);
//		extern void LcdDisplay_char(uint8_t xPos,uint8_t yPos,uint8_t *GBCodeptr);
//		extern void OLED_BMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint8_t BMP[]);
//		extern void test(void);
//	#endif
#endif

/*-------------------边界管理器头文件定义-----------------------*/
#ifdef BIAN_JIE
	/******************OLED C_/D 引脚配置*******************************/

	#define OLED_D_C_H   nrf_gpio_pin_set(oled_c_d_pin_num)   //高电平写数据
	#define OLED_D_C_L   nrf_gpio_pin_clear(oled_c_d_pin_num) //低电平写命令
	#define OLED_C_D_DIR_OUT nrf_gpio_cfg_output(oled_c_d_pin_num)
	/******************OLED CLK,SDI，CS电平定义*******************************/
	#define SCK_IO_DIR_OUT 	nrf_gpio_cfg_output(oled_sclk_pin_num)     
	#define MOSI_IO_DIR_OUT nrf_gpio_cfg_output(oled_sdi_pin_num)	
	#define OLED_SDA_H  nrf_gpio_pin_set(oled_sdi_pin_num)
	#define OLED_SDA_L  nrf_gpio_pin_clear(oled_sdi_pin_num)
	#define OLED_CLK_H  nrf_gpio_pin_set(oled_sclk_pin_num)
	#define OLED_CLK_L  nrf_gpio_pin_clear(oled_sclk_pin_num)
	#define OLED_CS_H   nrf_gpio_pin_set(oled_cs_pin_num)
	#define OLED_CS_L   nrf_gpio_pin_clear(oled_cs_pin_num)
	/******************OLED  字库引脚配置*******************************/
	#define OLED_ZK_CS_H nrf_gpio_pin_set(oled_zk_cs_pin_num)
	#define OLED_ZK_CS_L nrf_gpio_pin_clear(oled_zk_cs_pin_num)
	#define OLED_READ_DATA  nrf_gpio_pin_read(oled_zk_fso_pin_num)

	#define OLED_X_Parameter			132	   			//LCD宽度
	#define OLED_Y_Parameter			64			  	//LCD高度
	#define ZK_InitAddr       	0x00000
	#define ASCII_InitAdd				0x8100

	extern void SPI_Init(void);
	extern uint8_t hal_spi_read_write(uint8_t byte,uint8_t wr_e);
	extern void ucfErBulk(void);
	extern void ucfErSect(uint8_t *ucp);
	extern void ucfWr(uint8_t *ucp1, uint8_t *ucp2, uint8_t uc);
	extern void ucfRd(uint8_t *ucp1, uint8_t *ucp2, uint8_t uc);


	#if OLED_ZK==1
		extern void OLED_WrDat(uint8_t value);
		extern void OLED_WrCmd(uint8_t cmd);
		extern void OLED_SetPos(uint8_t x, uint8_t y);
		extern void OLED_Fill(uint8_t bmp_dat);
		extern void OLED_CLS(void);
		extern void OLED_Init(void);
		extern void LcdDisChar(uint8_t xPos,uint8_t yPos,uint8_t zknum,uint8_t *zkzip);
		extern void FF_Init(void);
		extern uint8_t Write_FF(uint8_t value);
		extern void Read_FLASH(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
		extern uint8_t LcdDisplay_HZ(uint8_t xPos,uint8_t yPos,uint8_t *GBCodeptr);
		extern void LcdDisplay_Chinese(uint8_t xPos,uint8_t yPos,uint8_t *GBCodeptr);
		extern void LcdDisplay_char(uint8_t xPos,uint8_t yPos,uint8_t *GBCodeptr);
		extern void OLED_BMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint8_t BMP[]);
		extern void test(void);
	#endif
#endif


#endif
