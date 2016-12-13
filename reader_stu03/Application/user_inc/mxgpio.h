#ifndef MX_GPIO_
#define MX_GPIO_
#include "nrf_gpio.h"
#define st03u
//引脚定义见引脚定义.txt文档
#ifdef ZY_ZHUJI
//	/*-------------OLED模块引脚定义-------------------*/
//	//#define oled_sclk_pin_num 17  //OLED时钟 out
//	//#define oled_data_pin_num 16  //OLED数据输入 out 
//	//#define oled_c_d_pin_num 15   //OLED命令/数据 out
//	#define oled_cs_pin_num 14    //CS屏片选信号 out

//	#define oled_zk_cs_pin_num 12 //字库FLASH片选信号  out
//	#define oled_zk_fso_pin_num 13 //字库FLASH数据输出 in
//	#define oled_power_num 18
//	/*-------------M25P16 2M FLASH引脚定义-----------*/
//	//#define m25pxx_clk_pin_num 17    //时钟信号 out
//	//#define m25pxx_datai_pin_num 16  //FLASH数据输入 out
//	//#define m25pxx_datao_pin_num 15  //FLASH数据输出 in
//	#define m25pxx_cs_pin_num 0      //FLASH片选信号 out

//	/*-------------M25P16与OLED共用引脚定义----------*/
//	#define OFlash_sclk_pin_num 17  //时钟 out
//	#define OFlash_datai_pin_num 16  //外设数据输入 out 
//	#define OFlash_cd_data_pin_num 15  //外设数据输出 ,输出，输入之间切换，当OLED工作是配置成输出，当FLASH工作时配置成输入

//	/*-------------按钮接口-------------------------*/
//	#define KeyLED5V_pin_num 10
//	#define KEY_pin_num 11
//	/*------------主控板--------------------*/

//	#define LED0V_pin_num 7

//	/*------------北斗电源控制引脚---------------*/
//	#define GPS_power_pin_num 30      //out
//	#define GPS_txd_pin_num 3         //in
//	#define GPS_DATA 3
//	/*------------WIFI电源控制---------------*/
//	#define Wifi_power_pin_num 2
//	/*------------4G电源控制---------------*/
//	#define P_4G_pin_num 29
//	/*------------摄像头控制---------------*/
//	#define Camera_pin_mun 6

//	/*------------测试引脚定义-------------*/
//	#define test25_pin_num 25
//	#define test1_pin_num 1

//	/*------------2.4g功放板-------------------------*/
//	//RW=0-接收  RW=1-发送		 
//	#define RFN_RW_pin_num 19
//	#define RFN_TX_MODE 	nrf_gpio_pin_set(RFN_RW_pin_num)	
//	#define RFN_RX_MODE 	nrf_gpio_pin_clear(RFN_RW_pin_num)	   //OUT
//	#define RFN_EN_pin_num 20
//	/*----------------串口--------------------------*/
//	#define RX_PIN_NUMBER  8    // UART RX pin number.
//	#define TX_PIN_NUMBER 9    // UART TX pin number.
//	#define CTS_PIN_NUMBER 1   // UART Clear To Send pin number. Not used if HWFC is set to false
//	#define RTS_PIN_NUMBER 25    // Not used if HWFC is set to false 
//	#define HWFC           false // UART hardware flow control
#endif
	
#ifdef BIAN_JIE
//		/*-------------OLED模块引脚定义-------------------*/
//	#define oled_sclk_pin_num 1  //OLED时钟 out
//	#define oled_sdi_pin_num 0  //OLED数据输入 out 
//	#define oled_c_d_pin_num 30   //OLED命令/数据 out
//	#define oled_cs_pin_num 29    //CS屏片选信号 out

//	#define oled_zk_cs_pin_num 25 //字库FLASH片选信号  out
//	#define oled_zk_fso_pin_num 28 //字库FLASH数据输出 in
//	#define oled_power_num 02
//	/*-----------MOSFET H-BRIDGE----------------*/
//	//P0.03 P0.04高，p0.18,P0.19低，天线2->1导通；P0.03 P0.04低p0.18,P0.19高，天线1->2导通
//	#define TC4426_PMOS1_PIN_NUM 19
//	#define TC4426_NMOS1_PIN_NUM 18
//	#define TC4426_PMOS1_ON NRF_GPIO->OUTSET = (1UL << TC4426_PMOS1_PIN_NUM)     //1
//	#define TC4426_PMOS1_OFF NRF_GPIO->OUTCLR = (1UL << TC4426_PMOS1_PIN_NUM) //0
//	#define TC4426_NMOS1_ON NRF_GPIO->OUTCLR = (1UL << TC4426_NMOS1_PIN_NUM)   //0
//	#define TC4426_NMOS1_OFF NRF_GPIO->OUTSET = (1UL << TC4426_NMOS1_PIN_NUM)   //1
//	
//	
//	//	  NRF_GPIO->OUTCLR = (1UL << 19);//-----------nrf_gpio_pin_clear
////	  NRF_GPIO->OUTSET = (1UL << 4);//-----------nrf_gpio_pin_set
////	
////	  NRF_GPIO->OUTCLR = (1UL << 18);//-----------nrf_gpio_pin_clear
////	  NRF_GPIO->OUTSET = (1UL << 3);//-----------nrf_gpio_pin_set
//  //自定义正向导通：天线2脚到1脚导通
//	#define TC4426_PMOS2_PIN_NUM 3
//	#define TC4426_NMOS2_PIN_NUM 4
//	#define TC4426_PMOS2_ON NRF_GPIO->OUTSET = (1UL << TC4426_PMOS2_PIN_NUM)  //TC4426高->PMOS低 导通
//	#define TC4426_PMOS2_OFF NRF_GPIO->OUTCLR = (1UL << TC4426_PMOS2_PIN_NUM) 
//	#define TC4426_NMOS2_ON NRF_GPIO->OUTCLR = (1UL << TC4426_NMOS2_PIN_NUM) 
//	#define TC4426_NMOS2_OFF NRF_GPIO->OUTSET = (1UL << TC4426_NMOS2_PIN_NUM)

//	#define PMOS2NMOS1ON()   {TC4426_NMOS2_OFF;TC4426_PMOS1_OFF;TC4426_PMOS2_ON;TC4426_NMOS1_ON;}//PMOS2NMOS1ON
//	#define PMOS1NMOS2ON()   {TC4426_PMOS2_OFF;TC4426_NMOS1_OFF;TC4426_NMOS2_ON;TC4426_PMOS1_ON;}//PMOS1NMOS2ON
//  #define ALLOFF() {TC4426_PMOS1_OFF;TC4426_PMOS2_OFF;TC4426_NMOS1_ON;TC4426_NMOS2_ON;}//ALLOFF
//	/*-----------档位开关引脚定义，低电平有效，同时只能有一个导通----------------*/	
//	#define KEY1 nrf_gpio_pin_read(21)
//	#define KEY2 nrf_gpio_pin_read(22)
//	#define KEY3 nrf_gpio_pin_read(23)
//	#define KEY4 nrf_gpio_pin_read(24)
//	#define KEY1_PIN_NUM 21
//	#define KEY2_PIN_NUM 22
//	#define KEY3_PIN_NUM 23
//	#define KEY4_PIN_NUM 24
//	/*----------LED灯----------------------------------*/
//	#define LED_PIN_NUM 17
#endif

//	/*-------------按钮接口-------------------------*/
//	#define KeyLED5V_pin_num 10
//	#define KEY_pin_num 11
//	/*------------主控板--------------------*/
//	#define LED0V_pin_num 7
//	/*------------测试引脚定义-------------*/
//	#define test25_pin_num 25
//	#define test1_pin_num 1

	/*----------------串口--------------------------*/
//	#define RX_PIN_NUMBER  8    // UART RX pin number.
//	#define TX_PIN_NUMBER 9    // UART TX pin number.
//	#define CTS_PIN_NUMBER 1   // UART Clear To Send pin number. Not used if HWFC is set to false
//	#define RTS_PIN_NUMBER 25    // Not used if HWFC is set to false 
//	#define HWFC           false // UART hardware flow control
#ifdef st03u
	/*----------------串口--------------------------*/
	#define RX_PIN_NUMBER  4    // UART RX pin number.
	#define TX_PIN_NUMBER 5   // UART TX pin number.
	#define CTS_PIN_NUMBER 6   // UART Clear To Send pin number. Not used if HWFC is set to false
	#define RTS_PIN_NUMBER 7    // Not used if HWFC is set to false 
	#define HWFC           false // UART hardware flow control
	#define LED     8
	#define LED_OFF NRF_GPIO->OUTSET = (1UL << LED)
	#define LED_ON NRF_GPIO->OUTCLR = (1UL << LED)
	
	
	/*------------2.4g功放板-------------------------*/
	//RW=0-接收  RW=1-发送	
	#if LNA_Board		
		#define RFN_RW_pin_num 19
		#define RFN_TX_MODE 	nrf_gpio_pin_set(RFN_RW_pin_num)	
		#define RFN_RX_MODE 	nrf_gpio_pin_clear(RFN_RW_pin_num)	   //OUT
		#define RFN_EN_pin_num 20
	#endif
#endif
	
#ifdef pingguoban
	/*----------------串口--------------------------*/
	#define RX_PIN_NUMBER  6    // UART RX pin number.
	#define TX_PIN_NUMBER 5   // UART TX pin number.
	#define CTS_PIN_NUMBER 1   // UART Clear To Send pin number. Not used if HWFC is set to false
	#define RTS_PIN_NUMBER 25    // Not used if HWFC is set to false 
	#define HWFC           false // UART hardware flow control
	#define LED     8
	#define LED_OFF NRF_GPIO->OUTSET = (1UL << LED)
	#define LED_ON NRF_GPIO->OUTCLR = (1UL << LED)
	
	
	/*------------2.4g功放板-------------------------*/
	//RW=0-接收  RW=1-发送	
	#if LNA_Board		
		#define RFN_RW_pin_num 19
		#define RFN_TX_MODE 	nrf_gpio_pin_set(RFN_RW_pin_num)	
		#define RFN_RX_MODE 	nrf_gpio_pin_clear(RFN_RW_pin_num)	   //OUT
		#define RFN_EN_pin_num 20
	#endif
#endif	

#endif

