#include "nrf.h"
#include "nrf_gpio.h"
#include "mxgpio.h"
#include "radio_config.h"
#include "simple_uart.h"
#include "tim.h"
#include "nrf_nvmc.h"
#include "sys.h"
#include "stdlib.h"
/*
USTagID 超声波标签ID
carTagID 巡检标签ID
USAge超声波标签离开事件
USAge us leave time
*/		
#if 0
#define	CAPACITY	1		//标签容量
struct {
	uint8_t carTagID[4];//car tag
	uint8_t	carSta[2];   //状态
	uint8_t RSSI;
	uint8_t baseboard[2];
}astRFID[CAPACITY];	
#endif

extern uint8_t radio_status;
extern uint8_t payload[PACKET_PAYLOAD_MAXSIZE];

//标签ID  __attribute__((at(0x3d100)))
//uint8_t TagID[4] = {0xff,0xff,0xff,0xff};
#define 	ID_BEGIN	0x3D000					//1KB,硬件信息区,前4字节放ID号

//以下几个参数用来记录ROM中的16/32块（偏移量）的计数值 值16/32，表示记录个数，0表示未记录
//255扇区打标记0x3fc00
//254扇区rom0 存储频道参数  0x3f800
//253扇区rom1 保留区 0x3f400
//252扇区rom2 用户区1 0x3f00
//251扇区rom3 用户区2 0x3ec00
uint8_t ucRomMark;//倒数第一个区打标机
uint8_t	ucROM0;		//倒数第二个区，内部参数区
uint8_t	ucROM1;		//倒数第三个扇区，保留区
uint8_t	ucROM2;	  //倒数第四个扇区，用户区1
uint8_t	ucROM3;   //倒数第五个扇区,用户区2
uint8_t *pROM;	//记录指针
uint8_t	* caucpROM[]={&ucROM0,&ucROM1,&ucROM2,&ucROM3};	
uint32_t Page_Base[5];//page addr
#define para_area 1
#define reserved_area 2
#define user_area1 3
#define user_area2 4
//1、2区16条记录，3、4区32条记录
//每条记录16个字节
uint8_t Rom_record_size[4] = {16,16,32,32};//4个扇区对应的记录个数
uint8_t Rom_record_byte[4] = {16,16,16,16};//每条记录对应的字节数
#define Rom0_record_size 16
#define Rom1_record_size 16
#define Rom2_record_size 32
#define Rom3_record_size 32
#define Rom0_record_byte 16
#define Rom1_record_byte 16
#define Rom2_record_byte 16
#define Rom3_record_byte 16
//uint8_t nvtempRom0[Rom0_record_size][Rom0_record_byte];
//uint8_t nvtempRom1[Rom1_record_size][Rom1_record_byte];
//uint8_t nvtempRom2[Rom2_record_size][Rom2_record_byte];
//uint8_t nvtempRom3[Rom3_record_size][Rom3_record_byte];


//flash参数记录
//代码存储区，大小根据mcu型号决定.
/*FICR寄存器中的CODEPAGESIZE对应着页个数，CODESIZE对应页包含的memory大小
CODEPAGESIZE*CODESIZE即为ROM的大小,pg_size=1024,pg_num = 256,256KB的代码存储区，FLASH。
代码存储区的最后一页存储用户配置信息*/
uint32_t pg_size;//一页对应的字节数
uint32_t pg_num;//页个数
uint32_t addr;
//user-defined MARK
const uint8_t nvmc_flash_mark[11]={0xaa,0xaa,0x01,0x03,0x05,0x01,0x23,0x45,0x67,0x89,0x8e};

/*
[5]	.7~0 	79		数据频道
[6]	.7~0	5		配置频道
[7]	.7~0	0~16	目标短号：0~15-静态接入目标；16-动态接入	
[8]	.7~6	0		保留
.7~4	6		发射功率：0- (-30dBm), 1- (-20dBm), 2- (-16dBm), 3- (-12dBm)
													4- (8dBm), 5- (4dBm), 6- (0dBm), 7- (4dBm)
.3~2	0~2		自动上报携带的信息来源：0-保留区；1-用户区1；2-用户区2
.1~0	0~2		工作模式：0-保存模式；1-活动模式；2-读写模式
[9~12]				4字节测试目标ID，默认值FFFFFFFA（非测试模式）
[13]	.7~4	0~15	传感类型：0-无；1--温度
.3~0	0~2		传感参数-振动灵敏度：值越小越灵敏
[14]	.7~6	0~2		传感参数-采样周期-时间单位：0-秒；1-分；2-时
.5~0	0~60	传感参数-采样周期-时间数值：0-关闭传感器		
[15]					保留
*/

//5字节射频地址，数据频道，配置频道,发射功率
#define	HWTYP		0 		//0-苹果（无衰减），1-短红板（单衰减）
#define	VERSION		0x02
#define data_channel 79//数据频道，标签自动上报 
#define config_channel 5//配置频道
uint8_t const ParaSet[16]={0xe7,0xe7,0xe7,0xe7,0xe7,79,5,16,0x61,0xff,0xff,0xff,0xfa,0,0,0};	//默认参数
//[0]~[6] read-only
//[7]~[15] can write
uint8_t CurParaSet[16];//当前运行参数,等于ParaSet的参数
//发射功率0,
//#define RADIO_TXPOWER_TXPOWER_0dBm (0x00UL) /*!< 0dBm. */
//#define RADIO_TXPOWER_TXPOWER_Pos4dBm (0x04UL) /*!< +4dBm. */
//#define RADIO_TXPOWER_TXPOWER_Neg30dBm (0xD8UL) /*!< -30dBm. */
//#define RADIO_TXPOWER_TXPOWER_Neg20dBm (0xECUL) /*!< -20dBm. */
//#define RADIO_TXPOWER_TXPOWER_Neg16dBm (0xF0UL) /*!< -16dBm. */
//#define RADIO_TXPOWER_TXPOWER_Neg12dBm (0xF4UL) /*!< -12dBm. */
//#define RADIO_TXPOWER_TXPOWER_Neg8dBm (0xF8UL) /*!< -8dBm. */
//#define RADIO_TXPOWER_TXPOWER_Neg4dBm (0xFCUL) /*!< -4dBm. */
//0- (-30dBm), 1- (-20dBm), 2- (-16dBm), 3- (-12dbm) 4- (-8dBm)  5- (-4dBm) 6- (0dBm) 7- (4dBm)
uint8_t	const caucTXPOWER[8]={0xd8,0xec,0xf0,0xf4,0xf8,0xfc,0x00,0x04};
//uint8_t data_channel = 79;//数据频道，标签自动上报 
//uint8_t config_channel = 5;//配置频道
//uint8_t caucCH[2] ={79,5};
uint8_t my_tx_power;//发射功率，由ParaSet[8] .7~4决定
uint8_t En_Period_Tx;//enable period tx 由ParaSet[8] .0决定
//射频工作频道状态，参数配置定义
typedef enum
{
	run_config_channel,
	run_data_channel,
	run_idle
}radio_run_chaneel;
uint8_t radio_channel = run_idle;
uint8_t config_send_state=0; //transmit mode switch 1:switch to config channel rx mode  2:config channel tx mode
typedef enum
{
	config_start=1,//开始启动参数配置
	config_timeout,//参数配置超时
	config_success,//参数配置成功
}rf_config_state;
uint8_t rf_cfg_state;//配置成功
uint8_t cfg_tx_end;//参数配置ok，回复完成


//默认2s发射一次自身ID,10s开启一次接收窗口，用来接收配置信息
uint8_t caucID[4]={0x00,0x00,0x0f,0xff};	//MSB first 接收器ID
uint8_t	withRFRx=0;//1:with receive window,告诉上位机携带接收窗口。
#define RF_RX_FREQ 10
uint8_t	ucCntRFRx=0;//10s携带接收窗口

uint8_t	bRFRxed=0;//收到读卡器指示
uint8_t bRFRPD=0;	//RPD有效指示
uint8_t tag_state;//标签状态
//work_mode
typedef enum
{
	tag_sleep_state=0,
	tag_work_state,
	tag_wr_state
}TagSat_Type;

uint8_t work_mode;
uint8_t	ucFlag_RTC=0;//rtc标志位
uint8_t	ucCnt_LowP=0;//
uint8_t	ucCnt_nLowP=0;
uint8_t	ucFlag_LowP=0;

uint16_t tim_delay;
#define delay_cont 825
//TIM0
//程序启动时发送RFID信息，以后间隔3S发送一次RFID信息
#define	RFID_CYCLE		40  //CC:50MS*40 = 2s
uint8_t RFID_TX_EN; //CC:允许射频发送
uint8_t ucCycRFID;


static void hfclk_config(void)
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;
		NRF_CLOCK->XTALFREQ=0xffUL;
    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        // Do nothing.
    }
}

static void lfclk_config(void)
{
		NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
		NRF_CLOCK->TASKS_LFCLKSTART=1UL;
		while(NRF_CLOCK->EVENTS_LFCLKSTARTED==0);	
		
		//rtc
		NRF_RTC0->CC[0]=0x7fffUL;	//1s
		NRF_RTC0->INTENSET=0x010000UL;	//使能compare0比较事件
}

//返回最新记录ROM位置
uint8_t Rom_Pos(uint32_t temp_addr,uint8_t temp_size,uint8_t temp_byte)
{
	uint8_t i,j;
	uint8_t (* temp_buf)[16];
	temp_buf = (uint8_t(*)[16])malloc(temp_size*temp_byte*sizeof(uint8_t));
	//读取buff
	for(i=0;i<temp_size;i++)
	{
		for(j=0;j<temp_byte;j++)
		{
			temp_buf[i][j]=*(uint8_t*)temp_addr++;
		}
	}
	//验证buff是否为空，返回记录中的最新记录
	//不相等，继续查找,找到空，则返回rom位置，i =  1~~Rom_record_size。返回0表示配置区全空
	for(i=0;i<temp_size;)
	{
		for(j=0;j<temp_byte;j++)
		{
			if(temp_buf[i][j]!=0xff) break;
		}
		if(j>=temp_byte) break;
		i++;
	}
	return i;
}

void UART_Init(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);  
	  //simple_uart_config( TX_PIN_NUMBER,RX_PIN_NUMBER, HWFC);  
	  //NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos;
	  NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos);
		NVIC_SetPriority(UART0_IRQn, 0);
    NVIC_EnableIRQ(UART0_IRQn);
}
//参数初始化
void settingsSetDefaultValues(void)
{
	uint16_t flash_i;
	uint8_t flash_temp[11];//temp memory
	//flash部分,代码存储区，大小根据mcu型号决定.
	/*FICR寄存器中的CODEPAGESIZE对应着页个数，CODESIZE对应页包含的memory大小
	CODEPAGESIZE*CODESIZE即为ROM的大小,pg_size=1024,pg_num = 256,256KB的代码存储区，FLASH。
	代码存储区的最后一页存储用户配置信息*/
	pg_size = NRF_FICR->CODEPAGESIZE;
	pg_num  = NRF_FICR->CODESIZE - 1;
	//para area
	Page_Base[0] = (pg_size * (pg_num-para_area));
	//reserved area
	Page_Base[1] = (pg_size * (pg_num-reserved_area));
	//user area1
	Page_Base[2] = (pg_size * (pg_num-user_area1));
	//user area2
	Page_Base[3] = (pg_size * (pg_num-user_area2));
	
	addr = (pg_size * pg_num);
	nrf_nvmc_read_bytes(addr,flash_temp,11);
	
	//最后一个扇区用来打标记，如果空，则清空ROM0-ROM5存储区,判断是否是新下载的程序
	if((flash_temp[0]!=nvmc_flash_mark[0])||(flash_temp[1]!=nvmc_flash_mark[1])||(flash_temp[2]!=nvmc_flash_mark[2])
		||(flash_temp[3]!=nvmc_flash_mark[3])||(flash_temp[4]!=nvmc_flash_mark[4])||(flash_temp[5]!=nvmc_flash_mark[5])
		||(flash_temp[6]!=nvmc_flash_mark[6])||(flash_temp[7]!=nvmc_flash_mark[7])||(flash_temp[8]!=nvmc_flash_mark[8])
		||(flash_temp[9]!=nvmc_flash_mark[9])||(flash_temp[10]!=nvmc_flash_mark[10]))
	{			 
		addr = Page_Base[0];		
		nrf_nvmc_page_erase(addr);

		addr = Page_Base[1];
		nrf_nvmc_page_erase(addr);

		addr = Page_Base[2];
		nrf_nvmc_page_erase(addr);

		addr = Page_Base[3];
		nrf_nvmc_page_erase(addr);

		addr = (pg_size * pg_num);
		nrf_nvmc_page_erase(addr);
				
		addr = (pg_size * pg_num);
	
		nrf_nvmc_write_bytes(addr,nvmc_flash_mark,11);
	}
	else
	{
		//打过标记读取上次存储的信息
		//配置信息	ucROM0
		addr = Page_Base[0];
		ucROM0 = Rom_Pos(addr,Rom0_record_size,Rom0_record_byte);
		//reserved area
		addr = Page_Base[1];
		ucROM1 = Rom_Pos(addr,Rom1_record_size,Rom1_record_byte);
		//user area1
		addr = Page_Base[2];
		ucROM2 = Rom_Pos(addr,Rom2_record_size,Rom2_record_byte);
		//user area2
		addr = Page_Base[3];
		ucROM3 = Rom_Pos(addr,Rom3_record_size,Rom3_record_byte);			
	}
	//恢复运行参数
	//test
//	for(ucROM0=0;ucROM0<15;ucROM0++)
//	{
//		for(flash_i=0;flash_i<16;flash_i++)
//		payload[flash_i] = flash_i;
//		addr = pg_size * (pg_num-para_area);
//		nrf_nvmc_write_bytes(addr+ucROM0*Rom0_record_byte,payload,16);
//	}
	if(ucROM0)
	{
		addr = Page_Base[0];
		flash_i = (ucROM0 - 1)*Rom0_record_byte;
		nrf_nvmc_read_bytes(addr+flash_i,CurParaSet,Rom0_record_byte);
	}
	else//否则默认参数
	{
		for(flash_i = 0;flash_i<Rom0_record_byte;flash_i++)
		{
			CurParaSet[flash_i] = ParaSet[flash_i];
		}
	}
	my_tx_power = (uint32_t)caucTXPOWER[CurParaSet[8]>>4];//0dbm
	work_mode = CurParaSet[8]&0x01;
	//超声波及巡检标签初始化
	#if 0
	for(mm=0;mm<CAPACITY;mm++)
	{
		astRFID[mm].carTagID[0]=0xff;
		astRFID[mm].carTagID[1]=0xff;
		astRFID[mm].carTagID[2]=0xff;
		astRFID[mm].carTagID[3]=0xff;
		astRFID[mm].carSta[0]=0xff;
		astRFID[mm].carSta[1]=0xff;
	}
	#endif
}

int main(void)
{
	uint8_t ii;
	uint8_t jj;
//	uint8_t *temp_buf;
	#if 0
	hfclk_config();//主时钟初始化
	#endif
	lfclk_config();//rtc初始化
	settingsSetDefaultValues();//参数设置
	Radio_Init();//射频初始化

	NRF_RTC0->TASKS_START = 1;
	
	NVIC_SetPriority(RTC0_IRQn, 2);
	NVIC_EnableIRQ(RTC0_IRQn);

	//CC:硬件信息区，前4字节存放ID号
	nrf_nvmc_read_bytes(ID_BEGIN,caucID,4);//BAT --memwr 0x3B000 --val 0x0201ca2e  TagID[0] = 0x2e-->
	while(1)
	{
		__wfi();
		/*-------------------RFID----------------------------*/
		if(tag_work_state == work_mode)
		{
			if(ucFlag_RTC)
			{
				ucFlag_RTC = 0;
				
				NRF_POWER->EVENTS_POFWARN=0;
				NRF_POWER->POFCON=5UL;//2.5V	power failure
				
				//低电指示
				if(NRF_POWER->EVENTS_POFWARN)
				{
					if(ucCnt_LowP>=10) ucFlag_LowP=1;	//连续10次低电才确认
					else ucCnt_LowP++;
					ucCnt_nLowP=0;
				}
				else
				{
					if(ucCnt_nLowP>=10) ucFlag_LowP=0;	//连续10次不低电才恢复
					else ucCnt_nLowP++;
					ucCnt_LowP=0;
				}
				NRF_POWER->POFCON=0UL;
				//1.tx
				//准备数据
				for(ii=0;ii<32;ii++)
				{
					payload[ii] = 0;
				}
				//10s with receive window
				if(++ucCntRFRx>=RF_RX_FREQ) 
				{
					ucCntRFRx=0;
				}
				withRFRx = ucCntRFRx?0:1;
				tag_state = (withRFRx<<7)|(work_mode<<4)|ucFlag_LowP;
				payload[1]=0;
				payload[2]=caucID[0];
				payload[3]=caucID[1];
				payload[4]=caucID[2];
				payload[5]=caucID[3];
				payload[6]=tag_state;
				payload[7]=CurParaSet[7];	
				payload[8]=0;
				payload[9]=0;
				payload[10]=0;
				payload[11]=0;
				ii=0;
				if(bRFRxed) ii|=0x80;//收到指令
				if(bRFRPD) ii|=0x40;//RPD是否有效
				payload[12]=ii;
				payload[13]=0;
				payload[14]=0;
				payload[15]=0;
				//is with ext area?
				switch(CurParaSet[8]&0x0c)
				{
					case 0:
						if(ucROM1)
						{
							payload[13]=0x60|(ucROM1-1);
							ii = (ucROM1 - 1)*Rom1_record_byte;
							nrf_nvmc_read_bytes(addr+ii,&payload[16],Rom1_record_byte);
						}
						else
						{
							payload[13]=0x60;
							for(ii=0;ii<16;ii++) payload[16+ii]=0;	//用0填充
						}					
						break;
					case 0x04:
						if(ucROM2)
						{
							payload[13]=0xa0|(ucROM2-1);
							ii = (ucROM2 - 1)*Rom2_record_byte;
							nrf_nvmc_read_bytes(addr+ii,&payload[16],Rom2_record_byte);
	//						for(ii=0;ii<16;ii++) payload[16+ii]=nvaucPage2[ucROM2-1][i];
						}
						else
						{
							payload[13]=0xa0;
							for(ii=0;ii<16;ii++) payload[16+ii]=0;	//用0填充
						}
						break;
					case 0x08:					
						if(ucROM3)
						{
							payload[13]=0xe0|(ucROM3-1);
							ii = (ucROM3 - 1)*Rom3_record_byte;
							nrf_nvmc_read_bytes(addr+ii,&payload[16],Rom3_record_byte);
	//						for(ii=0;ii<16;ii++) payload[16+i]=nvaucPage3[ucROM3-1][i];
						}
						else
						{
							payload[13]=0xe0;
							for(ii=0;ii<16;ii++) payload[16+ii]=0;	//用0填充
						}
						break;
					default:
						break;
				}
				//XOR
				for(ii=31,jj=0;ii;ii--) jj^=payload[ii];
				payload[ii]=jj;
				//RADIO功能必须启用外部时钟
				NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
				NRF_CLOCK->TASKS_HFCLKSTART     = 1;
				while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
				if(withRFRx)//携带接收窗口
				{
					config_send_state = 1;//rf中断允许进入配置接收
					cfg_tx_end = 0;
				}			
				#if LNA_Board		
				RFN_TX_MODE;//RW=0-接收  RW=1-发送	
				#endif				
				radio_tx_carrier(my_tx_power,RADIO_MODE_MODE_Nrf_1Mbit,data_channel);
				radio_channel = run_data_channel;
				if(withRFRx)
				{
					withRFRx = 0;//clear recive window
					tim_delay = delay_cont;//510us?
					while((rf_cfg_state != config_success)&&(--tim_delay));
					if( 0==tim_delay )//time-out
					{
						rf_cfg_state =config_timeout; //time-out	
						radio_disable();
						NRF_CLOCK->TASKS_HFCLKSTOP=1UL;
					}
					else if(config_success == rf_cfg_state)
					{
						while(!cfg_tx_end);
					}
					bRFRxed = 0;
					bRFRPD = 0;
					config_send_state = 0;
					rf_cfg_state = 0;
				}
				while(radio_status!=RADIO_STATUS_IDLE);
			}
			
		}
		else if(tag_sleep_state == work_mode)
		{
			if(ucFlag_RTC)
			{
				ucFlag_RTC = 0;
				ucCntRFRx++;
				if(ucCntRFRx>=10)
				{
					ucCntRFRx = 0;
					NRF_POWER->EVENTS_POFWARN=0;
					NRF_POWER->POFCON=5UL;//2.5V		
					//低电指示
					if(NRF_POWER->EVENTS_POFWARN)
					{
						if(ucCnt_LowP>=10) ucFlag_LowP=1;	//连续10次低电才确认
						else ucCnt_LowP++;
						ucCnt_nLowP=0;
					}
					else
					{
						if(ucCnt_nLowP>=10) ucFlag_LowP=0;	//连续10次不低电才恢复
						else ucCnt_nLowP++;
						ucCnt_LowP=0;
					}
					NRF_POWER->POFCON=0UL;
					//1.tx
					//准备数据
					for(ii=0;ii<32;ii++)
					{
						payload[ii] = 0;
					}
					withRFRx = 1;
					tag_state = (withRFRx<<7)|(work_mode<<4)|ucFlag_LowP;
					payload[1]=0;
					payload[2]=caucID[0];
					payload[3]=caucID[1];
					payload[4]=caucID[2];
					payload[5]=caucID[3];
					payload[6]=tag_state;
					payload[7]=CurParaSet[7];	
					payload[8]=0;
					payload[9]=0;
					payload[10]=0;
					payload[11]=0;
					ii=0;
					if(bRFRxed) ii|=0x80;//收到指令
					if(bRFRPD) ii|=0x40;//RPD是否有效
					payload[12]=ii;
					payload[13]=0;
					payload[14]=0;
					payload[15]=0;
					//XOR
					for(ii=31,jj=0;ii;ii--) jj^=payload[ii];
					payload[ii]=jj;
					
					//RADIO功能必须启用外部时钟
					NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
					NRF_CLOCK->TASKS_HFCLKSTART     = 1;
					while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
					if(withRFRx)//携带接收窗口
					{
						config_send_state = 1;//rf中断允许进入配置接收
						cfg_tx_end = 0;
					}			
					#if LNA_Board		
					RFN_TX_MODE;//RW=0-接收  RW=1-发送	
					#endif				
					radio_tx_carrier(my_tx_power,RADIO_MODE_MODE_Nrf_1Mbit,data_channel);
					radio_channel = run_data_channel;
					if(withRFRx)
					{
						withRFRx = 0;//clear recive window
						tim_delay = delay_cont;//510us?
						while((rf_cfg_state != config_success)&&(--tim_delay));
						if( 0==tim_delay )//time-out
						{
							rf_cfg_state =config_timeout; //time-out	
							radio_disable();
							NRF_CLOCK->TASKS_HFCLKSTOP=1UL;
						}
						else if(config_success == rf_cfg_state)
						{
							while(!cfg_tx_end);
						}
						bRFRxed = 0;
						bRFRPD = 0;
						config_send_state = 0;
						rf_cfg_state = 0;
					}
					while(radio_status!=RADIO_STATUS_IDLE);
				}
			}
		}
	}
}

void RTC0_IRQHandler(void)
{
	if(NRF_RTC0->EVENTS_COMPARE[0])
	{
		NRF_RTC0->EVENTS_COMPARE[0]=0UL;	//clear event
		NRF_RTC0->TASKS_CLEAR=1UL;	//clear count

		ucFlag_RTC=1;
	}
}


/*读卡器下发
payload[1]:	7~4:保留
						3~0:8-常规信息 10-携带指令扩展信息
payload[2~5]:	当为设备ID时，对指定设备进行参数配置
							当ID为fffffffe时，对所有485接收器进行配置
payload[6]:保留
payload[7]:读写器短号
payload[8]~[11]:保留
payload[12]:7~4有定义
						3~0:
								0:读
								1：写
								2：擦除
								3：保留
								4：读运行参数(ram)
								5：写运行参数(ram,掉电丢失）
								6：读ram块
								7：设置测试标签
								8：上报测试结果
								9：保留
								10：唤醒
								11：休眠
	payload[13]:7~6:内部FLASH页地址
									0：内部参数区
									1：保留区
									2：用户区1
									3：用户区2
								5: 记录偏移有效性:1有效
							4~0:记录偏移
									页0和1各16条记录0~15
									页2和3各32条记录0~31
	payload[14]:保留
	payload[15]:保留
	payload[16]~payload[20]: 射频地址E7E7E7E7E7
	payload[21]:数据频道
	payload[22]:配置频道
	payload[23]:短号
	payload[24]:7~6:保留
							5~4:发射功率
									0:(-16dbm)
									1:(-8dbm)
									2:(0dbm)
									3:(4dbm)
							3~2:自动上报携带的信息来源
									0：保留区
									1：用户区1
									2：用户区2
							1：保留
							0：保留
	payload[25]:保留
	payload[26]:保留
	payload[27]:待定
	payload[28]:
							0xfe-接收所有的超声波标签
							0xfd-接收匹配的超声波标签
							0xfc-不接受超声波标签
	payload[29]~payload[31]:绑定的超声波ID，低3字节
	*/
	
	
//写在中断里，防止以后如果添加flash芯片时，主函数要等待很久
void radio_cmd_process(const uint8_t* RxPayload,uint8_t isack)
{
	uint8_t cmd;//命令
	uint8_t rx_page_addr;//flash页地址
	uint8_t rx_page_offset;//偏移量
	uint8_t rx_offset_valid;//记录偏移有效
	uint8_t set_page_offset;
//	uint8_t rx_radio_power;
//	uint8_t rx_radio_power_valid;
	uint8_t error_flag;
	uint8_t ack;
	uint8_t m,n;
	uint8_t temp_flash1[16];
	uint8_t *ptROM;//临时指针rom

	uint8_t TxPaylod[32];
	copybuf(TxPaylod,RxPayload,32);
	
	//power
//	rx_radio_power_valid = (TxPaylod[17]&0xc0);
//	rx_radio_power = (TxPaylod[24]&0x30)>>4;	
	//check指令
	cmd=TxPaylod[12]&0x0f;	//指令0-read 1-write 2-erase 3-reserve 4-read run-para 
	rx_page_addr=TxPaylod[13]>>6;		//页地址
	rx_offset_valid = (TxPaylod[13]&0x20)>>5;//记录偏移有效
	rx_page_offset = TxPaylod[13]&0x1f;	
	error_flag=0;	//0-pass, 1-error
	ack = 0;
	switch(cmd)
	{
		case 0://read

			if(rx_page_addr<2&&rx_page_offset>15) {error_flag=1;break;}	//超出读取范围16偏移量
			if(0 == rx_offset_valid)//无效，返回最新参数
			{
				pROM=caucpROM[rx_page_addr];
				set_page_offset=*pROM;
				if(set_page_offset) set_page_offset--;//*ucpROM表示是记录个数，所以要减1
				TxPaylod[13] |= 0x20;
			}
			else
			{
				set_page_offset = rx_page_offset;
				TxPaylod[13] &= (~0x20);
			}
			ack = 1;
			break;

		case 1://write
		
			if(TxPaylod[13]&0x20) {error_flag=1;break;}	//配置参数时，记录偏移有效位为0.
			if(0 == rx_page_addr)//page0-set，内部参数		
			{				
				set_page_offset = ucROM0;//最新偏移量
				if(set_page_offset) set_page_offset--;
				addr = Page_Base[rx_page_addr];
				m = set_page_offset*Rom_record_byte[rx_page_addr];
				nrf_nvmc_read_bytes(addr+m,temp_flash1,Rom_record_byte[rx_page_addr]);
				//1-10，指向记录参数，0指向预设参数
/*
[0]	.7~6 	0		硬件类型
		.5~0	0~15	版本号
[1]	.7~4	6		发射功率：0- (-30dBm), 1- (-20dBm), 2- (-16dBm), 3- (-12dBm)
													4- (8dBm), 5- (4dBm), 6- (0dBm), 7- (4dBm)
		.3~2	0~2		自动上报携带的信息来源：0-保留区，1-用户区1, 2-用户区2
		.1~0	0		使能周期发射 1-发射
[2]	.7~0	0		保留
[3]	.7~0	0		保留
[4]	.7		0		保留		
		.6		0/1		衰减方案：0-收发对称衰减；1-接收衰减
		.5		0/1		开启RPD过滤：1-开启
		.4~0	0~31	衰减值，0~31-最小0dB最大31dB
[5~8]				绑定的超声波探测标签ID，MSB在先：
[5]	.7~0	0xfe~0xfc
							0xFEXXXXXX-接收所有的超声波标签（无需绑定）
							0xFDXXXXXX-接收匹配的超声波标签（绑定）
							0xFCXXXXXX-不接收超声波标签				
*/
				ptROM=ucROM0?temp_flash1:CurParaSet;
		
				//如果发送下来的参数无效，丢掉这段payload，并且回复的上次最新记录的参数
				//发射功率 7~6不等于00，无效 5~4等于11无效                                                               
				if(TxPaylod[24]&0x82)//第8/2bit不为1.
				{
					TxPaylod[24] = ptROM[8];
				}
				//id
				TxPaylod[25] = ptROM[9];	
				TxPaylod[26] = ptROM[10];
				TxPaylod[27] = ptROM[11];
				TxPaylod[28] = ptROM[12];
				TxPaylod[29] = ptROM[13];
				TxPaylod[30] = ptROM[14];
				TxPaylod[31] = ptROM[15];
				//read-only
				TxPaylod[16]=ParaSet[0];
				TxPaylod[17]=ParaSet[1];
				TxPaylod[18]=ParaSet[2];
				TxPaylod[19]=ParaSet[3];
				TxPaylod[20]=ParaSet[4];
				TxPaylod[21]=ParaSet[5];
				TxPaylod[22]=ParaSet[6];
				TxPaylod[23]=ParaSet[7];	
				//比较写入内容与最近记录，不重复写入
				for(m=0;m<Rom_record_byte[rx_page_addr];m++)
				{
					if(ptROM[m]!=TxPaylod[16+m]) break;
				}	
				if(m<Rom_record_byte[rx_page_addr])//change
				{
					if(ucROM0>=Rom_record_size[rx_page_addr]) //full? erase
					{
						ucROM0=0;	
						nrf_nvmc_page_erase(addr);	
					}
					
					nrf_nvmc_write_bytes(addr+ucROM0*Rom0_record_byte,&TxPaylod[16],16);	
					//将运行参数保存在acuSet中，后续设置参数
					//设置生效,[0]~[6] read-only
					for(m=7;m<16;m++) CurParaSet[m]=TxPaylod[m+16];//update parameter
					ucROM0++;
					set_page_offset = ucROM0-1;
					my_tx_power = (uint32_t)caucTXPOWER[CurParaSet[8]>>4];//transmit power
					work_mode = CurParaSet[8]&0x01;
					TxPaylod[13] |= 0x20;
				}
				else
				{
					TxPaylod[13] &= (~0x20);
				}
			}
			else if(1 == rx_page_addr || 2==rx_page_addr || 3 == rx_page_addr)//reserve page,user page1,user page2
			{					
				//不写空记录
				for(m=16;m<32;m++)
				{
					if(TxPaylod[m]!=0xff) break;	
				}
				if(m>=32) 
				{
					error_flag=1;
					break;
				}
				//比较写入内容与最近记录，不重复写入
				pROM = caucpROM[rx_page_addr];//记录个数
				set_page_offset = *pROM;
				if(set_page_offset>0) set_page_offset--;
				addr = Page_Base[rx_page_addr]; //BASE ADDR
				m = set_page_offset*Rom_record_byte[rx_page_addr];//offset addr
				nrf_nvmc_read_bytes(addr+m,temp_flash1,Rom_record_byte[rx_page_addr]);
				for(m=0;m<Rom_record_byte[rx_page_addr];m++)//不重复写入
				{
					if(temp_flash1[m]!=TxPaylod[16+m])break;//different parameter
				}
				if(m<Rom_record_byte[rx_page_addr])//write new parameter
				{
					if(*pROM>=Rom_record_size[rx_page_addr])
					{
						#if 0//not erase ,ack new parameter
						addr = Page_Base[rx_page_addr]; //BASE ADDR
						nrf_nvmc_page_erase(addr);
						(*pROM) = 0;
						#endif
						TxPaylod[13] &= (~0x20);
					}
					else
					{
						m = (*pROM) * Rom_record_byte[rx_page_addr];//offset
						nrf_nvmc_write_bytes(addr+m,&TxPaylod[16],Rom_record_byte[rx_page_addr]);
						
						//check read back
						nrf_nvmc_read_bytes(addr+m,temp_flash1,Rom_record_byte[rx_page_addr]);
						for(m=0;m<Rom_record_byte[rx_page_addr];m++)
						{
							if(temp_flash1[m]!=0xff)break;//different parameter
						}
						if(m<Rom_record_byte[rx_page_addr])
						{
							(*pROM)++;
							set_page_offset = (*pROM)-1;
						}
						TxPaylod[13] |= 0x20;
					}
				}
				else
				{
					TxPaylod[13] &= (~0x20);
				}
			}	
			ack = 1;
			break;
		
		case 2://erase
		
			pROM = caucpROM[rx_page_addr];//记录个数
			addr = Page_Base[rx_page_addr]; //BASE ADDR
			nrf_nvmc_page_erase(addr);
			(*pROM) = 0;
			ack =2;
					
		case 3:break;
		case 4://read ram
		
			for(m=0;m<16;m++) TxPaylod[m+16]=CurParaSet[m];
			ack = 4;
			break;
		
		case 5://write ram
		
			//不写空记录
			for(m=16;m<32;m++)
			{
				if(TxPaylod[m]!=0xff) break;	
			}
			if(m>=32) error_flag=1;
			else
			{
				//check para
				if(TxPaylod[24]&0x02)//第2bit不为1.
				{
					error_flag=1;
				}
				if(TxPaylod[28]==0xff||TxPaylod[28]<0xfc)
				{
					error_flag=1;
				}
			}
			ack =5;
			break;
			
		default:break;
	}	
		
	
	if(isack)
	{
		if(!error_flag)//ack
		{
			if(1==ack)//读写回复
			{
				m = TxPaylod[13]&0xe0;//7~6位页面指示	
//				m |=0x20;//偏移有效
				TxPaylod[13] = m | set_page_offset;
				addr = Page_Base[rx_page_addr];
				m = set_page_offset *Rom_record_byte[rx_page_addr];
				nrf_nvmc_read_bytes(addr+m,&TxPaylod[16],Rom_record_byte[rx_page_addr]);
				if(0 == rx_page_addr)
				{
					TxPaylod[23]=(HWTYP<<6)|(VERSION&0x3f);
				}
			}
			else if(2==ack)//擦除
			{
				addr = Page_Base[rx_page_addr];
				nrf_nvmc_read_bytes(addr,&TxPaylod[16],Rom_record_byte[rx_page_addr]);
				TxPaylod[13] = TxPaylod[13]&0xc0;
			}
			else if(4 == ack)//read ram
			{
				TxPaylod[23]=(HWTYP<<6)|(VERSION&0x3f);//插入硬件类型和软件版本: 原短号位置
			}
			else if(5 == ack)//write ram
			{
				for(m=7;m<16;m++) CurParaSet[m] = TxPaylod[16+m];
				my_tx_power = (uint32_t)caucTXPOWER[CurParaSet[8]>>4];//transmit power
				work_mode = CurParaSet[8]&0x01;
				
			}
			TxPaylod[1] = 0x02;//上行
			TxPaylod[2]=caucID[0];
			TxPaylod[3]=caucID[1];
			TxPaylod[4]=caucID[2];
			TxPaylod[5]=caucID[3];
			TxPaylod[6]=0;
			TxPaylod[7]=CurParaSet[7];
			TxPaylod[8]=0;
			TxPaylod[9]=0;
			TxPaylod[10]=0;
			TxPaylod[11]=0;
			m = 0;
			m = (bRFRxed<<7)|(bRFRPD<<6);
			TxPaylod[12] = TxPaylod[12]| m;
			//XOR
			for(m=31,n=0;m;m--) n^=TxPaylod[m];
			TxPaylod[m]=n;
			#if LNA_Board		
			RFN_TX_MODE;//RW=0-接收  RW=1-发送	
			#endif
			copybuf(payload,TxPaylod,32);
			config_send_state = 2;//ack
			radio_channel = run_config_channel;	
			rf_cfg_state = config_success;			
			radio_tx_carrier(my_tx_power,RADIO_MODE_MODE_Nrf_1Mbit, config_channel);
		}
	}
}

/*读卡器下发
payload[1]:	7~4:保留
						3~0:8-常规信息 10-携带指令扩展信息
payload[2~5]:	当为设备ID时，对指定设备进行参数配置
							当ID为fffffffe时，对所有485接收器进行配置
payload[6]:保留
payload[7]:读写器短号
payload[8]~[11]:保留
payload[12]:7~4有定义
						3~0:
								0:读
								1：写
								2：擦除
								3：保留
								4：读运行参数(ram)
								5：写运行参数(ram,掉电丢失）
								6：读ram块
								7：设置测试标签
								8：上报测试结果
								9：保留
								10：唤醒
								11：休眠
	payload[13]:7~6:内部FLASH页地址
									0：内部参数区
									1：保留区
									2：用户区1
									3：用户区2
								5: 记录偏移有效性:1有效
							4~0:记录偏移
									页0和1各16条记录0~15
									页2和3各32条记录0~31
	payload[14]:保留
	payload[15]:保留
	payload[16]~payload[20]: 射频地址E7E7E7E7E7
	payload[21]:数据频道
	payload[22]:配置频道
	payload[23]:短号
	payload[24]:7~6:保留
							5~4:发射功率
									0:(-16dbm)
									1:(-8dbm)
									2:(0dbm)
									3:(4dbm)
							3~2:自动上报携带的信息来源
									0：保留区
									1：用户区1
									2：用户区2
							1：保留
							0：保留
	payload[25]:保留
	payload[26]:保留
	payload[27]:待定
	payload[28]:
							0xfe-接收所有的超声波标签
							0xfd-接收匹配的超声波标签
							0xfc-不接受超声波标签
	payload[29]~payload[31]:绑定的超声波ID，低3字节
*/

void RADIO_IRQHandler()
{
	uint8_t radio_uc1,radio_uc2;
//	uint8_t rfid_uc1,rfid_uc2;
	uint8_t b1,b2,bAck;
	if(1 == NRF_RADIO->EVENTS_END)
	{
		NRF_RADIO->EVENTS_END = 0;
		if(radio_status == RADIO_STATUS_TX)
		{
			if(1==config_send_state)
			{
				#if LNA_Board
				RFN_RX_MODE;//RW=0-接收  RW=1-发送
				#endif
				radio_rx_carrier(RADIO_MODE_MODE_Nrf_1Mbit, config_channel);//switch to receive 
				radio_channel = run_config_channel;
				rf_cfg_state = config_start;//开始接收配置参数
			}
			else if(2 == config_send_state)
			{
				cfg_tx_end = 1;
				radio_disable();
				NRF_CLOCK->TASKS_HFCLKSTOP=1UL;
			}
			else
			{
				radio_disable();
				NRF_CLOCK->TASKS_HFCLKSTOP=1UL;
			}
		}
		else if(RADIO_STATUS_RX == radio_status)
		{
			if(NRF_RADIO->CRCSTATUS)
			{
				//XOR异或为0 表示数据有效
				for(radio_uc1=0,radio_uc2=0;radio_uc2<32;radio_uc2++)
				{
					radio_uc1^=payload[radio_uc2];
				}
				if(!radio_uc1)
				{
					//具体看协议文档-设计说明
					if(run_config_channel == radio_channel)
					{
						if((payload[1]&0x0a)==0x0a)//下行，携带扩展指令
						{
							//b1:1 receive config info 
							//b2:1:allow ack 0:forbid ack
							b1=0;b2=0;
							if(payload[2]==caucID[0]
							&&payload[3]==caucID[1]
							&&payload[4]==caucID[2]
							&&payload[5]==caucID[3])//id match
							{
								b1=1;
								b2=1;
							}
							else if(payload[2]==0xff
							&&payload[3]==0xff
							&&payload[4]==0xff
							&&payload[5]==0xfd)//如果是广播信息，一律不回复。
							{
								b1=1;
//								if(payload[5]!=0xff) b2=1;
							}
							if(b1)
							{
								bAck=b2;
								bRFRxed=1;//收到读卡器指示
								if(NRF_RADIO->RSSISAMPLE<64)
								{
									bRFRPD=1;//信号强度有效
								}
								#if 0
								if(!bAck)//50ms timeout,back to rx data channel
								{
									//返回数据频道接收
									#if LNA_Board
									RFN_RX_MODE;//RW=0-接收  RW=1-发送
									#endif
									radio_rx_carrier(RADIO_MODE_MODE_Nrf_1Mbit,data_channel);
									RFN_RX_MODE;//RW=0-接收  RW=1-发送
									radio_channel = run_data_channel;
								}
								#endif
								radio_cmd_process(&payload[0],bAck);
												
							}
						}
					}
				}
			}
		}
		if(RADIO_STATUS_RX == radio_status)
		{	
			NRF_RADIO->TASKS_START = 1U;
		}
	}
}
						



