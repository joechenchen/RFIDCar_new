#include "nrf.h"
#include "nrf_gpio.h"
#include "mxgpio.h"
#include "radio_config.h"
#include "simple_uart.h"
#include "tim.h"
#include "nrf_nvmc.h"
#include "sys.h"
#include "stdlib.h"
#include "common.h"
/*
USTagID ³¬Éù²¨±êÇ©ID
carTagID Ñ²¼ì±êÇ©ID
USAge³¬Éù²¨±êÇ©Àë¿ªÊÂ¼ş
USAge us leave time
*/		
#define	CAPACITY	1		//±êÇ©ÈİÁ¿
struct {
	uint8_t USTagID[4];//ultrasonic tag
	uint8_t	USAge;     //Ê±¼ä
	uint8_t USSta[2];		//state
	uint8_t carTagID[4];//car tag
	uint8_t	carSta[2];   //×´Ì¬
	uint8_t carAge;
	uint8_t RSSI;
	uint8_t baseboard[2];
}astRFID[CAPACITY];	

typedef enum
{
	us_without_car,
	us_with_car,
	us_error
}us_state;

typedef struct
{
	uint16_t Distance;//¾àÀë
	uint16_t threshold_const;//³¬Éù²¨ÃÅÏŞ
	uint8_t RX_Dis_Flag;//½ÓÊÕµ½³¬Éù²¨±êÖ¾Î»
	uint8_t confirm_cnt_const;//È·ÈÏ´ÎÊı³£Á¿
	uint8_t delay_send_cnt_const;//ÑÓ³Ù·¢ËÍ´ÎÊı³£Á¿
	uint8_t rxTag_recent_time_const;//×î½ü¼¸Ãë½ÓÊÕ±êÇ©£¬µ¥Î»0.5s£¬*10³£Á¿
	uint8_t confirm_cnt;//È·ÈÏ´ÎÊı
	uint8_t delay_send_cnt;//ÑÓ³Ù·¢ËÍ´ÎÊı
	uint8_t state;//³¬Éù²¨±êÖ¾
}US_Type;
US_Type ultrasonic;
#define US_Leave_time 3 //6ÃëÈÏÎª³¬Éù²¨±êÇ©Àë¿ª
uint8_t const constTagID[4] = {0xfd,0,0,0};
extern uint8_t radio_status;
extern uint8_t payload[PACKET_PAYLOAD_MAXSIZE];

//ÒÔÏÂ¼¸¸ö²ÎÊıÓÃÀ´¼ÇÂ¼ROMÖĞµÄ16/32¿é£¨Æ«ÒÆÁ¿£©µÄ¼ÆÊıÖµ Öµ16/32£¬±íÊ¾¼ÇÂ¼¸öÊı£¬0±íÊ¾Î´¼ÇÂ¼
//255ÉÈÇø´ò±ê¼Ç0x3fc00
//254ÉÈÇørom0 ´æ´¢ÆµµÀ²ÎÊı  0x3f800
//253ÉÈÇørom1 ±£ÁôÇø 0x3f400
//252ÉÈÇørom2 ÓÃ»§Çø1 0x3f00
//251ÉÈÇørom3 ÓÃ»§Çø2 0x3ec00
uint8_t ucRomMark;//µ¹ÊıµÚÒ»¸öÇø´ò±ê»ú
uint8_t	ucROM0;		//µ¹ÊıµÚ¶ş¸öÇø£¬ÄÚ²¿²ÎÊıÇø
uint8_t	ucROM1;		//µ¹ÊıµÚÈı¸öÉÈÇø£¬±£ÁôÇø
uint8_t	ucROM2;	  //µ¹ÊıµÚËÄ¸öÉÈÇø£¬ÓÃ»§Çø1
uint8_t	ucROM3;   //µ¹ÊıµÚÎå¸öÉÈÇø,ÓÃ»§Çø2
uint8_t *pROM;	//¼ÇÂ¼Ö¸Õë
uint8_t	* caucpROM[]={&ucROM0,&ucROM1,&ucROM2,&ucROM3};	
uint32_t Page_Base[5];//page addr
#define para_area 1
#define reserved_area 2
#define user_area1 3
#define user_area2 4
//1¡¢2Çø16Ìõ¼ÇÂ¼£¬3¡¢4Çø32Ìõ¼ÇÂ¼
//Ã¿Ìõ¼ÇÂ¼16¸ö×Ö½Ú
uint8_t Rom_record_size[4] = {16,16,32,32};//4¸öÉÈÇø¶ÔÓ¦µÄ¼ÇÂ¼¸öÊı
uint8_t Rom_record_byte[4] = {16,16,16,16};//Ã¿Ìõ¼ÇÂ¼¶ÔÓ¦µÄ×Ö½ÚÊı
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


//flash²ÎÊı¼ÇÂ¼
//´úÂë´æ´¢Çø£¬´óĞ¡¸ù¾İmcuĞÍºÅ¾ö¶¨.
/*FICR¼Ä´æÆ÷ÖĞµÄCODEPAGESIZE¶ÔÓ¦×ÅÒ³¸öÊı£¬CODESIZE¶ÔÓ¦Ò³°üº¬µÄmemory´óĞ¡
CODEPAGESIZE*CODESIZE¼´ÎªROMµÄ´óĞ¡,pg_size=1024,pg_num = 256,256KBµÄ´úÂë´æ´¢Çø£¬FLASH¡£
´úÂë´æ´¢ÇøµÄ×îºóÒ»Ò³´æ´¢ÓÃ»§ÅäÖÃĞÅÏ¢*/
uint32_t pg_size;//Ò»Ò³¶ÔÓ¦µÄ×Ö½ÚÊı
uint32_t pg_num;//Ò³¸öÊı
uint32_t addr;
//user-defined MARK
const uint8_t nvmc_flash_mark[11]={0xaa,0xaa,0x01,0x03,0x05,0x01,0x23,0x45,0x67,0x89,0x8e};

/*

³¬Éù²¨±êÇ©ÄÚ²¿¶¨Òå
ucSet[5]	.7~0	79		Êı¾İÆµµÀ£¨±êÇ©×Ô¶¯ÉÏ±¨£©
ucSet[6]	.7~0	5		ÅäÖÃÆµµÀ£¨¶ÁĞ´Æ÷£¬±êÇ©½»»¥£©	
ucSet[7]	.7~4 	0		±£Áô
			.3~0	0~15	¶ÌºÅ
ucSet[8]	.7~4	6		·¢Éä¹¦ÂÊ£º0- (-30dBm), 1- (-20dBm), 2- (-16dBm), 3- (-12dBm)
													4- (8dBm), 5- (4dBm), 6- (0dBm), 7- (4dBm)
					.3~2	0~2		×Ô¶¯ÉÏ±¨Ğ¯´øµÄĞÅÏ¢À´Ô´£º0-±£ÁôÇø£¬1-ÓÃ»§Çø1, 2-ÓÃ»§Çø2
					.1~0	0		±£Áô
ucSet[9]	.7~0	0		±£Áô
ucSet[10]	.7~0	0		±£Áô
ucSet[11]	.7		0		±£Áô		
			.6		0/1		Ë¥¼õ·½°¸£º0-ÊÕ·¢¶Ô³ÆË¥¼õ£»1-½ÓÊÕË¥¼õ
			.5		0/1		¿ªÆôRPD¹ıÂË£º1-¿ªÆô
			.4~0	0~31	Ë¥¼õÖµ£¬0~31-×îĞ¡0dB×î´ó31dB
ucSet[12]					³¬Éù²¨Ì½²â²ÎÊı£¬Ä¬ÈÏÖµ0x14
			.7~4	0~14	³¬Éù²¨Ì½²âÈ·ÈÏ´ÎÊı: 0-µ¥ÖµÈ·ÈÏ£¬·´Ó¦×î¿ì£¬1-Á¬ĞøÁ½¸öÖµ²ÅÈ·ÈÏ£¬n-Á¬Ğøn+1¸öÖµ²ÅÈ·ÈÏ£»
			.3~0	0~14	ÑÓ³Ù·¢Éä´ÎÊı: 0-²»ÑÓ³Ù·¢Éä£¬³¬Éù²¨Ì½²âÎŞ³µÁ¢¿Ì²»·¢£¬n-³¬Éù²¨Ì½²âÎŞ³µºó»¹·¢Éän´Î£¬ÖÜÆÚ0.5Ãë£»		
ucSet[13]	.7~0	0~16	³¬Éù²¨Ì½²âÃÅÏŞ: µ¥Î»256mm£¬Ä¬ÈÏÖµ0x07£¬¼´²â¾àµÍÓÚ7*0.256=1.792Ã×±¨ÓĞ³µ
ucSet[14]	.7~0	0~16	×î½ü¼¸ÃëÄÚÊÕµ½µÄ±êÇ©£¬µ¥Î»0.5Ãë£¬Ä¬ÈÏÖµ6£¨×î½ü3Ãë£©
ucSet[15]	.7~0	0~1		µ÷ÊÔÄ£Ê½£º1-µ÷ÊÔÄ£Ê½£¬Êä³öÃ¿´Î²âÁ¿Êı¾İºÍÉèÖÃ²ÎÊı	

*/
//5×Ö½ÚÉäÆµµØÖ·£¬Êı¾İÆµµÀ£¬ÅäÖÃÆµµÀ,·¢Éä¹¦ÂÊ
#define	HWTYP		0 		//0-Æ»¹û£¨ÎŞË¥¼õ£©£¬1-¶Ìºì°å£¨µ¥Ë¥¼õ£©
#define	VERSION		0x02
#define data_channel 79//Êı¾İÆµµÀ£¬±êÇ©×Ô¶¯ÉÏ±¨ 
#define config_channel 5//ÅäÖÃÆµµÀ
uint8_t const ParaSet[16]={0xe7,0xe7,0xe7,0xe7,0xe7,79,5,0,0x70,0,0,0x40,0x14,7,6,0};	//Ä¬ÈÏ²ÎÊı;	

//[0]~[6] read-only
//[7]~[15] can write
uint8_t CurParaSet[16];//µ±Ç°ÔËĞĞ²ÎÊı,µÈÓÚParaSetµÄ²ÎÊı
//·¢Éä¹¦ÂÊ0,
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
//uint8_t data_channel = 79;//Êı¾İÆµµÀ£¬±êÇ©×Ô¶¯ÉÏ±¨ 
//uint8_t config_channel = 5;//ÅäÖÃÆµµÀ
//uint8_t caucCH[2] ={79,5};
uint8_t debug_mode;//µ÷ÊÔÄ£Ê½
uint8_t my_tx_power;//·¢Éä¹¦ÂÊ£¬ÓÉParaSet[8] .7~4¾ö¶¨
uint8_t rssi;
#if 0
uint8_t En_Period_Tx;//enable period tx ÓÉParaSet[8] .0¾ö¶¨
#endif
//ÉäÆµ¹¤×÷ÆµµÀ×´Ì¬£¬²ÎÊıÅäÖÃ¶¨Òå
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
	config_start=1,//¿ªÊ¼Æô¶¯²ÎÊıÅäÖÃ
	config_timeout,//²ÎÊıÅäÖÃ³¬Ê±
	config_success//²ÎÊıÅäÖÃ³É¹¦
}rf_config_state;
uint8_t rf_cfg_state;//ÅäÖÃ³É¹¦
uint8_t timeout_cnt_start;//start cnt
uint8_t timeout_cnt;//timeout cnt

//Ä¬ÈÏ2s·¢ÉäÒ»´Î×ÔÉíID,10s¿ªÆôÒ»´Î½ÓÊÕ´°¿Ú£¬ÓÃÀ´½ÓÊÕÅäÖÃĞÅÏ¢
uint8_t caucID[4]={0xfD,0x00,0x00,0x61};	//MSB first ½ÓÊÕÆ÷ID
uint8_t	withRFRx=0;//1:with receive window,¸æËßÉÏÎ»»úĞ¯´ø½ÓÊÕ´°¿Ú¡£
#define RF_RX_FREQ 5
uint8_t	ucCntRFRx=0;//10sĞ¯´ø½ÓÊÕ´°¿Ú

uint8_t	bRFRxed=0;//ÊÕµ½¶Á¿¨Æ÷Ö¸Ê¾
uint8_t bRFRPD=0;	//RPDÓĞĞ§Ö¸Ê¾
//´®¿Ú²ÎÊı¶¨Òå
uint8_t uart_tx_en;//´®¿Ú·¢ËÍÊ¹ÄÜ
uint16_t tx_len;//´®¿Ú³¤¶È
UartData_TypeDef  U_Master; //´®¿Ú
PACKET_TypeDef RX_PACKET;
uint8_t tx_buf[28]; //6head+2len+4addr+2attr+1type+info(6/12)+1xor

//TIM0
//³ÌĞòÆô¶¯Ê±·¢ËÍRFIDĞÅÏ¢£¬ÒÔºó¼ä¸ô3S·¢ËÍÒ»´ÎRFIDĞÅÏ¢
#define	RFID_CYCLE		40  //CC:50MS*40 = 2s
uint8_t RFID_TX_EN; //CC:ÔÊĞíÉäÆµ·¢ËÍ
uint8_t ucCycRFID;

//Ã¿0.5s²âÁ¿Ò»´Î
uint8_t US_Start_Measure;//Æô¶¯²âÁ¿
uint8_t US_Cnt;//³¬Éù²¨¼ÆÊıÆ÷
#define US_CYCLE 10
#define US_CMD_MEA 0X55 //²âÁ¿



static void hfclk_config(void)
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        // Do nothing.
    }
}

//·µ»Ø×îĞÂ¼ÇÂ¼ROMÎ»ÖÃ
uint8_t Rom_Pos(uint32_t temp_addr,uint8_t temp_size,uint8_t temp_byte)
{
	uint8_t i,j;
	uint8_t (* temp_buf)[16];
	temp_buf = (uint8_t(*)[16])malloc(temp_size*temp_byte*sizeof(uint8_t));
	//¶ÁÈ¡buff
	for(i=0;i<temp_size;i++)
	{
		for(j=0;j<temp_byte;j++)
		{
			temp_buf[i][j]=*(uint8_t*)temp_addr++;
		}
	}
	//ÑéÖ¤buffÊÇ·ñÎª¿Õ£¬·µ»Ø¼ÇÂ¼ÖĞµÄ×îĞÂ¼ÇÂ¼
	//²»ÏàµÈ£¬¼ÌĞø²éÕÒ,ÕÒµ½¿Õ£¬Ôò·µ»ØromÎ»ÖÃ£¬i =  1~~Rom_record_size¡£·µ»Ø0±íÊ¾ÅäÖÃÇøÈ«¿Õ
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
		NRF_UART0->INTENSET = UART_INTENSET_ERROR_Msk;
		NVIC_SetPriority(UART0_IRQn, 0);
    NVIC_EnableIRQ(UART0_IRQn);
}
//²ÎÊı³õÊ¼»¯
void settingsSetDefaultValues(void)
{
	uint16_t flash_i,mm;
	uint8_t flash_temp[11];//temp memory
	//flash²¿·Ö,´úÂë´æ´¢Çø£¬´óĞ¡¸ù¾İmcuĞÍºÅ¾ö¶¨.
	/*FICR¼Ä´æÆ÷ÖĞµÄCODEPAGESIZE¶ÔÓ¦×ÅÒ³¸öÊı£¬CODESIZE¶ÔÓ¦Ò³°üº¬µÄmemory´óĞ¡
	CODEPAGESIZE*CODESIZE¼´ÎªROMµÄ´óĞ¡,pg_size=1024,pg_num = 256,256KBµÄ´úÂë´æ´¢Çø£¬FLASH¡£
	´úÂë´æ´¢ÇøµÄ×îºóÒ»Ò³´æ´¢ÓÃ»§ÅäÖÃĞÅÏ¢*/
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
	
	//×îºóÒ»¸öÉÈÇøÓÃÀ´´ò±ê¼Ç£¬Èç¹û¿Õ£¬ÔòÇå¿ÕROM0-ROM5´æ´¢Çø,ÅĞ¶ÏÊÇ·ñÊÇĞÂÏÂÔØµÄ³ÌĞò
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
		//´ò¹ı±ê¼Ç¶ÁÈ¡ÉÏ´Î´æ´¢µÄĞÅÏ¢
		//ÅäÖÃĞÅÏ¢	ucROM0
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
	//»Ö¸´ÔËĞĞ²ÎÊı
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
	else//·ñÔòÄ¬ÈÏ²ÎÊı
	{
		for(flash_i = 0;flash_i<Rom0_record_byte;flash_i++)
		{
			CurParaSet[flash_i] = ParaSet[flash_i];
		}
	}
	my_tx_power = (uint32_t)caucTXPOWER[CurParaSet[8]>>4];//0dbm
	#if 0
	En_Period_Tx = CurParaSet[8]&0x01;
	#endif
	ultrasonic.confirm_cnt_const = (CurParaSet[12]>>4) + 1; //È·ÈÏ´ÎÊı
	ultrasonic.delay_send_cnt_const = (CurParaSet[12]&0x0f);//ÑÓ³Ù·¢ËÍ´ÎÊı
	ultrasonic.threshold_const = CurParaSet[13]<<8; //Ì½²âÃÅÏŞ
	ultrasonic.rxTag_recent_time_const = CurParaSet[14];
	debug_mode = CurParaSet[15];
	rssi = CurParaSet[11];
	//³¬Éù²¨¼°Ñ²¼ì±êÇ©³õÊ¼»¯
	for(mm=0;mm<CAPACITY;mm++)
	{
		astRFID[mm].USTagID[0]=0xff;
		astRFID[mm].USTagID[1]=0xff;
		astRFID[mm].USTagID[2]=0xff;
		astRFID[mm].USTagID[3]=0xff;
		astRFID[mm].USSta[0]=0xff;
		astRFID[mm].USSta[1]=0xff;
		astRFID[mm].USAge=0xff;
		astRFID[mm].carTagID[0]=0xff;
		astRFID[mm].carTagID[1]=0xff;
		astRFID[mm].carTagID[2]=0xff;
		astRFID[mm].carTagID[3]=0xff;
		astRFID[mm].carAge=0xff;
		astRFID[mm].carSta[0]=0xff;
		astRFID[mm].carSta[1]=0xff;
	}
}

int main(void)
{
	uint8_t ii;
	uint8_t jj;
//	uint16_t pkt_len_temp;
	uint8_t old_us_state;//³¬Éù²¨×´Ì¬
	uint8_t us_state;
	uint8_t car_leave;//³µÀë¿ª
//	uint8_t *temp_buf;
	hfclk_config();//Ö÷Ê±ÖÓ³õÊ¼»¯
	settingsSetDefaultValues();//²ÎÊıÉèÖÃ
	Radio_Init();//ÉäÆµ³õÊ¼»¯
	Timer0_Init(50);//Ê±ÖÓ³õÊ¼»¯
	UART_Init();//´®¿Ú³õÊ¼»¯
	NRF_TIMER0->TASKS_START    = 1;
	#if LNA_Board	
	RFN_RX_MODE;//RW=0-½ÓÊÕ  RW=1-·¢ËÍ	
	#endif
	radio_rx_carrier(RADIO_MODE_MODE_Nrf_1Mbit,data_channel);
	radio_channel = run_data_channel;
	while(1)
	{
		//¶¨Ê±ÏµÍ³
		/*-------------------RFID----------------------------*/
		//2s²åÈëÒ»´Î·¢Éä£¬10s¿ªÆôÒ»´Î½ÓÊÕ£¬¼´½ô¸úĞ¯´ø½ÓÊÕ´°¿Ú
		#if 0
		if(RFID_TX_EN&&En_Period_Tx)
		#endif
		//³¬Éù²¨²â¾àÆµÂÊ0.5sÒ»´Î
		if(US_Start_Measure)
		{
			US_Start_Measure = 0;
			tx_buf[0] = US_CMD_MEA;
			UART_Send(tx_buf,1);
			if(astRFID[0].carAge<0xff)//±êÇ©Àë¿ª¼ÆÊı
			{
				astRFID[0].carAge++;
			}
		}
		//½ÓÊÕµ½ºÏ·¨µÄ³¬Éù²¨Êı¾İ
		if(ultrasonic.RX_Dis_Flag)
		{
			ultrasonic.RX_Dis_Flag = 0;
			us_state =  (ultrasonic.Distance < ultrasonic.threshold_const)?us_with_car:us_without_car;
			if(us_state ^ old_us_state)//×´Ì¬¸Ä±ä
			{
				old_us_state = us_state;
				ultrasonic.confirm_cnt = 0;
			}				
			else
			{
				ultrasonic.confirm_cnt++;
			}
			if(ultrasonic.confirm_cnt>=ultrasonic.confirm_cnt_const)
			{
				ultrasonic.state = us_state;
			}

			switch(ultrasonic.state)
			{
				case us_with_car://0.5s·¢ËÍÒ»´Î
					RFID_TX_EN = 1;
					car_leave=1;
					ultrasonic.delay_send_cnt = 0;
					break;
				case us_without_car:
					if(1==car_leave)
					{
						if(ultrasonic.delay_send_cnt < ultrasonic.delay_send_cnt_const)//ÑÓ³Ù·¢ËÍ
						{
							ultrasonic.delay_send_cnt++;
							
							RFID_TX_EN = 1;
							
						}
						else
						{
							car_leave=0;
						}
					}
					break;
				default:break;
			}			
		}
		if(RFID_TX_EN)
		{
			RFID_TX_EN = 0;
			radio_disable();
			//1.tx
			//×¼±¸Êı¾İ
			for(ii=0;ii<32;ii++)
			{
				payload[ii] = 0;
			}
			#if 0 
			//10s with receive window
			if(++ucCntRFRx>=RF_RX_FREQ) 
			{
				ucCntRFRx=0;
			}
			#endif
			withRFRx = 1;//Ğ¯´ø½ÓÊÕ´°¿Ú			
			withRFRx = ucCntRFRx?0:1;
			//rf transmit payload defined
			/*
			0:xor
			1:seq
			2~5:id FEFDFFFF~FEFD0000
			6:is with receive rf window?
			7:reserve
			8:sensor type
			9:sensor data
			10:sensor para
			11:current counter
			/8~11 test des receiver ID
			12:7bit receive instructions   6:1-PDR active
			*/
			payload[1]=0;
			payload[2]=caucID[0];
			payload[3]=caucID[1];
			payload[4]=caucID[2];
			payload[5]=caucID[3];
			payload[6]=withRFRx?0x80:0;
			payload[7]=CurParaSet[7];	
			payload[8]=0;
			payload[9]=0;
			payload[10]=0;
			payload[11]=0;
			ii=0;
			if(bRFRxed) ii|=0x80;//ÊÕµ½Ö¸Áî
			if(bRFRPD) ii|=0x40;//RPDÊÇ·ñÓĞĞ§
			payload[12]=ii;
			payload[13]=0;
			payload[14]=0;
			payload[15]=0;
			
			if(debug_mode)
			{
				payload[16] = 0xff;
			}
			else
			{
				//²åÈë×î½ü½ÓÊÕµ½µÄ±êÇ©
				if(astRFID[0].carAge<ultrasonic.rxTag_recent_time_const)
				{
					payload[16]=astRFID[0].carTagID[0];
					payload[17]=astRFID[0].carTagID[1];
					payload[18]=astRFID[0].carTagID[2];
					payload[19]=astRFID[0].carTagID[3];
					payload[20]=astRFID[0].carSta[0];
					payload[21]=astRFID[0].carSta[1];
				}
				else
				{
					payload[16] = 0xfd;
				}
			}
			payload[22]=ultrasonic.state;//ÓĞÎŞÕÏ°­
			payload[23]=ultrasonic.threshold_const;
			payload[24]=ultrasonic.Distance>>8;
			payload[25]=ultrasonic.Distance;
			//XOR
			for(ii=31,jj=0;ii;ii--) jj^=payload[ii];
			payload[ii]=jj;
			
			//·¢ËÍ
			#if LNA_Board	
			RFN_TX_MODE;//RW=0-½ÓÊÕ  RW=1-·¢ËÍ	
			#endif
			//³¬Éù²¨±»×èµ²,ÉäÆµÖĞ¶ÏÖĞ¿ªÆôÅäÖÃÆµµÀ½ÓÊÕ£¬È»ºó50msÄÚÍê³ÉÅäÖÃÆµµÀµÄÊı¾İ½»»¥£¬ÏêÏ¸ËµÃ÷¼ûÉäÆµÁ÷³ÌÍ¼ÎÄµ
			if(withRFRx)//Ğ¯´ø½ÓÊÕ´°¿Ú
			{
				withRFRx = 0;//clear recive window
				config_send_state = 1;//rfÖĞ¶ÏÔÊĞí½øÈëÅäÖÃ½ÓÊÕ
				timeout_cnt_start = 1;//¿ªÊ¼³¬Ê±¼ÆÊı
				timeout_cnt = 0;//³¬Ê±¼ÆÊıÖµ£¬50ms»Øµ½Êı¾İÆµµÀ½ÓÊÕ
			}			
			
			radio_tx_carrier(my_tx_power,RADIO_MODE_MODE_Nrf_1Mbit,data_channel);
			radio_channel = run_data_channel;
//			ucCycRFID  = 0;
		}
		if( timeout_cnt>=1 )//time-out
		{
			timeout_cnt_start = 0;
			timeout_cnt = 0;
			rf_cfg_state =config_timeout; //time-out	
		}
		if(rf_cfg_state>1)//time-out or success,switch to rx datachannel
		{
			#if LNA_Board	
			RFN_RX_MODE;//RW=0-½ÓÊÕ  RW=1-·¢ËÍ	
			#endif
			radio_rx_carrier(RADIO_MODE_MODE_Nrf_1Mbit,data_channel);
			radio_channel = run_data_channel;
			//clear 
			bRFRxed = 0;
			bRFRPD = 0;
			config_send_state = 0;
			rf_cfg_state = 0;
		}
	}
}
/*´®¿Ú½ÓÊÕÖĞ¶Ï*/
void UART0_IRQHandler()
{
	uint8_t rx_temp;
	static uint8_t US_state;
	if(NRF_UART0->EVENTS_RXDRDY)
	{
		NRF_UART0->EVENTS_RXDRDY=0;
		rx_temp = NRF_UART0->RXD;
		switch(US_state)
		{
			case 0:
				ultrasonic.Distance = rx_temp <<8;US_state =1;break;
			case 1:
				ultrasonic.Distance |=rx_temp;
				US_state=0;
				if(0==(ultrasonic.Distance&0xc000))
				{
					ultrasonic.RX_Dis_Flag = 1;
				}
				break;
				default:break;
		}
		
	}
	else if(NRF_UART0->EVENTS_ERROR)
	{
		NRF_UART0->EVENTS_ERROR = 0;
	}
}

/*¶Á¿¨Æ÷ÏÂ·¢
payload[1]:	7~4:±£Áô
						3~0:8-³£¹æĞÅÏ¢ 10-Ğ¯´øÖ¸ÁîÀ©Õ¹ĞÅÏ¢
payload[2~5]:	µ±ÎªÉè±¸IDÊ±£¬¶ÔÖ¸¶¨Éè±¸½øĞĞ²ÎÊıÅäÖÃ
							µ±IDÎªfffffffeÊ±£¬¶ÔËùÓĞ485½ÓÊÕÆ÷½øĞĞÅäÖÃ
payload[6]:±£Áô
payload[7]:¶ÁĞ´Æ÷¶ÌºÅ
payload[8]~[11]:±£Áô
payload[12]:7~4ÓĞ¶¨Òå
						3~0:
								0:¶Á
								1£ºĞ´
								2£º²Á³ı
								3£º±£Áô
								4£º¶ÁÔËĞĞ²ÎÊı(ram)
								5£ºĞ´ÔËĞĞ²ÎÊı(ram,µôµç¶ªÊ§£©
								6£º¶Áram¿é
								7£ºÉèÖÃ²âÊÔ±êÇ©
								8£ºÉÏ±¨²âÊÔ½á¹û
								9£º±£Áô
								10£º»½ĞÑ
								11£ºĞİÃß
	payload[13]:7~6:ÄÚ²¿FLASHÒ³µØÖ·
									0£ºÄÚ²¿²ÎÊıÇø
									1£º±£ÁôÇø
									2£ºÓÃ»§Çø1
									3£ºÓÃ»§Çø2
								5: ¼ÇÂ¼Æ«ÒÆÓĞĞ§ĞÔ:1ÓĞĞ§
							4~0:¼ÇÂ¼Æ«ÒÆ
									Ò³0ºÍ1¸÷16Ìõ¼ÇÂ¼0~15
									Ò³2ºÍ3¸÷32Ìõ¼ÇÂ¼0~31
	payload[14]:±£Áô
	payload[15]:±£Áô
	payload[16]~payload[20]: ÉäÆµµØÖ·E7E7E7E7E7
	payload[21]:Êı¾İÆµµÀ
	payload[22]:ÅäÖÃÆµµÀ
	payload[23]:¶ÌºÅ
	payload[24]:7~6:±£Áô
							5~4:·¢Éä¹¦ÂÊ
									0:(-16dbm)
									1:(-8dbm)
									2:(0dbm)
									3:(4dbm)
							3~2:×Ô¶¯ÉÏ±¨Ğ¯´øµÄĞÅÏ¢À´Ô´
									0£º±£ÁôÇø
									1£ºÓÃ»§Çø1
									2£ºÓÃ»§Çø2
							1£º±£Áô
							0£º±£Áô
	payload[25]:±£Áô
	payload[26]:±£Áô
	payload[27]:´ı¶¨
	payload[28]:
							0xfe-½ÓÊÕËùÓĞµÄ³¬Éù²¨±êÇ©
							0xfd-½ÓÊÕÆ¥ÅäµÄ³¬Éù²¨±êÇ©
							0xfc-²»½ÓÊÜ³¬Éù²¨±êÇ©
	payload[29]~payload[31]:°ó¶¨µÄ³¬Éù²¨ID£¬µÍ3×Ö½Ú
	*/
	
	
//Ğ´ÔÚÖĞ¶ÏÀï£¬·ÀÖ¹ÒÔºóÈç¹ûÌí¼ÓflashĞ¾Æ¬Ê±£¬Ö÷º¯ÊıÒªµÈ´ıºÜ¾Ã
void radio_cmd_process(const uint8_t* RxPayload,uint8_t isack)
{
	uint8_t cmd;//ÃüÁî
	uint8_t rx_page_addr;//flashÒ³µØÖ·
	uint8_t rx_page_offset;//Æ«ÒÆÁ¿
	uint8_t rx_offset_valid;//¼ÇÂ¼Æ«ÒÆÓĞĞ§
	uint8_t set_page_offset;
//	uint8_t rx_radio_power;
//	uint8_t rx_radio_power_valid;
	uint8_t error_flag;
	uint8_t ack;
	uint8_t m,n;
	uint8_t temp_flash1[16];
	uint8_t *ptROM;//ÁÙÊ±Ö¸Õërom
//	uint8_t flash_full_flag;

	uint8_t TxPaylod[32];
	copybuf(TxPaylod,RxPayload,32);
	
	//power
//	rx_radio_power_valid = (TxPaylod[17]&0xc0);
//	rx_radio_power = (TxPaylod[24]&0x30)>>4;	
	//checkÖ¸Áî
	cmd=TxPaylod[12]&0x0f;	//Ö¸Áî0-read 1-write 2-erase 3-reserve 4-read run-para 
	rx_page_addr=TxPaylod[13]>>6;		//Ò³µØÖ·
	rx_offset_valid = (TxPaylod[13]&0x20)>>5;//¼ÇÂ¼Æ«ÒÆÓĞĞ§
	rx_page_offset = TxPaylod[13]&0x1f;	
	error_flag=0;	//0-pass, 1-error
	ack = 0;
	switch(cmd)
	{
		case 0://read

			if(rx_page_addr<2&&rx_page_offset>15) {error_flag=1;break;}	//³¬³ö¶ÁÈ¡·¶Î§16Æ«ÒÆÁ¿
			if(0 == rx_offset_valid)//ÎŞĞ§£¬·µ»Ø×îĞÂ²ÎÊı
			{
				pROM=caucpROM[rx_page_addr];
				set_page_offset=*pROM;
				if(set_page_offset) set_page_offset--;//*ucpROM±íÊ¾ÊÇ¼ÇÂ¼¸öÊı£¬ËùÒÔÒª¼õ1
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
		
			if(TxPaylod[13]&0x20) {error_flag=1;break;}	//ÅäÖÃ²ÎÊıÊ±£¬¼ÇÂ¼Æ«ÒÆÓĞĞ§Î»Îª0.
			if(0 == rx_page_addr)//page0-set£¬ÄÚ²¿²ÎÊı		
			{				
				set_page_offset = ucROM0;//×îĞÂÆ«ÒÆÁ¿
				if(set_page_offset) set_page_offset--;
				addr = Page_Base[rx_page_addr];
				m = set_page_offset*Rom_record_byte[rx_page_addr];
				nrf_nvmc_read_bytes(addr+m,temp_flash1,Rom_record_byte[rx_page_addr]);
				//1-10£¬Ö¸Ïò¼ÇÂ¼²ÎÊı£¬0Ö¸ÏòÔ¤Éè²ÎÊı
/*
[0]	.7~6 	0		Ó²¼şÀàĞÍ
		.5~0	0~15	°æ±¾ºÅ
[1]	.7~4	6		·¢Éä¹¦ÂÊ£º0- (-30dBm), 1- (-20dBm), 2- (-16dBm), 3- (-12dBm)
													4- (8dBm), 5- (4dBm), 6- (0dBm), 7- (4dBm)
		.3~2	0~2		×Ô¶¯ÉÏ±¨Ğ¯´øµÄĞÅÏ¢À´Ô´£º0-±£ÁôÇø£¬1-ÓÃ»§Çø1, 2-ÓÃ»§Çø2
		.1~0	0		Ê¹ÄÜÖÜÆÚ·¢Éä 1-·¢Éä
[2]	.7~0	0		±£Áô
[3]	.7~0	0		±£Áô
[4]	.7		0		±£Áô		
		.6		0/1		Ë¥¼õ·½°¸£º0-ÊÕ·¢¶Ô³ÆË¥¼õ£»1-½ÓÊÕË¥¼õ
		.5		0/1		¿ªÆôRPD¹ıÂË£º1-¿ªÆô
		.4~0	0~31	Ë¥¼õÖµ£¬0~31-×îĞ¡0dB×î´ó31dB
[5~8]				°ó¶¨µÄ³¬Éù²¨Ì½²â±êÇ©ID£¬MSBÔÚÏÈ£º
[5]	.7~0	0xfe~0xfc
							0xFEXXXXXX-½ÓÊÕËùÓĞµÄ³¬Éù²¨±êÇ©£¨ÎŞĞè°ó¶¨£©
							0xFDXXXXXX-½ÓÊÕÆ¥ÅäµÄ³¬Éù²¨±êÇ©£¨°ó¶¨£©
							0xFCXXXXXX-²»½ÓÊÕ³¬Éù²¨±êÇ©				
*/
				ptROM=ucROM0?temp_flash1:CurParaSet;
		
				//Èç¹û·¢ËÍÏÂÀ´µÄ²ÎÊıÎŞĞ§£¬¶ªµôÕâ¶Îpayload£¬²¢ÇÒ»Ø¸´µÄÉÏ´Î×îĞÂ¼ÇÂ¼µÄ²ÎÊı
				//·¢Éä¹¦ÂÊ 7~6²»µÈÓÚ00£¬ÎŞĞ§ 5~4µÈÓÚ11ÎŞĞ§                                                               
				if(TxPaylod[24]&0x82)//µÚ8/2bit²»Îª1.
				{
					TxPaylod[24] = ptROM[8];
				}
				//×Ô¶¯Ğ¯´øĞÅÏ¢À´Ô´
				TxPaylod[25] = ptROM[9];	
				TxPaylod[26] = ptROM[10];
					//rssi
				if(TxPaylod[27]&0x80)
				{
					TxPaylod[27]=ptROM[11];
				}	
				if(TxPaylod[28]==0XFF)
				{
					TxPaylod[28] = ptROM[12]; 
				}
				if(TxPaylod[29]>=17) TxPaylod[29]=ptROM[13];
				if(TxPaylod[30]>=17) TxPaylod[30]=ptROM[14];
				if(TxPaylod[31]>=2) TxPaylod[31]=ptROM[15];
				//read-only
				TxPaylod[16]=ParaSet[0];
				TxPaylod[17]=ParaSet[1];
				TxPaylod[18]=ParaSet[2];
				TxPaylod[19]=ParaSet[3];
				TxPaylod[20]=ParaSet[4];
				TxPaylod[21]=ParaSet[5];
				TxPaylod[22]=ParaSet[6];
				TxPaylod[23]=ParaSet[7];	
				//±È½ÏĞ´ÈëÄÚÈİÓë×î½ü¼ÇÂ¼£¬²»ÖØ¸´Ğ´Èë
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
					//½«ÔËĞĞ²ÎÊı±£´æÔÚacuSetÖĞ£¬ºóĞøÉèÖÃ²ÎÊı
					//ÉèÖÃÉúĞ§,[0]~[6] read-only
					for(m=7;m<16;m++) CurParaSet[m]=TxPaylod[m+16];//update parameter
					ucROM0++;
					set_page_offset = ucROM0-1;
					my_tx_power = (uint32_t)caucTXPOWER[CurParaSet[8]>>4];//transmit power

					ultrasonic.confirm_cnt_const = (CurParaSet[12]>>4) + 1; //È·ÈÏ´ÎÊı
					ultrasonic.delay_send_cnt_const = (CurParaSet[12]&0x0f);//ÑÓ³Ù·¢ËÍ´ÎÊı
					ultrasonic.threshold_const = CurParaSet[13]<<8; //Ì½²âÃÅÏŞ
					ultrasonic.rxTag_recent_time_const = CurParaSet[14];
					debug_mode = CurParaSet[15];
					rssi = CurParaSet[11];
					#if 0
					En_Period_Tx = CurParaSet[8]&0x01;
					#endif
					TxPaylod[13] |= 0x20;
				}
				else
				{
					TxPaylod[13] &= (~0x20);
				}
			}
			else if(1 == rx_page_addr || 2==rx_page_addr || 3 == rx_page_addr)//reserve page,user page1,user page2
			{					
				//²»Ğ´¿Õ¼ÇÂ¼
				for(m=16;m<32;m++)
				{
					if(TxPaylod[m]!=0xff) break;	
				}
				if(m>=32) 
				{
					error_flag=1;
					break;
				}
				//±È½ÏĞ´ÈëÄÚÈİÓë×î½ü¼ÇÂ¼£¬²»ÖØ¸´Ğ´Èë
				pROM = caucpROM[rx_page_addr];//¼ÇÂ¼¸öÊı
				set_page_offset = *pROM;
				if(set_page_offset>0) set_page_offset--;
				addr = Page_Base[rx_page_addr]; //BASE ADDR
				m = set_page_offset*Rom_record_byte[rx_page_addr];//offset addr
				nrf_nvmc_read_bytes(addr+m,temp_flash1,Rom_record_byte[rx_page_addr]);
				for(m=0;m<Rom_record_byte[rx_page_addr];m++)//²»ÖØ¸´Ğ´Èë
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
		
			pROM = caucpROM[rx_page_addr];//¼ÇÂ¼¸öÊı
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
		
			//²»Ğ´¿Õ¼ÇÂ¼
			for(m=16;m<32;m++)
			{
				if(TxPaylod[m]!=0xff) break;	
			}
			if(m>=32) error_flag=1;
			else
			{
				//check para
				if(TxPaylod[24]&0x02)//µÚ2bit²»Îª1.
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
			if(1==ack)//¶ÁĞ´»Ø¸´
			{
				#if 1
				m = TxPaylod[13]&0xe0;//7~6Î»Ò³ÃæÖ¸Ê¾	
//				m |=0x20;//Æ«ÒÆÓĞĞ§
				TxPaylod[13] = m | set_page_offset;
				addr = Page_Base[rx_page_addr];
				m = set_page_offset *Rom_record_byte[rx_page_addr];
				nrf_nvmc_read_bytes(addr+m,&TxPaylod[16],Rom_record_byte[rx_page_addr]);
				#endif
				if(0 == rx_page_addr)
				{
					TxPaylod[23]=(HWTYP<<6)|(VERSION&0x3f);
				}
			}
			else if(2==ack)//²Á³ı
			{
				addr = Page_Base[rx_page_addr];
				nrf_nvmc_read_bytes(addr,&TxPaylod[16],Rom_record_byte[rx_page_addr]);
				TxPaylod[13] = TxPaylod[13]&0xc0;
			}
			else if(4 == ack)//read ram
			{
				TxPaylod[23]=(HWTYP<<6)|(VERSION&0x3f);//²åÈëÓ²¼şÀàĞÍºÍÈí¼ş°æ±¾: Ô­¶ÌºÅÎ»ÖÃ
			}
			else if(5 == ack)//write ram
			{
				for(m=7;m<16;m++) CurParaSet[m] = TxPaylod[16+m];
				my_tx_power = (uint32_t)caucTXPOWER[CurParaSet[8]>>4];//transmit power
				ultrasonic.confirm_cnt_const = (CurParaSet[12]>>4) + 1; //È·ÈÏ´ÎÊı
				ultrasonic.delay_send_cnt_const = (CurParaSet[12]&0x0f);//ÑÓ³Ù·¢ËÍ´ÎÊı
				ultrasonic.threshold_const = CurParaSet[13]<<8; //Ì½²âÃÅÏŞ
				ultrasonic.rxTag_recent_time_const = CurParaSet[14];
				debug_mode = CurParaSet[15];
				rssi = CurParaSet[11];
				#if 0
				En_Period_Tx = CurParaSet[8]&0x01;
				#endif
				
			}
			TxPaylod[1] = 0x02;//ÉÏĞĞ
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
			RFN_TX_MODE;//RW=0-½ÓÊÕ  RW=1-·¢ËÍ	
			#endif
			copybuf(payload,TxPaylod,32);
			radio_tx_carrier(my_tx_power,RADIO_MODE_MODE_Nrf_1Mbit, config_channel);
			config_send_state = 2;//ack
			radio_channel = run_config_channel;
		}
	}
}

/*¶Á¿¨Æ÷ÏÂ·¢
payload[1]:	7~4:±£Áô
						3~0:8-³£¹æĞÅÏ¢ 10-Ğ¯´øÖ¸ÁîÀ©Õ¹ĞÅÏ¢
payload[2~5]:	µ±ÎªÉè±¸IDÊ±£¬¶ÔÖ¸¶¨Éè±¸½øĞĞ²ÎÊıÅäÖÃ
							µ±IDÎªfffffffeÊ±£¬¶ÔËùÓĞ485½ÓÊÕÆ÷½øĞĞÅäÖÃ
payload[6]:±£Áô
payload[7]:¶ÁĞ´Æ÷¶ÌºÅ
payload[8]~[11]:±£Áô
payload[12]:7~4ÓĞ¶¨Òå
						3~0:
								0:¶Á
								1£ºĞ´
								2£º²Á³ı
								3£º±£Áô
								4£º¶ÁÔËĞĞ²ÎÊı(ram)
								5£ºĞ´ÔËĞĞ²ÎÊı(ram,µôµç¶ªÊ§£©
								6£º¶Áram¿é
								7£ºÉèÖÃ²âÊÔ±êÇ©
								8£ºÉÏ±¨²âÊÔ½á¹û
								9£º±£Áô
								10£º»½ĞÑ
								11£ºĞİÃß
	payload[13]:7~6:ÄÚ²¿FLASHÒ³µØÖ·
									0£ºÄÚ²¿²ÎÊıÇø
									1£º±£ÁôÇø
									2£ºÓÃ»§Çø1
									3£ºÓÃ»§Çø2
								5: ¼ÇÂ¼Æ«ÒÆÓĞĞ§ĞÔ:1ÓĞĞ§
							4~0:¼ÇÂ¼Æ«ÒÆ
									Ò³0ºÍ1¸÷16Ìõ¼ÇÂ¼0~15
									Ò³2ºÍ3¸÷32Ìõ¼ÇÂ¼0~31
	payload[14]:±£Áô
	payload[15]:±£Áô
	payload[16]~payload[20]: ÉäÆµµØÖ·E7E7E7E7E7
	payload[21]:Êı¾İÆµµÀ
	payload[22]:ÅäÖÃÆµµÀ
	payload[23]:¶ÌºÅ
	payload[24]:7~6:±£Áô
							5~4:·¢Éä¹¦ÂÊ
									0:(-16dbm)
									1:(-8dbm)
									2:(0dbm)
									3:(4dbm)
							3~2:×Ô¶¯ÉÏ±¨Ğ¯´øµÄĞÅÏ¢À´Ô´
									0£º±£ÁôÇø
									1£ºÓÃ»§Çø1
									2£ºÓÃ»§Çø2
							1£º±£Áô
							0£º±£Áô
	payload[25]:±£Áô
	payload[26]:±£Áô
	payload[27]:´ı¶¨
	payload[28]:
							0xfe-½ÓÊÕËùÓĞµÄ³¬Éù²¨±êÇ©
							0xfd-½ÓÊÕÆ¥ÅäµÄ³¬Éù²¨±êÇ©
							0xfc-²»½ÓÊÜ³¬Éù²¨±êÇ©
	payload[29]~payload[31]:°ó¶¨µÄ³¬Éù²¨ID£¬µÍ3×Ö½Ú
*/

void RADIO_IRQHandler()
{
	uint8_t radio_uc1,radio_uc2;
//	uint8_t rfid_uc1,rfid_uc2;
	uint8_t b1,b2,bAck;
	static uint8_t count;
  
	if(1 == NRF_RADIO->EVENTS_END)
	{
		NRF_RADIO->EVENTS_END = 0;
		if(radio_status == RADIO_STATUS_TX)
		{
			if(1==config_send_state)
			{
				#if LNA_Board	
				RFN_RX_MODE;//RW=0-½ÓÊÕ  RW=1-·¢ËÍ
				#endif
				radio_rx_carrier(RADIO_MODE_MODE_Nrf_1Mbit, config_channel);//switch to receive 
				radio_channel = run_config_channel;
				rf_cfg_state = config_start;//¿ªÊ¼½ÓÊÕÅäÖÃ²ÎÊı
				count = 0;
			}
			else if(2 == config_send_state)
			{
				if(count++<=3)
				{
					#if LNA_Board	
					RFN_TX_MODE;//RW=0-½ÓÊÕ  RW=1-·¢ËÍ	
					#endif
					radio_tx_carrier(my_tx_power,RADIO_MODE_MODE_Nrf_1Mbit, config_channel);
					radio_channel = run_config_channel;					
				}
				else
				{
					rf_cfg_state = config_success;
				}
			}
			else
			{
				#if LNA_Board	
				RFN_RX_MODE;//RW=0-½ÓÊÕ  RW=1-·¢ËÍ	
				#endif
				radio_rx_carrier(RADIO_MODE_MODE_Nrf_1Mbit,data_channel);
				radio_channel = run_data_channel;
			}
		}
		else if(RADIO_STATUS_RX == radio_status)
		{
			if(NRF_RADIO->CRCSTATUS)
			{
				//XORÒì»òÎª0 ±íÊ¾Êı¾İÓĞĞ§
				for(radio_uc1=0,radio_uc2=0;radio_uc2<32;radio_uc2++)
				{
					radio_uc1^=payload[radio_uc2];
				}
				if(!radio_uc1)
				{
					//¾ßÌå¿´Ğ­ÒéÎÄµµ-Éè¼ÆËµÃ÷
					if(run_config_channel == radio_channel)
					{
						if((payload[1]&0x0a)==0x0a)//ÏÂĞĞ£¬Ğ¯´øÀ©Õ¹Ö¸Áî
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
							&&payload[5]==0xfd)//Èç¹ûÊÇ¹ã²¥ĞÅÏ¢£¬Ò»ÂÉ²»»Ø¸´¡£
							{
								b1=1;
//								if(payload[5]!=0xff) b2=1;
							}
							if(b1)
							{
								bAck=b2;
								bRFRxed=1;//ÊÕµ½¶Á¿¨Æ÷Ö¸Ê¾
								if(NRF_RADIO->RSSISAMPLE<rssi)
								{
									bRFRPD=1;//ĞÅºÅÇ¿¶ÈÓĞĞ§
								}
								#if 0
								if(!bAck)//50ms timeout,back to rx data channel
								{
									//·µ»ØÊı¾İÆµµÀ½ÓÊÕ
									#if LNA_Board	
									RFN_RX_MODE;//RW=0-½ÓÊÕ  RW=1-·¢ËÍ	
									#endif
									radio_rx_carrier(RADIO_MODE_MODE_Nrf_1Mbit,data_channel);
									RFN_RX_MODE;//RW=0-½ÓÊÕ  RW=1-·¢ËÍ
									radio_channel = run_data_channel;
								}
								#endif
								radio_cmd_process(&payload[0],bAck);
												
							}
						}
					}
					else if(run_data_channel == radio_channel)
					{
						if(NRF_RADIO->RSSISAMPLE<rssi)
						{
							if(0 == (payload[1]&0x0a)&&payload[2]<0xfd) //ultrasonic id
							{
								astRFID[0].carTagID[0] = payload[2];
								astRFID[0].carTagID[1] = payload[3];
								astRFID[0].carTagID[2] = payload[4];
								astRFID[0].carTagID[3] = payload[5];
								astRFID[0].carSta[0] = payload[6];
								astRFID[0].carSta[1] = payload[7];
								astRFID[0].carAge = 0;
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
						

void TIMER0_IRQHandler()
{
	if(NRF_TIMER0->EVENTS_COMPARE[0])
	{
		//RFID 2S¶¨Ê±
		ucCycRFID++;
		if(ucCycRFID>=RFID_CYCLE)
		{
			ucCycRFID = 0;
//			RFID_TX_EN = 1;
		}
		if(timeout_cnt_start)
		{
			timeout_cnt++;
		}
		US_Cnt++;
		if(US_Cnt>US_CYCLE)
		{
			US_Cnt = 0;
			US_Start_Measure = 1;
		}
		//rfid config recive time-out
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
	}
}


