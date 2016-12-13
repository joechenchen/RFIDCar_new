/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
/** @file
* @addtogroup nrf_dev_radio_rx_example_main nrf_dev_radio_tx_example_main
* @{
*/
 
#include "radio_config.h"
//#include "nrf_delay.h"

#define SHORTCUT_EN 0
#define RSSI_EN 1
/* These are set to zero as Shockburst packets don't have corresponding fields. */
#define PACKET_S1_FIELD_SIZE             (0UL)  /**< Packet S1 field size in bits. */
#define PACKET_S0_FIELD_SIZE             (0UL)  /**< Packet S0 field size in bits. */
#define PACKET_LENGTH_FIELD_SIZE         (0UL)  /**< Packet length field size in bits. */

uint8_t payload[PACKET_PAYLOAD_MAXSIZE];
uint8_t radio_status = RADIO_STATUS_IDLE;
/** 
 * @brief Function for configuring the radio to operate in Shockburst compatible mode.
 * 
 * To configure the application running on nRF24L series devices:
 *
 * @verbatim
 * uint8_t tx_address[5] = { 0xC0, 0x01, 0x23, 0x45, 0x67 };
 * hal_nrf_set_rf_channel(7);
 * hal_nrf_set_address_width(HAL_NRF_AW_5BYTES); 
 * hal_nrf_set_address(HAL_NRF_TX, tx_address);
 * hal_nrf_set_address(HAL_NRF_PIPE0, tx_address); 
 * hal_nrf_open_pipe(0, false);
 * hal_nrf_set_datarate(HAL_NRF_1MBPS);
 * hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);
 * hal_nrf_setup_dynamic_payload(0xFF);
 * hal_nrf_enable_dynamic_payload(false);
 * @endverbatim
 *
 * When transmitting packets with hal_nrf_write_tx_payload(const uint8_t *tx_pload, uint8_t length),
 * match the length with PACKET_STATIC_LENGTH.
 * hal_nrf_write_tx_payload(payload, PACKET_STATIC_LENGTH);
 * 
*/
/*射频参数配置*/
void radio_configure()
{
    // Radio config
    //NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
    //NRF_RADIO->FREQUENCY = 79UL;           // Frequency bin 7, 2407MHz
    //NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

    // Radio address config

		//LOGIC0:0xE7E7E7E7E7         LOGIC1:0xC200C2C2C2
		//LOGIC2:0xC300C2C2C2         LOGIC3:0xC400C2C2C2
		//LOGIC4:0xC800C2C2C2         LOGIC5:0xC700C2C2C2
		//LOGIC6:0xC600C2C2C2         LOGIC7:0xC500C2C2C2
    //通过以下配置，我们得到的逻辑地址即8个通道的地址分别为以上8个地址，
    NRF_RADIO->PREFIX0 = 0xC4C3C2E7UL;  // 逻辑地址 // Prefix byte of addresses 3 to 0
	  NRF_RADIO->PREFIX1 = 0xC5C6C7C8UL;  // Prefix byte of addresses 7 to 4
	  NRF_RADIO->BASE0   = 0xE7E7E7E7UL;  // 逻辑地址// Base address for prefix 0
    NRF_RADIO->BASE1   = 0x43434343UL;  // 逻辑地址设定 // Base address for prefix 1-7
	  //本射频协议使用通道0传输数据，即地址为0xE7E7E7E7E7
    NRF_RADIO->TXADDRESS   = 0x00UL;      // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = 0x01UL;    // Enable device address 0 to use to select which addresses to receive

    // Packet configuration
	  //设置S1长度
	  //设置S0长度
		//设置LENGTH的长度
	  //设置这三个域的长度都为0
    NRF_RADIO->PCNF0 = (PACKET_S1_FIELD_SIZE     << RADIO_PCNF0_S1LEN_Pos) |
                       (PACKET_S0_FIELD_SIZE     << RADIO_PCNF0_S0LEN_Pos) |
                       (PACKET_LENGTH_FIELD_SIZE << RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // Packet configuration
		//不使能数据加噪
		//数据高位在先
		//静态地址为4，意味着比LENGTH filed所定义的包长度多四
		//PAYLOAD最大长度为32
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |    
                       (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |
                       (PACKET_BASE_ADDRESS_LENGTH   << RADIO_PCNF1_BALEN_Pos)   |
                       (PACKET_STATIC_LENGTH         << RADIO_PCNF1_STATLEN_Pos) |
                       (PACKET_PAYLOAD_MAXSIZE       << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"
//如果使能相对应的shortcut就不需要将TASK->START设置为1了接收到radio->ready自动会发start事件
#if SHORTCUT_EN == 1
		NRF_RADIO->SHORTS = (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |
										(RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
										(RADIO_SHORTS_END_START_Enabled << RADIO_SHORTS_END_START_Pos) |
										(RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos); //|
//										(RADIO_SHORTS_DISABLED_TXEN_Enabled << RADIO_SHORTS_DISABLED_TXEN_Pos);
#endif		
    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value      
        NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
    }
    else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFUL;        // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
    }
}

/*射频初始化*/
void Radio_Init(void)
{
    // Set radio configuration parameters.
    radio_configure();
	
	  // Set payload pointer.
    NRF_RADIO->PACKETPTR = (uint32_t)payload;
		
    // Enable END interrupt
    NRF_RADIO->EVENTS_END   = 0;
    NRF_RADIO->INTENCLR     = 0xFFFFFFFFUL;
    NRF_RADIO->INTENSET     |= RADIO_INTENSET_END_Set << RADIO_INTENSET_END_Pos;
	
    NVIC_ClearPendingIRQ(RADIO_IRQn);
	  NVIC_SetPriority(RADIO_IRQn, 2);
    NVIC_EnableIRQ(RADIO_IRQn);
}

#if old_radio
void radio_modulated_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel)
{
    NRF_RADIO->TXPOWER    = (txpower << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->MODE       = (mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY  = channel;
}

void radio_rx_carrier(uint8_t mode, uint8_t channel)
{
    NRF_RADIO->MODE       = (mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY  = channel;
}

void startTX(void)
{
    radio_status = RADIO_STATUS_TX;

		// Disable radio 
    NRF_RADIO->EVENTS_DISABLED  = 0U;
    NRF_RADIO->TASKS_DISABLE    = 1U;
    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        
    }

    // Enable radio TX and wait for ready 
    NRF_RADIO->EVENTS_READY     = 0U;
    NRF_RADIO->TASKS_TXEN       = 1;
    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        
    }
    
    // Start TX 
		NRF_RADIO->EVENTS_END  = 0U;
    NRF_RADIO->TASKS_START = 1U;	
}

void startRX(void)
{  
		radio_status = RADIO_STATUS_RX;
		
    NRF_RADIO->EVENTS_DISABLED   = 0U;
    NRF_RADIO->TASKS_DISABLE     = 1U;
    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        
    } 
    NRF_RADIO->EVENTS_READY      = 0U;
    NRF_RADIO->TASKS_RXEN        = 1U;
    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        
    }
    NRF_RADIO->TASKS_START = 1U;
}

#else
//新版本函数
void radio_disable(void)
{
    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE   = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0)
    {
        // Do nothing.
    }
    NRF_RADIO->EVENTS_DISABLED = 0;
		radio_status = RADIO_STATUS_IDLE;
}


void radio_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel)
{
    radio_disable();
    NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;  //开启快捷方式，自动启动START任务
    NRF_RADIO->TXPOWER    = (txpower << RADIO_TXPOWER_TXPOWER_Pos);    
    NRF_RADIO->MODE       = (mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY  = channel;

    NRF_RADIO->TASKS_TXEN = 1;
	  radio_status = RADIO_STATUS_TX;
}


/**
 * @brief Function for starting modulated TX carrier by repeatedly sending a packet with random address and 
 * random payload.
*/
void radio_modulated_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel)
{
    radio_disable();
//    generate_modulated_rf_packet();
    NRF_RADIO->SHORTS     = RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_READY_START_Msk | \
                            RADIO_SHORTS_DISABLED_TXEN_Msk;  //循环发送
    NRF_RADIO->TXPOWER    = (txpower << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->MODE       = (mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY  = channel;
    NRF_RADIO->TASKS_TXEN = 1;
		radio_status = RADIO_STATUS_TX;
}

/**
 * @brief Function for turning on RX carrier.
*/
void radio_rx_carrier(uint8_t mode, uint8_t channel)
{
    radio_disable();
    NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;//开启快捷方式，自动启动START任务
#if RSSI_EN==1
		NRF_RADIO->SHORTS = NRF_RADIO->SHORTS|RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
#endif
    NRF_RADIO->FREQUENCY  = channel;
		NRF_RADIO->MODE       = (mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TASKS_RXEN = 1;
		radio_status = RADIO_STATUS_RX;
}
////循环接收不好，一般都处理完数据再开启接收
//void radio_modulated_rx_carrier(uint8_t mode, uint8_t channel)
//{
//    radio_disable();
//    NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;//开启快捷方式，自动启动START任务
//    NRF_RADIO->FREQUENCY  = channel;
//    NRF_RADIO->TASKS_RXEN = 1;
//		radio_status = RADIO_STATUS_RX;
//}

#endif

/** 
 * @}
 */
