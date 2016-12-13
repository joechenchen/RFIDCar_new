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
#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H
#include "nrf.h"

#define PACKET_BASE_ADDRESS_LENGTH       (4UL)  //!< Packet base address length field size in bytes
#define PACKET_STATIC_LENGTH             (32UL)  //!< Packet static length in bytes
#define PACKET_PAYLOAD_MAXSIZE           (PACKET_STATIC_LENGTH)  //!< Packet payload maximum size in bytes
//2.4G²¿·Ö
typedef enum
{
    RADIO_STATUS_IDLE = 1,
    RADIO_STATUS_RX,
    RADIO_STATUS_TX,

}RADIO_Status;

void radio_configure(void);
void Radio_Init(void);
void radio_modulated_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel);
void radio_rx_carrier(uint8_t mode, uint8_t channel);
void radio_modulated_rx_carrier(uint8_t mode, uint8_t channel);
void startTX(void);
void startRX(void);
void radio_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel);
void radio_disable(void);
#endif
