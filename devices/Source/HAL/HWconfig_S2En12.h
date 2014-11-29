/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_S2EN12_H
#define HWCONFIG_S2EN12_H

// Board: STM32F0DISCOVERY, MB1034B
// uC: STM32F051
// PHY1: ENC28J60

// GPIOA
// Pin  Port    Func
//   0  PA0
//   1  PA1
//   2  PA2     USART2_TX
//   3  PA3     USART2_RX
//   4  PA4     SPI1_NSS
//   5  PA5     SPI1_SCK
//   6  PA6     SPI1_MISO
//   7  PA7     SPI1_MOSI
//   8  PA8
//   9  PA9     USART1_TX
//  10  PA10    USART1_RX
//  11  PA11
//  12  PA12
//  13  PA13
//  14  PA14
//  15  PA15
// GPIOB
//  16  PB0
//  17  PB1
//  18  PB2
//  19  PB3
//  20  PB4
//  21  PB5
//  22  PB6
//  23  PB7
//  24  PB8
//  25  PB9
//  26  PB10
//  27  PB11
//  28  PB12    SPI2_NSS
//  29  PB13    SPI2_SCK
//  30  PB14    SPI2_MISO
//  31  PB15    SPI2_MOSI
// GPIOC
//  32  PC0
//  33  PC1
//  34  PC2
//  35  PC3
//  36  PC4
//  37  PC5
//  38  PC6
//  39  PC7
//  40  PC8     LED_Blue
//  41  PC9     LED_Green
//  42  PC10
//  43  PC11
//  44  PC12
//  45  PC13
//  46  PC14
//  47  PC15
// GPIOD, GPIOF - Not used


#ifdef __cplusplus
extern "C" {
#endif

#include "STM32/hal.h"

// DIO Section
#define EXTDIO_USED                 1
#define EXTDIO_MAXPORT_NR           3
#define EXTDIO_PORTNUM2PORT         {GPIOA, GPIOB, GPIOC}
#define EXTDIO_PORTNUM2MASK         {(uint16_t)0x0000, (uint16_t)0xF000, (uint16_t)0x0000}
// End DIO Section

// Analogue Inputs
//#define EXTAIN_MAXPORT_NR           9           // ADC0-ADC7, Vbg
//#define EXTAIN_BASE_2_APIN          {0, 1, 2, 3, 4, 5, 6, 7, 14}
//#define EXTAIN_REF                  0x02        // Bit0 - Ext, Bit1 - Vcc, Bit2 - Int1, Bit3 - Int2
// End Analogue Inputs

// UART Section
#define EXTSER_USED                 1
#define EXTSER_PORT2UART            {1, 0}
// End UART Section

// ENC Section
#define ENC_USE_SPI                 2   // 1 - SPI1 PA4-PA7, 2 - SPI2 PB12-PB15

#if (ENC_USE_SPI == 1)
#define ENC_NSS_PORT                GPIOA
#define ENC_NSS_PIN                 GPIO_Pin_4

#define SPIe                        SPI1

#elif (ENC_USE_SPI == 2)
#define ENC_NSS_PORT                GPIOB
#define ENC_NSS_PIN                 GPIO_Pin_12

#define SPIe                        SPI2

#endif  //  ENC_USE_SPI

#define ENC_SELECT()                ENC_NSS_PORT->BRR = ENC_NSS_PIN
#define ENC_RELEASE()               {while(SPIe->SR & SPI_SR_BSY); ENC_NSS_PORT->BSRR = ENC_NSS_PIN;}
// End ENC Section

#define ENC28J60_PHY                1

#define PHY1_ADDR_t                 uint32_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)inet_addr(255,255,255,255)
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)inet_addr(255,255,255,255)

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'S'
#define OD_DEV_UC_SUBTYPE           '2'
#define OD_DEV_PHY1                 'E'
#define OD_DEV_PHY2                 'n'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '2'

#define OD_ADDR_TYPE                objUInt32
#define OD_DEV_MAC                  {0x00,0x04,0xA3,0x00,0x00,0x03}   // MAC MSB->LSB
//#define OD_DEF_IP_ADDR              inet_addr(192,168,10,202)
//#define OD_DEF_IP_MASK              inet_addr(255,255,255,0)
//#define OD_DEF_IP_ROUTER            inet_addr(192,168,10,1)
//#define OD_DEF_IP_BROKER            inet_addr(192,168,20,8)

#include "../PHY/ENC28J60/enc28j60_phy.h"

#define PHY1_Init                   ENC28J60_Init
#define PHY1_Send                   ENC28J60_Send
#define PHY1_Get                    ENC28J60_Get
#define PHY1_GetAddr                ENC28J60_GetAddr
#define PHY1_NodeId                 objIPAddr
#define PHY1_GateId                 objIPBroker

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_S2EN12_H
