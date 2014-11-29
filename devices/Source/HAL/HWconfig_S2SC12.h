/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_S2SC12_H
#define HWCONFIG_S2SC12_H

// Board: STM32F0DISCOVERY, MB1034B
// uC: STM32F051
// PHY1: UART
// PHY2: CC11/Anaren

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
#define EXTDIO_PORTNUM2MASK         {(uint16_t)0x0600, (uint16_t)0x0000, (uint16_t)0x0000}
// End DIO Section

// UART Section
#define UART_PHY_PORT               0   //  0 - USART1 PA9,PA10 GPIOA MASK 0x0600, 1 - USART2 PA2,PA3 GPIO MASK 0x000C
#define EXTSER_USED                 1
#define EXTSER_PORT2UART            {1}
// End UART Section

// CC11 Section
#define CC11_ANAREN                 1
#define CC11_USE_SPI                2   // 1 - SPI1 PA4-PA7, 2 - SPI2 PB12-PB15

#if (CC11_USE_SPI == 1)
#define CC11_NSS_PORT               GPIOA
#define CC11_NSS_PIN                GPIO_Pin_4

#define SPIc_PORT                   GPIOA
#define SPIc_SCK_PIN                GPIO_Pin_5
#define SPIc_SCK_PINSRC             GPIO_PinSource5
#define SPIc_MISO_PIN               GPIO_Pin_6
#define SPIc_MISO_PINSRC            GPIO_PinSource6
#define SPIc_MOSI_PIN               GPIO_Pin_7
#define SPIc_MOSI_PINSRC            GPIO_PinSource7

#define SPIc                        SPI1

#elif (CC11_USE_SPI == 2)

#define CC11_NSS_PORT               GPIOB
#define CC11_NSS_PIN                GPIO_Pin_12

#define SPIc_PORT                   GPIOB
#define SPIc_SCK_PIN                GPIO_Pin_13
#define SPIc_SCK_PINSRC             GPIO_PinSource13
#define SPIc_MISO_PIN               GPIO_Pin_14
#define SPIc_MISO_PINSRC            GPIO_PinSource14
#define SPIc_MOSI_PIN               GPIO_Pin_15
#define SPIc_MOSI_PINSRC            GPIO_PinSource15

#define SPIc                        SPI2

#else
#error unknown CC11 configuration
#endif  //  CC11_USE_SPI

#define RF_WAIT_LOW_MISO()          while(SPIc_PORT->IDR & SPIc_MISO_PIN)

#define RF_SELECT()                 CC11_NSS_PORT->BRR = CC11_NSS_PIN
#define RF_RELEASE()                CC11_NSS_PORT->BSRR = CC11_NSS_PIN

// End CC11 Section

#define UART_PHY                    1
#define CC11_PHY                    2

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0xFF

#define PHY2_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY2         (PHY2_ADDR_t)0x00
#define ADDR_UNDEF_PHY2             (PHY2_ADDR_t)0xFF

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'S'
#define OD_DEV_UC_SUBTYPE           '2'
#define OD_DEV_PHY1                 'S'
#define OD_DEV_PHY2                 'C'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '2'

#define OD_ADDR_TYPE                objUInt8

#include "../PHY/UART/uart_phy.h"
#include "../PHY/CC1101/cc11_phy.h"

#define PHY1_Init                   UART_Init
#define PHY1_Send                   UART_Send
#define PHY1_Get                    UART_Get
#define PHY1_GetAddr                UART_GetAddr
#define PHY1_NodeId                 objRFNodeId
#define PHY1_GateId                 objGateID

#define PHY2_Init                   CC11_Init
#define PHY2_Send                   CC11_Send
#define PHY2_Get                    CC11_Get
#define PHY2_GetRSSI                CC11_GetRSSI
#define PHY2_GetAddr                CC11_GetAddr
#define PHY2_NodeId                 objRFNodeId

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_S2SN12_H
