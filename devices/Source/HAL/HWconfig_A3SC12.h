/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_A3SC12_H
#define HWCONFIG_A3SC12_H

// Busware CSM V2.0
// uc ATMega1284p
// Phy1: UART
// Phy2: RF - CC1101

// PORTA
// PA0  --    LEDG
// PA1  --    LEDR
// PA2  --    Loopback to PA3
// PA3  --    Loopback to PA2
// PA4  CN9
// PA5  CN8
// PA6  --
// PA7  --
// PORTB
// PB0  CN30
// PB1  CN22  RF_GDO0
// PB2  CN24
// PB3  CN25
// PB4  --    RF_NCS
// PB5  CN28  RF_MOSI
// PB6  CN27  RF_MISO
// PB7  CN29  RF_SCK
// PORTC
// PC0  CN15  SCL
// PC1  CN14  SDA
// PC2  CN13  JTAG_TCK
// PC3  CN11  JTAG_TMS
// PC4  CN10  JTAG_TDO
// PC5  CN12  JTAG_TDI
// PC6  --
// PC7  --
// PORTD
// PD0  CN3   RXD0
// PD1  CN4   TXD0
// PD2  CN23  RF_GDO2
// PD3  CN6
// PD4  CN7
// PD5  --    LEDB
// PD6  --    LEDY
// PD7  --

#ifdef __cplusplus
extern "C" {
#endif

#include "AVR/hal.h"
#include <util/delay.h>

// DIO Section
#define EXTDIO_USED                 1
#define EXTDIO_MAXPORT_NR           3                                     // Number of digital Ports
#define EXTDIO_PORTNUM2PORT         {(uint16_t)&PORTA, (uint16_t)&PORTB, (uint16_t)&PORTD}
#define EXTDIO_PORTNUM2MASK         {(uint8_t)0xC3, (uint8_t)0xF0, (uint8_t)0x83}
// End DIO Section

// PWM Section
#define EXTPWM_USED                 1
#define EXTPWM_MAXPORT_NR           3
#define EXTPWM_PORT2CFG             {0, 9, 8}           // bits 7-3 Timer, bits 2-0 Channel
#define EXTPWM_PORT2DIO             {11, 28, 29}        // Mapping PWM channel to DIO
// End PWM Section

// Analogue Inputs
#define EXTAIN_USED                 1
#define EXTAIN_MAXPORT_NR           3           // ADC4-ADC5, Vbg
#define EXTAIN_BASE_2_APIN          {4, 5, 14}
#define EXTAIN_BASE_2_DIO           {4, 5, 0xFF}
#define EXTAIN_REF                  0x0E        // Bit0 - Ext, Bit1 - Vcc, Bit2 - Int1, Bit3 - Int2
// End Analogue Inputs

// TWI Section
#define EXTTWI_USED                 1
#define TWIM_SCL_STAT()             (PINC & (1<<PC0))
// End TWI Section

// LEDs
#define LED1_On()                   PORTA |= (1<<PA0)
#define LED1_Off()                  PORTA &= ~(1<<PA0)
#define LED2_On()                   PORTA |= (1<<PA1)
#define LED2_Off()                  PORTA &= ~(1<<PA1)
#define LEDsInit()                  {DDRA |= ((1<<PA0) | (1<<PA1)); PORTA |= (1<<PA0) | (1<<PA1);}

// UART Section
#define UART0_PORT                  PORTD
#define UART0_DDR                   DDRD
#define UART0_RX_PIN                PD0
#define UART0_TX_PIN                PD1

#define UART_PHY_PORT               0
// End UART Section

// RF Section
#define RF_PORT                     PORTB
#define RF_DDR                      DDRB
#define RF_PIN                      PINB
#define RF_PIN_SS                   PB4
#define RF_PIN_MOSI                 PB5
#define RF_PIN_MISO                 PB6
#define RF_PIN_SCK                  PB7
// End RF Section

#define UART_PHY                    1
#define CC11_PHY                    2

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0xFF

#define PHY2_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY2         (PHY2_ADDR_t)0x00
#define ADDR_UNDEF_PHY2             (PHY2_ADDR_t)0xFF

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'A'
#define OD_DEV_UC_SUBTYPE           '3'
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

#endif // HWCONFIG_A3SN12_H