/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_A1EN12_H
#define HWCONFIG_A1EN12_H

// uNode Version 2.0
// uc ATMega328p
// Phy1: ENC28J60

// 0 - 7    PORTA - not exist
// PORTB
// --   PB0     --      LED
// --   PB1     --      LAN_IRQ / NC
// --   PB2     --      LAN_CSN
// --   PB3     ISP-4   LAN_MOSI
// --   PB4     ISP-1   LAN_MISO
// --   PB5     ISP-3   LAN_SCK
// --   PB6     --      OSC
// --   PB7     --      OSC
// PORT C
// 16   PC0     SV1-18  Ain0
// 17   PC1     SV1-17  Ain1
// 18   PC2     SV1-16  Ain2
// 19   PC3     SV1-15  Ain3
// 20   PC4     SV1-14  SDA
// 21   PC5     SV1-13  SCL
// --   PC6      ISP-5  RESET
// --   --      SV1-20  Ain6
// --   --      SV1-19  Ain7
// PORT D
// 24   PD0     SV1-10  RXD
// 25   PD1     SV1-9   TXD
// 26   PD2     SV1-8
// 27   PD3     SV1-7
// 28   PD4     SV1-6
// 29   PD5     SV1-5   PWM0
// 30   PD6     SV1-4   PWM1
// 31   PD7     SV1-3

#ifdef __cplusplus
extern "C" {
#endif

#include "Atmel/hal.h"
#include <util/delay.h>

// DIO Section
#define DIO_PORT_SIZE               8
#define EXTDIO_BASE_OFFSET          2
#define EXTDIO_MAXPORT_NR           2                                     // Number of digital Ports
#define EXTDIO_PORTNUM2PORT         {(uint16_t)&PORTC, (uint16_t)&PORTD}
#define EXTDIO_PORTNUM2MASK         {(uint8_t)0xE0, (uint8_t)0x00}
// End DIO Section

// Analogue Inputs
#define EXTAIN_MAXPORT_NR           9           // ADC0-ADC7, Vbg
#define EXTAIN_BASE_2_APIN          {0, 1, 2, 3, 4, 5, 6, 7, 14}
#define EXTAIN_BASE_2_DIO           {16, 17, 18, 19, 20, 21, 0xFF, 0xFF, 0xFF}
#define EXTAIN_REF                  0x06        // Bit0 - Ext, Bit1 - Vcc, Bit2 - Int1, Bit3 - Int2
// End Analogue Inputs

// TWI Section
#define TWIM_SCL_STAT()             (PINC & (1<<PC5))
// End TWI Section

// LEDs
#define LED1_On()                   PORTB &= ~(1<<PB0)
#define LED1_Off()                  PORTB |= (1<<PB0)
#define LEDsInit()                  {DDRB |= (1<<PB0); PORTB |= (1<<PB0);}

// LAN Section
#define LAN_PORT                    PORTB
#define LAN_DDR                     DDRB
#define LAN_PIN                     PINB
#define LAN_PIN_SS                  PB2
#define LAN_PIN_MOSI                PB3
#define LAN_PIN_MISO                PB4
#define LAN_PIN_SCK                 PB5

#define ENC_SELECT()                (PORTB &= ~(1<<LAN_PIN_SS))
#define ENC_RELEASE()               (PORTB |= (1<<LAN_PIN_SS))
#define inet_addr(d,c,b,a)          (((uint32_t)a<<24) | ((uint32_t)b << 16) | ((uint32_t)c<<8)  | ((uint32_t)d))
// End LAN Section

#define ENC28J60_PHY                1

#define PHY1_ADDR_t                 uint32_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)inet_addr(255,255,255,255)
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)inet_addr(255,255,255,255)

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'A'
#define OD_DEV_UC_SUBTYPE           '1'
#define OD_DEV_PHY1                 'E'
#define OD_DEV_PHY2                 'n'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '2'
#define OD_ADDR_TYPE                objUInt32
#define OD_DEV_MAC                  {0x00,0x04,0xA3,0x00,0x00,0x05}   // MAC MSB->LSB
#define OD_DEF_IP_ADDR              inet_addr(192,168,10,205)
#define OD_DEF_IP_MASK              inet_addr(255,255,255,0)
//#define OD_DEF_IP_ROUTER            inet_addr(192,168,10,1)
//#define OD_DEF_IP_BROKER            inet_addr(192,168,20,8)

#include "../PHY/ENC28J60/enc28j60_phy.h"

#define PHY1_Init                   ENC28J60_Init
#define PHY1_Send                   ENC28J60_Send
#define PHY1_Get                    ENC28J60_Get
#define PHY1_NodeId                 objIPAddr
#define PHY1_GateId                 objIPBroker

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_A1EN12_H