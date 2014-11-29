/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _CC11_PHY_H
#define _CC11_PHY_H

#define OD_DEFAULT_GROUP        0x2DD4

// 433 MHz
#if (RF_BASE_FREQ > 433050000UL) && (RF_BASE_FREQ < 434790000UL)
#define OD_DEFAULT_CHANNEL      ((RF_BASE_FREQ - 433000000UL)/25000)

// 868 MHz
#elif (RF_BASE_FREQ > 868000000UL) && (RF_BASE_FREQ < 870000000UL)
#define OD_DEFAULT_CHANNEL      ((RF_BASE_FREQ - 868000000UL)/25000)

// Bad Frequency
#else
#error  RF_BASE_FREQ does not belond to ISM band
#endif  // RF_BASE_FREQ

#ifdef RF_ADDR_t
#undef RF_ADDR_t
#warning redefine RF_ADDR_t in cc11_phy.h
#endif  //  RF_ADDR_t

#ifdef ADDR_UNDEF_RF
#undef ADDR_UNDEF_RF
#warning redefine ADDR_UNDEF_RF in cc11_phy.h
#endif  //  ADDR_UNDEF_RF

#define RF_ADDR_t               uint8_t
#define ADDR_UNDEF_RF           (RF_ADDR_t)0xFF

// API Section
void CC11_Init(void);
void CC11_Send(void *pBuf);
void * CC11_Get(void);
uint8_t CC11_GetRSSI(void);
void * CC11_GetAddr(void);

#endif  //  _CC11_PHY_H
