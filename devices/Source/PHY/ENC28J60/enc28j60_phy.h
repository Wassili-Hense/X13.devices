/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _ENC28J60_PHY_H
#define _ENC28J60_PHY_H

#ifdef __cplusplus
extern "C" {
#endif

#define LAN_NODE

#define inet_addr(d,c,b,a)          (((uint32_t)a<<24) | ((uint32_t)b << 16) | ((uint32_t)c<<8)  | ((uint32_t)d))

// API Section
void ENC28J60_Init(void);
void ENC28J60_Send(void *pBuf);
void * ENC28J60_Get(void);
void * ENC28J60_GetAddr(void);

#ifdef __cplusplus
}
#endif

#endif  //  _UART_PHY_H
