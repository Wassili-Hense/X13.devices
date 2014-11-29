/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _UART_PHY_H
#define _UART_PHY_H

// API Section
void UART_Init(void);
void UART_Send(void *pBuf);
void * UART_Get(void);
void * UART_GetAddr(void);

#endif  //  _UART_PHY_H
