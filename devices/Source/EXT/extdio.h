/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _EXTDIO_H
#define _EXTDIO_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  DIO_MODE_IN_FLOAT = 0,
  DIO_MODE_IN_PD,
  DIO_MODE_IN_PU,
  DIO_MODE_OUT,
  DIO_MODE_PWM,
  DIO_MODE_AIN
}eDIOmode_t;

void dioInit(void);
uint8_t dioCheckIdx(subidx_t * pSubidx);
e_MQTTSN_RETURNS_t dioRegisterOD(indextable_t *pIdx);
e_MQTTSN_RETURNS_t pwmRegisterOD(indextable_t *pIdx);
void dioDeleteOD(subidx_t * pSubidx);
void dioProc(void);

#ifdef __cplusplus
}
#endif

#endif  //  _EXTDIO_H
