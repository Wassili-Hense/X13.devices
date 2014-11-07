/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#include "config.h"
#include "ext.h"
#include "EXT/thermostat.h"

#ifdef EXTDIO_USED
#include "EXT/extdio.h"
#endif  //  EXTDIO_USED

#ifdef EXTAIN_USED
#include "EXT/extain.h"
#endif  //  EXTAIN_USED

#ifdef EXTTWI_USED
#include "EXT/exttwi.h"
#endif  //  EXTTWI_USED

// Initialise extensions
void extInit(void)
{
#ifdef EXTDIO_USED
    dioInit();
#endif  //  EXTDIO_USED

#ifdef EXTAIN_USED
    ainInit();
#endif  //  EXTAIN_USED

#ifdef EXTTWI_USED
    twiInit();
#endif
  tsConfig();
}

// Check Subindex: 0 - free / 1 - busy / 2 - invalid
uint8_t extCheckIdx(subidx_t * pSubidx)
{
    switch(pSubidx->Place)
    {
#ifdef EXTDIO_USED
        case objDin:
        case objDout:
#ifdef EXTPWM_USED
        case objPWM:
#endif  //  EXTPWM_USED
            return dioCheckIdx(pSubidx);
#endif  //  EXTDIO_USED

#ifdef EXTAIN_USED
        case objAin:
            return ainCheckIdx(pSubidx);
#endif  //  EXTAIN_USED

        default:
            break;
  }
  return 2;
}

// Register Object
e_MQTTSN_RETURNS_t extRegisterOD(indextable_t * pIdx)
{
    pIdx->cbRead = NULL;
    pIdx->cbWrite = NULL;
    pIdx->cbPoll = NULL;

    switch(pIdx->sidx.Place)
    {
#ifdef EXTDIO_USED
        case objDin:        // Digital(bool) Input's
        case objDout:       // Digital(bool) Output's
            return dioRegisterOD(pIdx);
#ifdef EXTPWM_USED
        case objPWM:        // PWM
            return pwmRegisterOD(pIdx);
#endif  //  EXTPWM_USED
#endif  //  EXTDIO_USED

#ifdef EXTAIN_USED
        case objAin:        // Analog(int16_t) Input's
            return ainRegisterOD(pIdx);
#endif  //  EXTAIN_USED
        default:
            break;
    }
    return MQTTSN_RET_REJ_NOT_SUPP;
}

// Delete Object
void extDeleteOD(subidx_t * pSubidx)
{
    // Delete Objects
    switch(pSubidx->Place)
    {
#ifdef EXTDIO_USED
        case objDin:
        case objDout:
#ifdef EXTPWM_USED
        case objPWM:
#endif  //  EXTPWM_USED        
            dioDeleteOD(pSubidx);
            break;
#endif  //  EXTDIO_USED

#ifdef EXTAIN_USED
        case objAin:
            ainDeleteOD(pSubidx);
            break;
#endif  //  EXTAIN_USED
        default:
            break;
    }
}

void extProc(void)
{
#ifdef EXTDIO_USED
    dioProc();
#endif  //  EXTDIO_USED

#ifdef EXTAIN_USED
    ainProc();
#endif  //  EXTAIN_USED
  tsTick();
}
