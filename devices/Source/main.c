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
#include "diag.h"

static volatile uint8_t SystemTickCnt;

int main(void)
{
    // Initialise System Hardware
    INIT_SYSTEM();
    // Initialise Memory manager
    mqInit();
    // Initialise Object's Dictionary
    InitOD();
    // Initialise PHY's
    PHY1_Init();
#ifdef PHY2_ADDR_t
    PHY2_Init();
#endif  //  PHY2_ADDR_t
    // Initialize MQTTSN
    MQTTSN_Init();
    // Initialise optional components
#ifdef  LEDsInit
    LEDsInit();
#endif  //  LEDsInit
#ifdef DIAG_USED
    DIAG_Init();
#endif  //  USE_DIAG

    SystemTickCnt = 0;

    StartSheduler();
  
    while(1)
    {
        if(SystemTickCnt)
        {
            SystemTickCnt--;
            OD_Poll();

            MQTTSN_Poll();
#ifdef DIAG_USED
            DIAG_Poll();
#endif  //  USE_DIAG
        }

        MQ_t * pBuf;
        pBuf = PHY1_Get();
        if(pBuf != NULL)
        {
            mqttsn_parser_phy1(pBuf);
        }

#ifdef PHY2_Get
        pBuf = PHY2_Get();
        if(pBuf != NULL)
        {
            mqttsn_parser_phy2(pBuf);
        }
#endif  //  PHY2_Get
    }
}

#ifdef LED1_On
static uint16_t LED1_mask = 0xFFFF;
void SetLED1mask(uint16_t mask)
{
    LED1_mask = mask;
}
#endif  //  LED1_On

#ifdef LED2_On
static uint16_t LED2_mask = 0xFFFF;
void SetLED2mask(uint16_t mask)
{
    LED2_mask = mask;
}
#endif  //  LED2_On

static volatile uint32_t tick_count = 0;

uint32_t inline GetTickCounter()
{
    return tick_count;
}

void SystemTick(void)
{
    tick_count++;
    SystemTickCnt++;

#ifdef LED1_On
    if(LED1_mask & 1)
        LED1_On();
    else
        LED1_Off();

    LED1_mask >>= 1;
#endif  //  LED1_On

#ifdef LED2_On
    if(LED2_mask & 1)
        LED2_On();
    else
        LED2_Off();

    LED2_mask >>= 1;
#endif  //  LED2_On
}
