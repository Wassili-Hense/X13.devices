/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Extensions TWI/I2C

#include "../config.h"

#ifdef EXTTWI_USED

#include "exttwi.h"

// Global variable used in HAL
volatile TWI_QUEUE_t  * pTwi_exchange = NULL;

// local queues
static Queue_t  twi_tx_queue = {NULL, NULL, 0, 0};

// HAL
bool hal_twi_configure(uint8_t enable);
void hal_twi_tick(void);

e_MQTTSN_RETURNS_t twiReadOD(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf)
{
    assert(pTwi_exchange != NULL);

    *pLen = pTwi_exchange->frame.read + sizeof(TWI_FRAME_t);
    memcpy(pBuf, (void *)&pTwi_exchange->frame, *pLen);
    mqFree((void *)pTwi_exchange);
    pTwi_exchange = NULL;
    return MQTTSN_RET_ACCEPTED;
}

e_MQTTSN_RETURNS_t twiWriteOD(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    if(Len < sizeof(TWI_FRAME_t))
        return MQTTSN_RET_REJ_NOT_SUPP;

    TWI_QUEUE_t * pQueue = mqAlloc(sizeof(MQ_t));
    if(pQueue == NULL)
        return MQTTSN_RET_REJ_CONG;

    memcpy(&pQueue->frame, pBuf, Len);
    
    pQueue->frame.access &= (TWI_WRITE | TWI_READ);
    pQueue->frame.write = (Len - sizeof(TWI_FRAME_t));
    if(pQueue->frame.write != 0)
    {
        pQueue->frame.access |= TWI_WRITE;
    }
    if(pQueue->frame.read != 0)
    {
        pQueue->frame.access |= TWI_READ;
    }

    if((pQueue->frame.read > (MQTTSN_MSG_SIZE - sizeof(TWI_FRAME_t) - MQTTSN_SIZEOF_MSG_PUBLISH)) ||
       ((pQueue->frame.access & (TWI_WRITE | TWI_READ)) == 0))
    {
        mqFree(pQueue);
        return MQTTSN_RET_REJ_NOT_SUPP;
    }

    if(!mqEnqueue(&twi_tx_queue, pQueue))
    {
        mqFree(pQueue);
        return MQTTSN_RET_REJ_CONG;
    }

    return MQTTSN_RET_ACCEPTED;
}

uint8_t twiPollOD(subidx_t * pSubidx, uint8_t sleep)
{
    if(pTwi_exchange != NULL)
    {
        uint8_t access = pTwi_exchange->frame.access;
        if((access & (TWI_ERROR | TWI_SLANACK | TWI_WD)) != 0)      // Error state
        {
            return 1;
        }
        else if(access & TWI_RDY)
        {
            if(pTwi_exchange->frame.read != 0)
            {
                return 1;
            }
            else
            {
                mqFree((void *)pTwi_exchange);
                pTwi_exchange = NULL;
            }
        }
        else
        {
            hal_twi_tick();
        }
    }
    else if(twi_tx_queue.Size != 0)
    {
        pTwi_exchange = mqDequeue(&twi_tx_queue);
    }

    return 0;
}

void twiInit()
{
    if(!hal_twi_configure(1))           // Enable
        return;

    if(pTwi_exchange != NULL)
    {
        mqFree((void *)pTwi_exchange);
        pTwi_exchange = NULL;
    }

    // Register variable Ta0
    indextable_t * pIndex = getFreeIdxOD();
    if(pIndex == NULL)
    {
        hal_twi_configure(0);
        return;
    }

    pIndex->cbRead     = &twiReadOD;
    pIndex->cbWrite    = &twiWriteOD;
    pIndex->cbPoll     = &twiPollOD;
    pIndex->sidx.Place = objTWI;        // TWI object
    pIndex->sidx.Type  = objArray;      // Variable Type -  Byte Array
    pIndex->sidx.Base  = 0;             // Device address
}
#endif    //  EXTTWI_USED
