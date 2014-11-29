/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Extensions, serial port

#include "../config.h"

#ifdef EXTSER_USED

#include "extser.h"

#define EXTSER_FLAG_TXEN    1
#define EXTSER_FLAG_RXEN    2
#define EXTSER_FLAG_RXRDY   0x10

typedef struct
{
    uint8_t     nBaud;
    uint8_t     flags;

    uint8_t   * pRxBuf;
    uint8_t     RxHead;
    uint8_t     RxTail;

    MQ_t      * pTxBuf;
}EXTSER_VAR_t;

static const uint8_t extser2uart[] = EXTSER_PORT2UART;
#define MAX_SER_PORT    (sizeof(extser2uart)/sizeof(extser2uart[0]))
static EXTSER_VAR_t * extSerV[MAX_SER_PORT] = {NULL,};

// HAL Section, subroutines defined in hal_uart.c
void hal_uart_deinit(uint8_t port);
void hal_uart_init_hw(uint8_t port, uint8_t nBaud);
bool hal_uart_free(uint8_t port);
void hal_uart_send(uint8_t port, uint8_t len, uint8_t * pBuf);
bool hal_uart_datardy(uint8_t port);
uint8_t hal_uart_get(uint8_t port);

void serInit()
{
    uint8_t port;
    
    for(port = 0; port < MAX_SER_PORT; port++)
    {
        hal_uart_deinit(extser2uart[port]);
        if(extSerV[port] != NULL)
        {
            if(extSerV[port]->pTxBuf != NULL)
                mqFree(extSerV[port]->pTxBuf);

            mqFree(extSerV[port]);
            extSerV[port] = NULL;
        }
    }
}

// Check Index
uint8_t serCheckIdx(subidx_t * pSubidx)
{
    uint8_t type = pSubidx->Type;
    uint8_t port = pSubidx->Base/10;
    uint8_t nBaud = pSubidx->Base % 10;

    if((port >= MAX_SER_PORT) ||
       (nBaud > 4) ||
       ((type != ObjSerRx) && (type != ObjSerTx)))
        return 2;

//    if(extSerV[port] != NULL)
//        return 1;

    return 0;
}

// Read data
e_MQTTSN_RETURNS_t serReadOD(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf)
{
    uint8_t port = pSubidx->Base/10;
    
    uint8_t size = *pLen;
    uint8_t pos = 0;
    uint8_t head = extSerV[port]->RxHead;
    uint8_t tail = extSerV[port]->RxTail;

    while((pos < size) && (tail != head))
    {
        *(pBuf++) = extSerV[port]->pRxBuf[tail++];
        if(tail >= sizeof(MQ_t))
            tail = 0;
        pos++;
    }

    extSerV[port]->RxTail = tail;
    *pLen = pos;
    return MQTTSN_RET_ACCEPTED;
}

// Write data
e_MQTTSN_RETURNS_t serWriteOD(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    uint8_t port = pSubidx->Base/10;

    assert(extSerV[port] != NULL);

    if(Len > 0)
    {
        uint8_t uart = extser2uart[port];
    
        if((extSerV[port]->pTxBuf == NULL) && hal_uart_free(uart))
        {
            MQ_t * pTxBuf = mqAlloc(sizeof(MQ_t));
            if(pTxBuf == NULL)
                return MQTTSN_RET_REJ_CONG;

            memcpy(pTxBuf->raw, pBuf, Len);
            hal_uart_send(uart, Len, pTxBuf->raw);

            extSerV[port]->pTxBuf = pTxBuf;
        }
        else
            return MQTTSN_RET_REJ_CONG;
    }

    return MQTTSN_RET_ACCEPTED;
}

// Poll Procedure
uint8_t serPollOD(subidx_t * pSubidx, uint8_t sleep)
{
    uint8_t port = pSubidx->Base/10;
    
    assert(extSerV[port] != NULL);

    if(extSerV[port]->RxHead != extSerV[port]->RxTail)
    {
        if(extSerV[port]->flags & EXTSER_FLAG_RXRDY)
            return 1;
            
        extSerV[port]->flags |= EXTSER_FLAG_RXRDY;
    }
    
    return 0;
}

// Register Object
e_MQTTSN_RETURNS_t serRegisterOD(indextable_t *pIdx)
{
    uint8_t port = pIdx->sidx.Base / 10;
    uint8_t nBaud = pIdx->sidx.Base % 10;
    
    if(extSerV[port] != NULL)
    {
        if(extSerV[port]->nBaud != nBaud)
            return MQTTSN_RET_REJ_INV_ID;
    }
    else
    {
        extSerV[port] = mqAlloc(sizeof(EXTSER_VAR_t));
        if(extSerV[port] == NULL)
            return MQTTSN_RET_REJ_CONG;

        extSerV[port]->nBaud = nBaud;
        extSerV[port]->flags = 0;

        extSerV[port]->pRxBuf = NULL;
        extSerV[port]->RxHead = 0;
        extSerV[port]->RxTail = 0;

        extSerV[port]->pTxBuf = NULL;

        hal_uart_init_hw(extser2uart[port], nBaud);
    }

    if(pIdx->sidx.Type == ObjSerTx)
    {
        if((extSerV[port]->flags & EXTSER_FLAG_TXEN) || (extSerV[port]->nBaud != nBaud))
            return MQTTSN_RET_REJ_INV_ID;

        extSerV[port]->flags |= EXTSER_FLAG_TXEN;
        
        pIdx->cbWrite = &serWriteOD;
    }
    else // ObjSerRx
    {
        if((extSerV[port]->flags & EXTSER_FLAG_RXEN) || (extSerV[port]->nBaud != nBaud))
            return MQTTSN_RET_REJ_INV_ID;
            
        if(extSerV[port]->pRxBuf == NULL)
        {
            extSerV[port]->pRxBuf = mqAlloc(sizeof(MQ_t));
            if(extSerV[port]->pRxBuf == NULL)
                return MQTTSN_RET_REJ_CONG;
        }

        extSerV[port]->flags |= EXTSER_FLAG_RXEN;
        
        pIdx->cbRead = &serReadOD;
        pIdx->cbPoll = &serPollOD;
    }

    return MQTTSN_RET_ACCEPTED;
}

void serDeleteOD(subidx_t * pSubidx)
{
    uint8_t port = pSubidx->Base / 10;

    if(extSerV[port] != NULL)
    {
        if(pSubidx->Type == ObjSerTx)
        {
            extSerV[port]->flags &= ~EXTSER_FLAG_TXEN;
        }
        else
        {
            extSerV[port]->flags &= ~EXTSER_FLAG_RXEN;
            
            if(extSerV[port]->pRxBuf != NULL)
            {
                mqFree(extSerV[port]->pRxBuf);
                extSerV[port]->pRxBuf = NULL;
            }
        }
        
        if((extSerV[port]->flags & (EXTSER_FLAG_TXEN | EXTSER_FLAG_RXEN)) == 0)
        {
            hal_uart_deinit(extser2uart[port]);
            mqFree(extSerV[port]);
            extSerV[port] = NULL;
        }
    }
}

void serProc(void)
{
    uint8_t port;

    for(port = 0; port < MAX_SER_PORT; port++)
    {
        if(extSerV[port] != NULL)
        {
            uint8_t uart = extser2uart[port];
            
            if(extSerV[port]->flags & EXTSER_FLAG_TXEN)
            {
                if((extSerV[port]->pTxBuf != NULL) && (hal_uart_free(uart)))
                {
                    mqFree(extSerV[port]->pTxBuf);
                    extSerV[port]->pTxBuf = NULL;
                }
            }
            
            if(extSerV[port]->flags & EXTSER_FLAG_RXEN)
            {
                if(hal_uart_datardy(uart))
                {
                    uint8_t tmphead = extSerV[port]->RxHead + 1;
                    if(tmphead >= sizeof(MQ_t))
                        tmphead = 0;

                    if(tmphead != extSerV[port]->RxTail)
                    {
                        extSerV[port]->pRxBuf[extSerV[port]->RxHead] = hal_uart_get(uart);
                        extSerV[port]->RxHead = tmphead;
                        
                        uint8_t size;
                        
                        if(tmphead > extSerV[port]->RxTail)
                            size = tmphead - extSerV[port]->RxTail;
                        else
                            size = sizeof(MQ_t) - extSerV[port]->RxTail + tmphead;
                            
                        if(size < (sizeof(MQ_t)/2))
                            extSerV[port]->flags &= ~EXTSER_FLAG_RXRDY;
                        else
                            extSerV[port]->flags |= EXTSER_FLAG_RXRDY;
                    }
                }
            }
        }
    }
}
#endif    //  EXTSER_USED
