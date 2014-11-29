#include "../../config.h"

#ifdef EXTDIO_USED

#include "../../EXT/extdio.h"

const uint16_t dio_portnum2port[]  = EXTDIO_PORTNUM2PORT;

void hal_dio_configure(uint8_t PortNr, uint8_t Mask, eDIOmode_t Mode)
{
    uint16_t base = dio_portnum2port[PortNr];

    uint8_t *pPORT;
    pPORT = (uint8_t *)base;
    uint8_t *pDDR;
    pDDR = (uint8_t *)(base - 1);

    switch(Mode)
    {
        case DIO_MODE_IN_PU:
            *pPORT |= Mask;
            *pDDR  &= ~Mask;
            break;
        case DIO_MODE_OUT:
        case DIO_MODE_PWM:
            *pDDR  |= Mask;
            break;
        default:
            *pPORT &= ~Mask;
            *pDDR  &= ~Mask;
        break;
    }
}

uint8_t hal_dio_read(uint8_t PortNr)
{
    uint8_t *pPIN;
    pPIN = (uint8_t *)(dio_portnum2port[PortNr] - 2);
    return *pPIN;
}

void hal_dio_set(uint8_t PortNr, uint8_t Mask)
{
    uint8_t * pPORT = (uint8_t *)dio_portnum2port[PortNr];
    *pPORT |= Mask;
}

void hal_dio_reset(uint8_t PortNr, uint8_t Mask)
{
    uint8_t * pPORT = (uint8_t *)dio_portnum2port[PortNr];
    *pPORT &= ~Mask;
}

#endif  //  EXTDIO_USED